#include "pyro_uav_gimbal.h"

#include <bits/stl_algo.h>

#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_ins.h"

namespace pyro
{
uav_gimbal_t::uav_gimbal_t()
    : module_base_t("gimbal")
{
    gimbal_ctx = {};
    gimbal_ins = ins_drv_t::get_instance();
    gimbal_ins->init();

    gimbal_ctx.auto_ctx.auto_enable = false;
}

status_t uav_gimbal_t::_init()
{
    gimbal_ctx.cfg.motor_ctx.yaw_motor = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_5,can_hub_t::can2);
    gimbal_ctx.cfg.motor_ctx.pitch_motor = new dm_motor_drv_t(0x01,0x00,can_hub_t::can2);

    static_cast<dm_motor_drv_t *>(gimbal_ctx.cfg.motor_ctx.pitch_motor)->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.cfg.motor_ctx.pitch_motor)->set_rotate_range(-30, 30);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.cfg.motor_ctx.pitch_motor)->set_torque_range(-7, 7);

    //遥控器pid
    gimbal_ctx.cfg.pid_ctx.yaw_position_pid = new pid_t(21.0f,0.01f,0.000f,1.0f,
                15.0f,60,30,4);
    gimbal_ctx.cfg.pid_ctx.yaw_speed_pid = new pid_t(2.85f,0.001f,0.0002f,0.5f,
                3.0f,30,20,4);

    gimbal_ctx.cfg.pid_ctx.pitch_position_pid = new pid_t(19.5f,0.009f,0.025f,0.5f,
                10.0f,40,20,4);
    gimbal_ctx.cfg.pid_ctx.pitch_speed_pid = new pid_t(1.156f,0.000838f,0.0004f,0.5f,
                7.0f,40,20,4);


    //自瞄pid
    gimbal_ctx.cfg.pid_ctx.auto_yaw_position_pid = new pid_t(17.68f,0.81f,0.0004,1.8f,
                8.0f,90,50,4);
    gimbal_ctx.cfg.pid_ctx.auto_pitch_position_pid = new pid_t(15.28f,0.285f,0.00029f,1.6f,
                5.0f,80,50,4);
    gimbal_ctx.cfg.pid_ctx.auto_yaw_speed_pid = new pid_t(4.185f,0.782f,0.00075f,1.8f,
                3.0f,100,60,4);
    gimbal_ctx.cfg.pid_ctx.auto_pitch_speed_pid = new pid_t(1.18f,0.195f,0.00024f,1.6f,
                7.0f,80,50,4);

    return PYRO_OK;
}

void uav_gimbal_t::_update_feedback()
{
    gimbal_ctx.auto_ctx.auto_enable = gimbal_ctx.cmd->auto_flag;

    gimbal_ctx.cfg.motor_ctx.yaw_motor->update_feedback();
    gimbal_ctx.cfg.motor_ctx.pitch_motor->update_feedback();

    float current_yaw_angle =
     gimbal_ctx.cfg.motor_ctx.yaw_motor->get_current_position() - YAW_OFFSET_RAD;
    normalize_angle(current_yaw_angle);
    gimbal_ctx.data.yaw_motor_angle = current_yaw_angle;

    // gimbal_ctx.data.pitch_motor_angle = gimbal_ctx.cfg.motor_ctx.pitch_motor->get_current_position();
    float current_pitch_angle =
        gimbal_ctx.cfg.motor_ctx.pitch_motor->get_current_position() - PITCH_OFFSET_RAD;
    normalize_angle(current_pitch_angle);
    gimbal_ctx.data.pitch_motor_angle = current_pitch_angle;
    gimbal_ctx.data.pitch_motor_speed = gimbal_ctx.cfg.motor_ctx.pitch_motor->get_current_rotate();

    //读取IMU获得当前角度
    gimbal_ins->get_rads_n(&gimbal_ctx.data._current_imu_yaw_angle,
                                          &gimbal_ctx.data._current_imu_pitch_angle,
                                          &gimbal_ctx.data._current_imu_roll_angle);

    gimbal_ins->get_gyro_b(&gimbal_ctx.data._current_imu_yaw_speed,
                                          &gimbal_ctx.data._current_imu_pitch_speed,
                                          &gimbal_ctx.data._current_imu_roll_speed);

    //对current_imu_angle作归一化
    //因为电机安装位置的原因 yaw轴电机和imu角度减小的方向相反 Motor ↑  IMU ↓ 所以和是一个常数
    gimbal_ctx.data.yaw_real_min_limit_angle = gimbal_ctx.data._current_imu_yaw_angle + gimbal_ctx.data.yaw_motor_angle - yaw_motor_max_value;
    gimbal_ctx.data.yaw_real_max_limit_angle = gimbal_ctx.data._current_imu_yaw_angle + gimbal_ctx.data.yaw_motor_angle - yaw_motor_min_value;

    //pitch轴电机角度减小的方向和imu角度减小的方向相同 Motor ↑ IMU ↑ 所以二者的差是一个常数
    // gimbal_ctx.data.pitch_real_min_limit_angle = gimbal_ctx.data._current_imu_pitch_angle - gimbal_ctx.data.pitch_motor_angle + pitch_motor_min_value;
    // gimbal_ctx.data.pitch_real_max_limit_angle = gimbal_ctx.data._current_imu_pitch_angle - gimbal_ctx.data.pitch_motor_angle + pitch_motor_max_value;
    // 定义滤波系数 (0 < alpha < 1)
    // alpha 越小，滤波效果越强，响应越慢
    // 建议值：0.1 ~ 0.3
    static const float PITCH_LIMIT_FILTER_ALPHA = 0.0159f;

    // 上一次的滤波值（需要静态存储或放在结构体中）
    // 建议在 gimbal_ctx.data 中添加这两个变量：
    // gimbal_ctx.data.pitch_real_min_limit_angle_filtered
    // gimbal_ctx.data.pitch_real_max_limit_angle_filtered

    // 计算原始值
    float raw_min = gimbal_ctx.data._current_imu_pitch_angle
                    - gimbal_ctx.data.pitch_motor_angle
                    + pitch_motor_min_value;
    float raw_max = gimbal_ctx.data._current_imu_pitch_angle
                    - gimbal_ctx.data.pitch_motor_angle
                    + pitch_motor_max_value;

    // 一阶低通滤波 (低通滤波 = alpha * 当前原始值 + (1 - alpha) * 上一次滤波值)
    gimbal_ctx.data.pitch_real_min_limit_angle =
        PITCH_LIMIT_FILTER_ALPHA * raw_min
        + (1.0f - PITCH_LIMIT_FILTER_ALPHA) * gimbal_ctx.data.pitch_real_min_limit_angle_filtered;

    gimbal_ctx.data.pitch_real_max_limit_angle =
        PITCH_LIMIT_FILTER_ALPHA * raw_max
        + (1.0f - PITCH_LIMIT_FILTER_ALPHA) * gimbal_ctx.data.pitch_real_max_limit_angle_filtered;

    // 更新滤波状态值
    gimbal_ctx.data.pitch_real_min_limit_angle_filtered = gimbal_ctx.data.pitch_real_min_limit_angle;
    gimbal_ctx.data.pitch_real_max_limit_angle_filtered = gimbal_ctx.data.pitch_real_max_limit_angle;
}

void uav_gimbal_t::_fsm_execute()
{
    gimbal_ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::ACTIVE == gimbal_ctx.cmd->mode)
        main_fsm.change_state(&state_active);
    else if (cmd_base_t::mode_t::PASSIVE == gimbal_ctx.cmd->mode)
        main_fsm.change_state(&state_passive);

    main_fsm.execute(this);
}

void uav_gimbal_t::rc_gimbal_control(gimbal_ctx_t *ctx)
{
    ctx->data._target_yaw_speed = ctx->cfg.pid_ctx.yaw_position_pid->calculate(
            ctx->data._target_yaw_angle,  ctx->data._current_imu_yaw_angle);

    ctx->data._target_pitch_speed = ctx->cfg.pid_ctx.pitch_position_pid->calculate(
             ctx->data._target_pitch_angle, ctx->data._current_imu_pitch_angle);

    ctx->data._output_yaw_torque = - ctx->cfg.pid_ctx.yaw_speed_pid->calculate(
            ctx->data._target_yaw_speed, ctx->data._current_imu_yaw_speed);

    constexpr float YAW_DEADBAND_RADPS = 0.04f;

    if (ctx->data._target_yaw_speed > YAW_DEADBAND_RADPS)
    {
        ctx->data._output_yaw_torque += -0.1f;
    }
    else if (ctx->data._target_yaw_speed < -YAW_DEADBAND_RADPS)
    {
        ctx->data._output_yaw_torque += 0.1f;
    }

    ctx->data._output_yaw_torque = std::clamp(ctx->data._output_yaw_torque, -3.0f, 3.0f);
    float angle = ctx->cfg.motor_ctx.pitch_motor->get_current_position();
    // 拟合后的动态系数：k = 0.9167 * angle - 0.175(用最小二乘法拟合的)
    // 这样当 angle 减小时（低头），系数会变得更负，补偿更强
    ctx->data.gravity_k = 0.9167f * angle - 0.175f;
    // 加上安全限幅，防止计算出的系数超出物理极限
    if (ctx->data.gravity_k < -0.8f) ctx->data.gravity_k = -0.8f;
    if (ctx->data.gravity_k > -0.1f) ctx->data.gravity_k = -0.1f;

    // 最终输出
    ctx->data.gravity_compensate = ctx->data.gravity_k * cosf(angle);

    // ctx->data._output_pitch_torque = ctx->data.gravity_compensate;
    ctx->data._output_pitch_torque = ctx->cfg.pid_ctx.pitch_speed_pid->calculate(
        ctx->data._target_pitch_speed,ctx->data._current_imu_pitch_speed) + ctx->data.gravity_compensate;

    // ctx->data._output_pitch_torque = ctx->cfg.pid_ctx.pitch_speed_pid->calculate(
    //     ctx->data._target_pitch_speed,ctx->data._current_imu_pitch_speed);
}

void uav_gimbal_t::auto_aim_gimbal_control(gimbal_ctx_t *ctx)
{
    ctx->data._target_yaw_speed = ctx->cfg.pid_ctx.auto_yaw_position_pid->calculate(
            ctx->data._target_yaw_angle,  ctx->data._current_imu_yaw_angle);

    ctx->data._target_pitch_speed = ctx->cfg.pid_ctx.auto_pitch_position_pid->calculate(
             ctx->data._target_pitch_angle, ctx->data._current_imu_pitch_angle);

    ctx->data._output_yaw_torque = - ctx->cfg.pid_ctx.auto_yaw_speed_pid->calculate(
            ctx->data._target_yaw_speed, ctx->data._current_imu_yaw_speed);

    ctx->data._output_pitch_torque = ctx->cfg.pid_ctx.pitch_speed_pid->calculate(
        ctx->data._target_pitch_speed,ctx->data._current_imu_pitch_speed);
}

void uav_gimbal_t::send_motor_command(const gimbal_ctx_t *ctx)
{
     ctx->cfg.motor_ctx.yaw_motor->send_torque(ctx->data._output_yaw_torque);
     // ctx->cfg.motor_ctx.yaw_motor->send_torque(0);

     ctx->cfg.motor_ctx.pitch_motor->send_torque(ctx->data._output_pitch_torque);
    // ctx->cfg.motor_ctx.pitch_motor->send_torque(0);
}

void uav_gimbal_t::normalize_angle(float& angle)
{
    if (angle > PI)
    {
        angle -= 2.0f * PI;
    }
    if (angle < -PI)
    {
        angle += 2.0f * PI;
    }
}

//计算垂直速度 然后加入速度环前馈
void uav_gimbal_t::feedforward_compensation(float *yaw_compensation,float *pitch_compensation)
{
    gimbal_ctx.feedforward_data.last_yaw_target_angle = gimbal_ctx.feedforward_data.now_yaw_target_angle;
    gimbal_ctx.feedforward_data.now_yaw_target_angle = gimbal_ctx.cmd->yaw_target_angle;

    gimbal_ctx.feedforward_data.last_pitch_target_angle = gimbal_ctx.feedforward_data.now_pitch_target_angle;
    gimbal_ctx.feedforward_data.now_pitch_target_angle = gimbal_ctx.cmd->pitch_target_angle;

     *yaw_compensation = (gimbal_ctx.feedforward_data.now_yaw_target_angle -
     gimbal_ctx.feedforward_data.last_yaw_target_angle) / gimbal_ctx.feedforward_data.dt * gimbal_ctx.feedforward_data.yaw_kff;

    *pitch_compensation = (gimbal_ctx.feedforward_data.now_pitch_target_angle -
     gimbal_ctx.feedforward_data.last_pitch_target_angle) / gimbal_ctx.feedforward_data.dt * gimbal_ctx.feedforward_data.pitch_kff;
}
}
