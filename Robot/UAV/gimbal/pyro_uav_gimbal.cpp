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
    static_cast<dm_motor_drv_t *>(gimbal_ctx.cfg.motor_ctx.pitch_motor)->set_torque_range(-10, 10);

    //遥控器pid 4.16版
    // uint8_t improve =     pid_t::improvement_t::INTEGRAL_LIMIT |
    //                       pid_t::improvement_t::CHANGING_INTEGRATION_RATE |
    //                       pid_t::improvement_t::OUTPUT_FILTER |
    //                       pid_t::improvement_t::DERIVATIVE_FILTER;
    //
    // gimbal_ctx.cfg.pid_ctx.yaw_position_pid = new pid_t(
    //         15.0f,1.0f,0.003f,25.4f,0.01f,0.001f,0.2f,0.05f,
    //         60.0f,30.0f,4,improve);

    //上次遥控器控制不抖的pid
    // gimbal_ctx.cfg.pid_ctx.yaw_position_pid = new pid_t(21.4f,0.008f,0.0005,1.0f,
    //         10.0f,60,25,4);
    // gimbal_ctx.cfg.pid_ctx.yaw_speed_pid = new pid_t(2.38f,0.0025f,0.0008f,1.0f,
    //             3.0f,60,20,4);
    //这是那天晚上调好的pid
    gimbal_ctx.cfg.pid_ctx.yaw_position_pid = new pid_t(21.0f,0.01f,0.000f,1.0f,
               15.0f,60,30,4);
    gimbal_ctx.cfg.pid_ctx.yaw_speed_pid = new pid_t(2.85f,0.001f,0.0002f,0.5f,
                3.0f,30,20,4);

    //那天不抖的pid
    gimbal_ctx.cfg.pid_ctx.pitch_position_pid = new pid_t(10.0f,0.01f,0.004f,0.8f,
                12.0f,60,35,4);
    gimbal_ctx.cfg.pid_ctx.pitch_speed_pid = new pid_t(1.0f,0.008f,0.0004f,0.8f,
                7.0f,50,30,4);
    //那天调好的pid
    // gimbal_ctx.cfg.pid_ctx.pitch_position_pid = new pid_t(19.5f,0.009f,0.025f,0.5f,
    //             10.0f,40,20,4);
    // gimbal_ctx.cfg.pid_ctx.pitch_speed_pid = new pid_t(1.156f,0.000838f,0.0004f,0.5f,
    //             7.0f,40,20,4);

    //自瞄pid 4/23版
    gimbal_ctx.cfg.pid_ctx.auto_yaw_position_pid = new pid_t(15.0f,0.01f,0.000f,1.0f,
                15.0f,60,30,4);
    gimbal_ctx.cfg.pid_ctx.auto_yaw_speed_pid = new pid_t(2.05f,0.001f,0.0002f,0.5f,
                3.0f,30,20,4);

    //进自瞄不抖的pid
    // gimbal_ctx.cfg.pid_ctx.auto_pitch_position_pid = new pid_t(12.0f,0.0012f,0.06f,0.7f,
    //             10.0f,50,30,4);
    // gimbal_ctx.cfg.pid_ctx.auto_pitch_speed_pid = new pid_t(1.0f,0.001f,0.04f,0.7f,
    //             7.0f,50,30,4);
    gimbal_ctx.cfg.pid_ctx.pitch_position_pid = new pid_t(19.5f,0.009f,0.025f,0.5f,
                10.0f,40,20,4);
    gimbal_ctx.cfg.pid_ctx.pitch_speed_pid = new pid_t(1.156f,0.000838f,0.0004f,0.5f,
                7.0f,40,20,4);
    return PYRO_OK;
}

void uav_gimbal_t::_update_feedback()
{
    gimbal_ctx.auto_ctx.auto_enable = gimbal_ctx.cmd->auto_flag;

    gimbal_ctx.cfg.motor_ctx.yaw_motor->update_feedback();
    gimbal_ctx.cfg.motor_ctx.pitch_motor->update_feedback();

    // gimbal_ctx.data.yaw_motor_angle = gimbal_ctx.cfg.motor_ctx.yaw_motor->get_current_position();
    float current_yaw_angle =
     gimbal_ctx.cfg.motor_ctx.yaw_motor->get_current_position() - YAW_OFFSET_RAD;
    normalize_angle(current_yaw_angle);
    gimbal_ctx.data.yaw_motor_angle = current_yaw_angle;

    gimbal_ctx.data.pitch_motor_angle = gimbal_ctx.cfg.motor_ctx.pitch_motor->get_current_position();

    //读取IMU获得当前角度
    gimbal_ins->get_rads_n(&gimbal_ctx.data._current_imu_yaw_angle,
                                          &gimbal_ctx.data._current_imu_pitch_angle,
                                          &gimbal_ctx.data._current_imu_roll_angle);

    gimbal_ins->get_gyro_b(&gimbal_ctx.data._current_imu_yaw_speed,
                                          &gimbal_ctx.data._current_imu_pitch_speed,
                                          &gimbal_ctx.data._current_imu_roll_speed);

    if (gimbal_ctx.data._current_imu_pitch_speed < 0.02f && gimbal_ctx.data._current_imu_pitch_speed > -0.02f)
    {
        gimbal_ctx.data._current_imu_pitch_speed = 0.0f;
    }

    //对current_imu_angle作归一化
    //因为电机安装位置的原因 yaw轴电机和imu角度减小的方向相反 Motor ↑  IMU ↓ 所以和是一个常数
    gimbal_ctx.data.yaw_real_min_limit_angle = gimbal_ctx.data._current_imu_yaw_angle + gimbal_ctx.data.yaw_motor_angle - yaw_motor_max_value;
    gimbal_ctx.data.yaw_real_max_limit_angle = gimbal_ctx.data._current_imu_yaw_angle + gimbal_ctx.data.yaw_motor_angle - yaw_motor_min_value;

    //pitch轴电机角度减小的方向和imu角度减小的方向相同 Motor ↑ IMU ↑ 所以二者的差是一个常数
    // gimbal_ctx.data.pitch_real_min_limit_angle = gimbal_ctx.data._current_imu_pitch_angle - gimbal_ctx.data.pitch_motor_angle + pitch_motor_min_value;
    // gimbal_ctx.data.pitch_real_max_limit_angle = gimbal_ctx.data._current_imu_pitch_angle - gimbal_ctx.data.pitch_motor_angle + pitch_motor_max_value;
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

    ctx->data._output_pitch_torque = ctx->cfg.pid_ctx.pitch_speed_pid->calculate(
        ctx->data._target_pitch_speed,ctx->data._current_imu_pitch_speed) + ctx->data.gravity_compensate;
    // ctx->data._output_pitch_torque = ctx->cfg.pid_ctx.pitch_speed_pid->calculate(
    // ctx->data._target_pitch_speed,ctx->data._current_imu_pitch_speed);
}

void uav_gimbal_t::auto_aim_gimbal_control(gimbal_ctx_t *ctx)
{
    ctx->data._target_yaw_speed = ctx->cfg.pid_ctx.auto_yaw_position_pid->calculate(
            ctx->data._target_yaw_angle,  ctx->data._current_imu_yaw_angle) + ctx->feedforward_data.yaw_ff;

    ctx->data._output_yaw_torque = - ctx->cfg.pid_ctx.auto_yaw_speed_pid->calculate(
            ctx->data._target_yaw_speed, ctx->data._current_imu_yaw_speed);

    ctx->data._target_pitch_speed = ctx->cfg.pid_ctx.auto_pitch_position_pid->calculate(
             ctx->data._target_pitch_angle, ctx->data._current_imu_pitch_angle) + ctx->feedforward_data.pitch_ff;

    ctx->data._output_pitch_torque = ctx->cfg.pid_ctx.auto_pitch_speed_pid->calculate(
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
    void uav_gimbal_t::feedforward_compensation()
{
    // 1. 设置滤波系数 (可以根据实际效果调整，alpha 越小越平滑但延迟越高)
    constexpr float angle_alpha = 0.3f; // 角度滤波系数
    constexpr float v_alpha = 0.2f;     // 速度滤波系数

    // 2. 对输入的原始目标角度进行低通滤波
    // 这样处理后，now_yaw_target_angle 就不再跳变了
    gimbal_ctx.feedforward_data.last_yaw_target_angle = gimbal_ctx.feedforward_data.now_yaw_target_angle;
    // 增量滤波公式：新值 = 旧值 + alpha * (输入 - 旧值)
    // 注意：如果是弧度，这里需要处理 -PI 到 PI 的跳变，这里假设是角度或已处理
    gimbal_ctx.feedforward_data.now_yaw_target_angle += angle_alpha * (gimbal_ctx.cmd->yaw_target_angle - gimbal_ctx.feedforward_data.now_yaw_target_angle);

    gimbal_ctx.feedforward_data.last_pitch_target_angle = gimbal_ctx.feedforward_data.now_pitch_target_angle;
    gimbal_ctx.feedforward_data.now_pitch_target_angle += angle_alpha * (gimbal_ctx.cmd->pitch_target_angle - gimbal_ctx.feedforward_data.now_pitch_target_angle);

    // 3. 基于平滑后的角度计算原始速度
    float raw_yaw_v = (gimbal_ctx.feedforward_data.now_yaw_target_angle -
                       gimbal_ctx.feedforward_data.last_yaw_target_angle) / gimbal_ctx.feedforward_data.dt;

    float raw_pitch_v = (gimbal_ctx.feedforward_data.now_pitch_target_angle -
                         gimbal_ctx.feedforward_data.last_pitch_target_angle) / gimbal_ctx.feedforward_data.dt;

    // 4. 对速度再次进行滤波 (二次滤波，确保前馈输出极其丝滑)
    gimbal_ctx.feedforward_data.filtered_yaw_v = v_alpha * raw_yaw_v + (1.0f - v_alpha) * gimbal_ctx.feedforward_data.filtered_yaw_v;
    gimbal_ctx.feedforward_data.filtered_pitch_v = v_alpha * raw_pitch_v + (1.0f - v_alpha) * gimbal_ctx.feedforward_data.filtered_pitch_v;

    // 5. 输出前馈量并限幅
    gimbal_ctx.feedforward_data.yaw_ff = std::clamp(gimbal_ctx.feedforward_data.filtered_yaw_v * gimbal_ctx.feedforward_data.yaw_kff, -0.2f, 0.2f);
    gimbal_ctx.feedforward_data.pitch_ff = std::clamp(gimbal_ctx.feedforward_data.filtered_pitch_v * gimbal_ctx.feedforward_data.pitch_kff, -0.2f, 0.2f);
}
}
