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

    gimbal_ctx.yaw_td.r = 5800.0f;   // 根据响应速度调整 响应慢的话调大到 600-800 计算公式 比如目标角度变了0.1° 我想让云台在50ms内跟上这个变化
                                    // 0.1 = 1/2 * r * （0.05）² 但要克服阻力 惯性等因素 所以要给大一点
    gimbal_ctx.yaw_td.h = 0.002f;   // 滤波因子，一般设为 5~10 倍 dt 响应慢的话可以适当调小
    gimbal_ctx.yaw_td.dt = 0.001f;  //控制周期

    gimbal_ctx.pitch_td.r = 6000.0f;
    gimbal_ctx.pitch_td.h = 0.002f;
    gimbal_ctx.pitch_td.dt = 0.001f;

    // Order=2, omega_o - 观测器带宽, b - 控制增益项
    // z_limit 为扰动观测值的限制，防止异常抖动
    gimbal_ctx.yaw_leso = new leso_t<2>(100.0f, 2.0f, 5.0f);
    gimbal_ctx.pitch_leso = new leso_t<2>(80.0f, 10.0f, 8.0f);

    //遥控器pid
    gimbal_ctx.cfg.pid_ctx.yaw_position_pid = new pid_t(22.5f,0.001f,0.0005f,0.5f,
               6.0f,80,50,4);
    gimbal_ctx.cfg.pid_ctx.yaw_speed_pid = new pid_t(0.6f,0.05f,0.0f,1.2f,
                3.0f,80,50,4);

    gimbal_ctx.cfg.pid_ctx.pitch_position_pid = new pid_t(24.2f,0.0004f,0.006f,0.4f,
                9.0f,50,30,4);
    gimbal_ctx.cfg.pid_ctx.pitch_speed_pid = new pid_t(1.2f,0.007f,0.0f,1.2f,
                7.0f,80,20,4);

    //前哨站
    gimbal_ctx.cfg.pid_ctx.auto_yaw_position_pid_tower = new pid_t(19.8f,0.0f,0.0025f,0.8f,
    8.0f);
    gimbal_ctx.cfg.pid_ctx.auto_yaw_speed_pid_tower = new pid_t(1.0f,0.08f,0.0002f,1.0f,3.0f);

    gimbal_ctx.cfg.pid_ctx.auto_pitch_position_pid_tower = new pid_t(21.5f,0.0f,0.001f,0.8f,
                8.0f);
    gimbal_ctx.cfg.pid_ctx.auto_pitch_speed_pid_tower = new pid_t(0.6f,0.0f,0.00055f,0.8f,
                7.0f);

    //打车pid
    gimbal_ctx.cfg.pid_ctx.auto_yaw_position_pid = new pid_t(17.8f,0.0f,0.0025f,0.8f,
    10.0f);
    gimbal_ctx.cfg.pid_ctx.auto_yaw_speed_pid = new pid_t(0.8f,0.08f,0.0002f,1.0f,3.0f);

    gimbal_ctx.cfg.pid_ctx.auto_pitch_position_pid = new pid_t(19.5f,0.0f,0.001f,0.8f,
                8.0f);
    gimbal_ctx.cfg.pid_ctx.auto_pitch_speed_pid = new pid_t(0.58f,0.0f,0.00055f,0.8f,
                7.0f);

    //测试LESO
    gimbal_ctx.cfg.pid_ctx.yaw_position_pid_leso = new pid_t(5.0f,0.0f,0.0f,0.0f,
                10.0f);
    gimbal_ctx.cfg.pid_ctx.yaw_speed_pid_leso = new pid_t(1.0f,0.0f,0.0f,0.0f,
                3.0f);

    gimbal_ctx.cfg.pid_ctx.pitch_position_pid_leso = new pid_t(12.5f, 0.0f, 0.018f, 0.0f, 8.0f);
    gimbal_ctx.cfg.pid_ctx.pitch_speed_pid_leso = new pid_t(1.6f, 0.0f, 0.0005f, 1.2f, 7.0f);

    return PYRO_OK;
}

void uav_gimbal_t::_update_feedback()
{
    gimbal_ctx.auto_ctx.auto_enable = gimbal_ctx.cmd->auto_flag;

    gimbal_ctx.cfg.motor_ctx.yaw_motor->update_feedback();
    gimbal_ctx.cfg.motor_ctx.pitch_motor->update_feedback();

    gimbal_ctx.data.current_yaw_raw_rad = gimbal_ctx.cfg.motor_ctx.yaw_motor->get_current_position();
    gimbal_ctx.data.current_pitch_raw_rad = gimbal_ctx.cfg.motor_ctx.pitch_motor->get_current_position();

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

    //对current_imu_angle作归一化
    //因为电机安装位置的原因 yaw轴电机和imu角度减小的方向相反 Motor ↑  IMU ↓ 所以和是一个常数
    gimbal_ctx.data.yaw_real_min_limit_angle = gimbal_ctx.data._current_imu_yaw_angle + gimbal_ctx.data.yaw_motor_angle - yaw_motor_max_value;
    gimbal_ctx.data.yaw_real_max_limit_angle = gimbal_ctx.data._current_imu_yaw_angle + gimbal_ctx.data.yaw_motor_angle - yaw_motor_min_value;

    gimbal_ctx.yaw_leso->update(gimbal_ctx.data._current_imu_yaw_angle,
                                 gimbal_ctx.data._output_yaw_torque);

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
    // float observed_disturbance = ctx->yaw_leso->get_disturbance();

    float yaw_ff = ctx->cmd->yaw_delta_angle / control_dt * yaw_k_ff;

    ctx->data._target_yaw_speed = ctx->cfg.pid_ctx.yaw_position_pid->calculate
            (ctx->data._target_yaw_angle,ctx->data._current_imu_yaw_angle) + yaw_ff;

    ctx->data._output_yaw_torque = - ctx->cfg.pid_ctx.yaw_speed_pid->calculate(
            ctx->data._target_yaw_speed, ctx->data._current_imu_yaw_speed);

    // 4.LESO 核心：扰动补偿
    // 扰动项 z2 包含了摩擦、不平衡力矩等，将其反向叠加到输出中
    // 补偿系数通常为 1/b
    // float compensation = - (observed_disturbance / ctx->yaw_leso->get_b());
    // ctx->data._output_yaw_torque += compensation;

    float pitch_speed_ff = ctx->cmd->pitch_delta_angle / control_dt * pitch_k_ff;

    ctx->data._target_pitch_speed = ctx->cfg.pid_ctx.pitch_position_pid->calculate(
         ctx->data._target_pitch_angle, ctx->data._current_imu_pitch_angle) + pitch_speed_ff;

    // 物理建模补偿：水平时 cos(0)=1, 输出 -0.92f
    ctx->data.gravity_compensate = -0.92f * cosf(ctx->data._current_imu_pitch_angle);

    ctx->data._output_pitch_torque = ctx->cfg.pid_ctx.pitch_speed_pid->calculate(
        ctx->data._target_pitch_speed,ctx->data._current_imu_pitch_speed) + ctx->data.gravity_compensate;
}

void uav_gimbal_t::auto_aim_gimbal_control(gimbal_ctx_t *ctx)
{
    td_calculate(&ctx->yaw_td,ctx->data._target_yaw_angle);
    td_calculate(&ctx->pitch_td,ctx->data._target_pitch_angle);

    float yaw_ff = ctx->yaw_td.x2 * 0.05f;
    float pitch_ff = ctx->pitch_td.x2 * 0.55f;

    //加0.01后 在阶跃信号为0.2rad的情况下 基本跟上
    ctx->data._target_yaw_speed = ctx->cfg.pid_ctx.auto_yaw_position_pid_tower->calculate(
            ctx->yaw_td.x1,  ctx->data._current_imu_yaw_angle) + yaw_ff;

    ctx->data._output_yaw_torque = - ctx->cfg.pid_ctx.auto_yaw_speed_pid_tower->calculate(
            ctx->data._target_yaw_speed, ctx->data._current_imu_yaw_speed);

    ctx->data._target_pitch_speed = ctx->cfg.pid_ctx.auto_pitch_position_pid_tower->calculate(
             ctx->pitch_td.x1, ctx->data._current_imu_pitch_angle) + pitch_ff;

    //0.92是让pitch读取到的imu数据为0时的力矩 再乘上角度cos就能得到要补偿的重力大小
    ctx->data.gravity_compensate = -0.92f * cosf(ctx->data._current_imu_pitch_angle);
    ctx->data._output_pitch_torque = ctx->cfg.pid_ctx.auto_pitch_speed_pid_tower->calculate(
        ctx->data._target_pitch_speed,ctx->data._current_imu_pitch_speed) + ctx->data.gravity_compensate;
}

void uav_gimbal_t::auto_aim_gimbal_control_car(gimbal_ctx_t *ctx)
{
    td_calculate(&ctx->yaw_td,ctx->data._target_yaw_angle);
    td_calculate(&ctx->pitch_td,ctx->data._target_pitch_angle);

    float yaw_ff = ctx->yaw_td.x2 * 0.22f;
    float pitch_ff = ctx->pitch_td.x2 * 1.7f;

    //加0.01后 在阶跃信号为0.2rad的情况下 基本跟上
    ctx->data._target_yaw_speed = ctx->cfg.pid_ctx.auto_yaw_position_pid->calculate(
            ctx->yaw_td.x1,  ctx->data._current_imu_yaw_angle) + yaw_ff;

    ctx->data._output_yaw_torque = - ctx->cfg.pid_ctx.auto_yaw_speed_pid->calculate(
            ctx->data._target_yaw_speed, ctx->data._current_imu_yaw_speed);

    ctx->data._target_pitch_speed = ctx->cfg.pid_ctx.auto_pitch_position_pid->calculate(
             ctx->pitch_td.x1, ctx->data._current_imu_pitch_angle) + pitch_ff;

    //0.92是让pitch读取到的imu数据为0时的力矩 再乘上角度cos就能得到要补偿的重力大小
    ctx->data.gravity_compensate = -0.92f * cosf(ctx->data._current_imu_pitch_angle);
    ctx->data._output_pitch_torque = ctx->cfg.pid_ctx.auto_pitch_speed_pid->calculate(
        ctx->data._target_pitch_speed,ctx->data._current_imu_pitch_speed) + ctx->data.gravity_compensate;
}

void uav_gimbal_t::auto_aim_gimbal_control_leso(gimbal_ctx_t *ctx)
{
    float yaw_observed_disturbance = ctx->yaw_leso->get_disturbance();
    float yaw_ff = ctx->auto_ctx.kalman_yaw_v * 0.6f;

    ctx->data._target_yaw_speed = ctx->cfg.pid_ctx.yaw_position_pid_leso->calculate
            (ctx->data._target_yaw_angle,ctx->data._current_imu_yaw_angle) + yaw_ff;
    ctx->data._output_yaw_torque = - ctx->cfg.pid_ctx.yaw_speed_pid_leso->calculate(
            ctx->data._target_yaw_speed, ctx->data._current_imu_yaw_speed);
    //引入leso
    float compensation = - (yaw_observed_disturbance / ctx->yaw_leso->get_b());
    ctx->data._output_yaw_torque += compensation;

    float pitch_ff = ctx->auto_ctx.kalman_pitch_v * 0.5f;

    ctx->data._target_pitch_speed = ctx->cfg.pid_ctx.auto_pitch_position_pid->calculate(
             ctx->data._target_pitch_angle, ctx->data._current_imu_pitch_angle) + pitch_ff;

    //0.92是让pitch读取到的imu数据为0时的力矩 再乘上角度cos就能得到要补偿的重力大小
    ctx->data.gravity_compensate = -0.92f * cosf(ctx->data._current_imu_pitch_angle);

    ctx->data._output_pitch_torque = ctx->cfg.pid_ctx.auto_pitch_speed_pid->calculate(
        ctx->data._target_pitch_speed,ctx->data._current_imu_pitch_speed) + ctx->data.gravity_compensate;
}

void uav_gimbal_t::send_motor_command(const gimbal_ctx_t *ctx)
{
     ctx->cfg.motor_ctx.yaw_motor->send_torque(ctx->data._output_yaw_torque);
     ctx->cfg.motor_ctx.pitch_motor->send_torque(ctx->data._output_pitch_torque);
}

//用fmod 防止某些极端情况while卡死
void uav_gimbal_t::normalize_angle(float& angle)
{
    // 1. 先把角度挪到 [0, 2*PI] 范围内
    angle = fmodf(angle + PI, 2.0f * PI);
    // 2. 处理负数情况（C++ 的 fmod 结果符号与被除数一致）
    if (angle < 0)
    {
        angle += 2.0f * PI;
    }
    // 3. 再平移回 [-PI, PI]
    angle -= PI;
}

void uav_gimbal_t::td_calculate(TD_t *td, float target)
{
    const float x1_err = td->x1 - target;
    float d = td->r * td->h * td->h;
    const float a0 = td->h * td->x2;
    const float y = x1_err + a0;

    auto sgn = [](const float x) { return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f); };

    float a1 = sqrtf(d * (d + 8.0f * fabsf(y)));
    float a2 = a0 + sgn(y) * (a1 - d) * 0.5f;

    float sy = (sgn(y + d) - sgn(y - d)) * 0.5f;
    float a = (a0 + y - a2) * sy + a2;

    float sa = (sgn(a + d) - sgn(a - d)) * 0.5f;
    td->fh = -td->r * ((a / d - sgn(a)) * sa + sgn(a));

    td->x1 += td->dt * td->x2;
    td->x2 += td->dt * td->fh;
}

uav_gimbal_t::gimbal_ctx_t* uav_gimbal_t::get_data()
{
    return &gimbal_ctx;
}

}