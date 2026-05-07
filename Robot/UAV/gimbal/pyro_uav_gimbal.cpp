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

    gimbal_ctx.yaw_td.r = 500.0f;   // 根据响应速度调整 响应慢的话调大到 600-800 计算公式 比如目标角度变了0.1° 我想让云台在50ms内跟上这个变化
                                    // 0.1 = 1/2 * r * （0.05）² 但要克服阻力 惯性等因素 所以要给大一点
    gimbal_ctx.yaw_td.h = 0.005f;   // 滤波因子，一般设为 5~10 倍 dt 响应慢的话可以适当调小
    gimbal_ctx.yaw_td.dt = 0.001f;  //控制周期

    gimbal_ctx.pitch_td.r = 700.0f;
    gimbal_ctx.pitch_td.h = 0.004f;
    gimbal_ctx.pitch_td.dt = 0.001f;

    gimbal_ctx.cfg.pid_ctx.yaw_position_pid = new pid_t(24.5f,0.0f,0.0f,0.0f,
               10.0f,50,20,4);
    gimbal_ctx.cfg.pid_ctx.yaw_speed_pid = new pid_t(3.25f,0.08f,0.0003f,1.5f,
                3.0f,50,20,4);

    gimbal_ctx.cfg.pid_ctx.pitch_position_pid = new pid_t(22.1f,0.0004f,0.006f,0.4f,
                8.0f,50,30,4);
    gimbal_ctx.cfg.pid_ctx.pitch_speed_pid = new pid_t(1.1f,0.065f,0.008f,1.2f,
                7.0f,30,15,4);

    //自瞄pid
    gimbal_ctx.cfg.pid_ctx.auto_yaw_position_pid = new pid_t(24.1f,0.0f,0.0f,0.0f,
               8.0f,50,20,2);
    gimbal_ctx.cfg.pid_ctx.auto_yaw_speed_pid = new pid_t(2.9f,0.08f,0.0003f,1.0f,
                3.0f,50,20,2);

    gimbal_ctx.cfg.pid_ctx.auto_pitch_position_pid = new pid_t(21.5f,0.002f,0.006f,0.5f,
                8.0f,50,30,2);
    gimbal_ctx.cfg.pid_ctx.auto_pitch_speed_pid = new pid_t(1.1f,0.065f,0.0007f,1.0f,
                7.0f,30,15,2);

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
    float yaw_speed_ff = ctx->cmd->yaw_delta_angle / control_dt * yaw_k_ff;

    ctx->data._target_yaw_speed = ctx->cfg.pid_ctx.yaw_position_pid->calculate(
            ctx->data._target_yaw_angle,  ctx->data._current_imu_yaw_angle) + yaw_speed_ff;

    ctx->data._output_yaw_torque = - ctx->cfg.pid_ctx.yaw_speed_pid->calculate(
            ctx->data._target_yaw_speed, ctx->data._current_imu_yaw_speed);

    //yaw 阻力补偿
    constexpr float YAW_DEADBAND_RADPS = 0.08f;

    if (ctx->data._target_yaw_speed > YAW_DEADBAND_RADPS)
    {
        ctx->data._output_yaw_torque += -0.08f;
    }
    else if (ctx->data._target_yaw_speed < -YAW_DEADBAND_RADPS)
    {
        ctx->data._output_yaw_torque += 0.08f;
    }

    ctx->data._output_yaw_torque = std::clamp(ctx->data._output_yaw_torque, -3.0f, 3.0f);

    float pitch_speed_ff = ctx->cmd->pitch_delta_angle / control_dt * pitch_k_ff;

    ctx->data._target_pitch_speed = ctx->cfg.pid_ctx.pitch_position_pid->calculate(
         ctx->data._target_pitch_angle, ctx->data._current_imu_pitch_angle) + pitch_speed_ff;

    ctx->data.gravity_compensate = 0.27f * ctx->data.pitch_motor_angle * ctx->data.pitch_motor_angle
                            + 0.53f * ctx->data.pitch_motor_angle - 1.09f;

    ctx->data._output_pitch_torque = ctx->cfg.pid_ctx.pitch_speed_pid->calculate(
        ctx->data._target_pitch_speed,ctx->data._current_imu_pitch_speed) + ctx->data.gravity_compensate;
    // ctx->data._output_pitch_torque = ctx->data.gravity_compensate;
}

//无前馈 仅靠pid版
// void uav_gimbal_t::auto_aim_gimbal_control(gimbal_ctx_t *ctx)
// {
//     float yaw_speed_ff = ctx->feedforward_data.yaw_ff;
//     ctx->data._target_yaw_speed = ctx->cfg.pid_ctx.auto_yaw_position_pid->calculate(
//             ctx->data._target_yaw_angle,  ctx->data._current_imu_yaw_angle) + yaw_speed_ff;
//
//     constexpr float YAW_DEADBAND_RADPS = 0.04f;
//
//     if (ctx->data._target_yaw_speed > YAW_DEADBAND_RADPS)
//     {
//         ctx->data._output_yaw_torque += -0.12f;
//     }
//     else if (ctx->data._target_yaw_speed < -YAW_DEADBAND_RADPS)
//     {
//         ctx->data._output_yaw_torque += 0.12f;
//     }
//
//     ctx->data._output_yaw_torque = - ctx->cfg.pid_ctx.auto_yaw_speed_pid->calculate(
//             ctx->data._target_yaw_speed, ctx->data._current_imu_yaw_speed);
//
//     float pitch_speed_ff = ctx->feedforward_data.pitch_ff;
//
//     ctx->data._target_pitch_speed = ctx->cfg.pid_ctx.auto_pitch_position_pid->calculate(
//              ctx->data._target_pitch_angle, ctx->data._current_imu_pitch_angle);// + pitch_speed_ff;
//
//     ctx->data.gravity_compensate = (1.7671f * ctx->data.pitch_motor_angle - 0.9575f) * ctx->data.pitch_motor_angle - 1.1147f;
//     if (abs(ctx->data.gravity_compensate) > 1.35f) ctx->data.gravity_compensate = 1.35f;
//
//     ctx->data._output_pitch_torque = ctx->cfg.pid_ctx.auto_pitch_speed_pid->calculate(
//         ctx->data._target_pitch_speed,ctx->data._current_imu_pitch_speed);// + ctx->data.gravity_compensate;
// }

//结合TD跟踪器 把阶跃变斜坡 再加上TD算出来的角速度作前馈
// void uav_gimbal_t::auto_aim_gimbal_control(gimbal_ctx_t *ctx)
// {
//     // 防止切入瞬间抖动 如果刚开启自瞄 强制 TD 位置等于当前 IMU 位置
//     static bool last_auto_flag = false;
//     if (ctx->auto_ctx.auto_enable && !last_auto_flag)
//     {
//         ctx->yaw_td.x1 = ctx->data._current_imu_yaw_angle;
//         ctx->yaw_td.x2 = 0.0f;
//
//         ctx->pitch_td.x1 = ctx->data._current_imu_pitch_angle;
//         ctx->pitch_td.x2 = 0.0f;
//     }
//     last_auto_flag = ctx->auto_ctx.auto_enable;
//
//     //  TD 计算平滑目标
//     td_calculate(&ctx->yaw_td, ctx->data._target_yaw_angle);
//     td_calculate(&ctx->pitch_td, ctx->data._target_pitch_angle);
//
//     // 2. Yaw 轴计算
//     // PID 时使用的是 TD 输出的平滑位置 x1
//     float yaw_speed_ff = ctx->yaw_td.x2 * 0.7f;
//
//     ctx->data._target_yaw_speed = ctx->cfg.pid_ctx.auto_yaw_position_pid->calculate(
//             ctx->yaw_td.x1,  ctx->data._current_imu_yaw_angle) + yaw_speed_ff;
//
//     ctx->data._output_yaw_torque = - ctx->cfg.pid_ctx.auto_yaw_speed_pid->calculate(
//             ctx->data._target_yaw_speed, ctx->data._current_imu_yaw_speed);
//
//     constexpr float YAW_DEADBAND_RADPS = 0.08f;
//
//     if (ctx->data._target_yaw_speed > YAW_DEADBAND_RADPS)
//     {
//         ctx->data._output_yaw_torque += -0.1f;
//     }
//     else if (ctx->data._target_yaw_speed < -YAW_DEADBAND_RADPS)
//     {
//         ctx->data._output_yaw_torque += 0.1f;
//     }
//
//     // 3. Pitch 轴计算
//     float pitch_speed_ff = ctx->pitch_td.x2 * 0.35f;
//
//     ctx->data._target_pitch_speed = ctx->cfg.pid_ctx.auto_pitch_position_pid->calculate(
//              ctx->pitch_td.x1, ctx->data._current_imu_pitch_angle) + pitch_speed_ff;
//
//     // 重力补偿
//     ctx->data.gravity_compensate = (1.7671f * ctx->data.pitch_motor_angle - 0.9575f) * ctx->data.pitch_motor_angle - 1.1147f;
//     if (abs(ctx->data.gravity_compensate) > 1.35f) ctx->data.gravity_compensate = 1.35f;
//
//     ctx->data._output_pitch_torque = ctx->cfg.pid_ctx.auto_pitch_speed_pid->calculate(
//         ctx->data._target_pitch_speed, ctx->data._current_imu_pitch_speed) + ctx->data.gravity_compensate;
// }

//结合自瞄发的预测角速度进行的前馈预测
void uav_gimbal_t::auto_aim_gimbal_control(gimbal_ctx_t *ctx)
{
    // 1. 状态初始化与切换逻辑
    static bool last_auto_flag = false;
    if (ctx->auto_ctx.auto_enable && !last_auto_flag)
    {
        ctx->yaw_td.x1 = ctx->data._current_imu_yaw_angle;
        ctx->yaw_td.x2 = 0.0f;
        ctx->pitch_td.x1 = ctx->data._current_imu_pitch_angle;
        ctx->pitch_td.x2 = 0.0f;
    }
    last_auto_flag = ctx->auto_ctx.auto_enable;

    // 2. 计算当前误差阈值，用于判断是否处于“剧烈运动”模式
    float yaw_error = ctx->data._target_yaw_angle - ctx->data._current_imu_yaw_angle;
    normalize_angle(yaw_error);
    float pitch_error = ctx->data._target_pitch_angle - ctx->data._current_imu_pitch_angle;

    // --- 动态 TD 策略 (解决跟不上小陀螺的关键) ---
    // 如果误差很大，增大 r 以求快速响应；误差小时减小 r 以求平滑
    if (fabsf(yaw_error) > 0.2f)
    {
        ctx->yaw_td.r = 1500.0f;
    }
    else
    {
        ctx->yaw_td.r = 500.0f;
    }

    if (fabsf(pitch_error) > 0.15f)
    {
        ctx->pitch_td.r = 1200.0f;
    }
    else
    {
        ctx->pitch_td.r = 700.0f;
    }

    td_calculate(&ctx->yaw_td, ctx->data._target_yaw_angle);
    td_calculate(&ctx->pitch_td, ctx->data._target_pitch_angle);

    float kalman_yaw_v = ctx->auto_ctx.kalman_yaw_v;

    float error_factor = std::clamp(fabsf(yaw_error) * 2.0f, 0.0f, 1.0f);

    // 2. 最终前馈 = (大模型给的保底预测速度) + (本地计算的追赶速度 × 急躁系数)
    float combined_ff = (kalman_yaw_v * 0.6f) + (ctx->yaw_td.x2 * 0.4f * error_factor);
    // float yaw_speed_ff = (kalman_yaw_v * 0.6f);// + (ctx->yaw_td.x2 * 0.2f);

    float kalman_pitch_v = ctx->auto_ctx.kalman_pitch_v;
    float pitch_speed_ff = (kalman_pitch_v * 0.6f);// + (ctx->pitch_td.x2 * 0.2f);

    ctx->data._target_yaw_speed = ctx->cfg.pid_ctx.auto_yaw_position_pid->calculate(
            ctx->yaw_td.x1,  ctx->data._current_imu_yaw_angle) + combined_ff;

    ctx->data._output_yaw_torque = - ctx->cfg.pid_ctx.auto_yaw_speed_pid->calculate(
            ctx->data._target_yaw_speed, ctx->data._current_imu_yaw_speed);

    //电机安装位置的原因 输出正扭矩电机朝角度减小的方向转 所以这里逻辑是反的
    constexpr float YAW_STATIC_FRICTION = 0.1f;
    if (fabsf(ctx->data._target_yaw_speed) > 0.02f)
    {
        ctx->data._output_yaw_torque += (ctx->data._target_yaw_speed > 0) ? -YAW_STATIC_FRICTION : YAW_STATIC_FRICTION;
    }
    ctx->data._output_yaw_torque = std::clamp(ctx->data._output_yaw_torque, -3.0f, 3.0f);

    ctx->data._target_pitch_speed = ctx->cfg.pid_ctx.auto_pitch_position_pid->calculate(
             ctx->pitch_td.x1, ctx->data._current_imu_pitch_angle) + pitch_speed_ff;

    // 重力补偿
    ctx->data.gravity_compensate = (1.7671f * ctx->data.pitch_motor_angle - 0.9575f) * ctx->data.pitch_motor_angle - 1.1147f;
    ctx->data.gravity_compensate = std::clamp(ctx->data.gravity_compensate, -1.35f, 1.35f);

    ctx->data._output_pitch_torque = ctx->cfg.pid_ctx.auto_pitch_speed_pid->calculate(
        ctx->data._target_pitch_speed, ctx->data._current_imu_pitch_speed) + ctx->data.gravity_compensate;
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
    float fh = -td->r * ((a / d - sgn(a)) * sa + sgn(a));

    td->x1 += td->dt * td->x2;
    td->x2 += td->dt * fh;
}

uav_gimbal_t::gimbal_ctx_t* uav_gimbal_t::get_data()
{
    return &gimbal_ctx;
}

}
