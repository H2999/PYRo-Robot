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

    //TD参数
    gimbal_ctx.yaw_td.r = 700.0f;   // 根据响应速度调整 响应慢的话调大到 600-800 计算公式 比如目标角度变了0.1° 我想让云台在50ms内跟上这个变化
                                    // 0.1 = 1/2 * r * （0.05）² 但要克服阻力 惯性等因素 所以要给大一点
    gimbal_ctx.yaw_td.h = 0.005f;   // 滤波因子，一般设为 5~10 倍 dt 响应慢的话可以适当调小
    gimbal_ctx.yaw_td.dt = 0.001f;  //控制周期

    gimbal_ctx.pitch_td.r = 1200.0f;
    gimbal_ctx.pitch_td.h = 0.004f;
    gimbal_ctx.pitch_td.dt = 0.001f;

    //ESO参数
    gimbal_ctx.yaw_eso.r = 90.0f;    // 观测器带宽 (初始建议 100-200)
    gimbal_ctx.yaw_eso.b0 = 100.0f;   // 控制增益 (需要根据电机力矩常数和惯量整定)
    gimbal_ctx.yaw_eso.dt = 0.001f;  // 1ms 周期
    gimbal_ctx.yaw_eso.z1 = gimbal_ctx.yaw_eso.z2 = gimbal_ctx.yaw_eso.z3 = 0.0f;

    // Pitch ESO 初始化
    gimbal_ctx.pitch_eso.r = 110.0f;  // Pitch 通常需要更快的观测速度
    gimbal_ctx.pitch_eso.b0 = 180.0f;
    gimbal_ctx.pitch_eso.dt = 0.001f;
    gimbal_ctx.pitch_eso.z1 = gimbal_ctx.pitch_eso.z2 = gimbal_ctx.pitch_eso.z3 = 0.0f;

    gimbal_ctx.cfg.pid_ctx.yaw_position_pid = new pid_t(24.5f,0.0f,0.0f,0.0f,
               10.0f,50,20,4);
    gimbal_ctx.cfg.pid_ctx.yaw_speed_pid = new pid_t(3.25f,0.08f,0.0003f,1.5f,
                3.0f,50,20,4);

    gimbal_ctx.cfg.pid_ctx.pitch_position_pid = new pid_t(25.2f,0.0004f,0.006f,0.4f,
                9.0f,50,30,4);
    gimbal_ctx.cfg.pid_ctx.pitch_speed_pid = new pid_t(1.18f,0.068f,0.006f,1.8f,
                7.0f,30,15,4);

    //自瞄pid
    gimbal_ctx.cfg.pid_ctx.auto_yaw_position_pid = new pid_t(25.5f,0.005f,0.0f,0.8f,
               8.0f,50,20,4);
    gimbal_ctx.cfg.pid_ctx.auto_yaw_speed_pid = new pid_t(3.4f,0.08f,0.0003f,1.0f,
                3.0f,50,20,4);

    // gimbal_ctx.cfg.pid_ctx.auto_pitch_position_pid = new pid_t(25.8f,0.002f,0.006f,0.5f,
    //             9.0f,50,30,2);
    gimbal_ctx.cfg.pid_ctx.auto_pitch_position_pid = new pid_t(25.0f,0.008f,0.005f,0.8f,
                8.0f,50,30,4);
    gimbal_ctx.cfg.pid_ctx.auto_pitch_speed_pid = new pid_t(1.3f,0.075f,0.002f,1.0f,
                7.0f,45,20,4);

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

    //打了几个点 ai拟合的
    // 获取角度
    float x = ctx->data.pitch_motor_angle;
    // 2. 双项正弦高精度拟合 (傅里叶级数)
    float term1 = 0.455f * sinf(2.12f * x - 0.58f);
    float term2 = 0.038f * sinf(10.5f * x + 1.85f);

    ctx->data.gravity_compensate = -0.762f + term1 + term2;

    ctx->data._output_pitch_torque = ctx->cfg.pid_ctx.pitch_speed_pid->calculate(
        ctx->data._target_pitch_speed,ctx->data._current_imu_pitch_speed) + ctx->data.gravity_compensate;
    // ctx->data._output_pitch_torque = ctx->data.gravity_compensate;
}

void uav_gimbal_t::eso_update(eso_t *eso, float y, float u)
{
    // y 是 IMU 实际测量值，u 是上一帧的控制量
    float error = y - eso->z1;

    // 基于带宽 w_o (eso->r) 的增益分配
    float b1 = 3.0f * eso->r;
    float b2 = 3.0f * eso->r * eso->r;
    float b3 = eso->r * eso->r * eso->r;

    // 状态更新迭代 (欧拉法)
    eso->z1 += (eso->z2 + b1 * error) * eso->dt;
    eso->z2 += (eso->z3 + b2 * error + eso->b0 * u) * eso->dt;
    eso->z3 += (b3 * error) * eso->dt;
}

//结合自瞄发的预测角速度进行的前馈预测
void uav_gimbal_t::auto_aim_gimbal_control(gimbal_ctx_t *ctx)
{
    // --- 第一步：目标平滑 (TD) ---
    // 根据误差动态调整收敛速度 r，防止大切换时的冲击
    float yaw_error = ctx->data._target_yaw_angle - ctx->data._current_imu_yaw_angle;
    normalize_angle(yaw_error);

    float yaw_r_ratio = std::clamp((fabsf(yaw_error) - 0.02f) / (0.35f - 0.02f), 0.0f, 1.0f);
    ctx->yaw_td.r = 800.0f + (1500.0f * yaw_r_ratio);
    td_calculate(&ctx->yaw_td, ctx->data._target_yaw_angle);

    // float pitch_error = ctx->data._target_pitch_angle - ctx->data._current_imu_pitch_angle;
    // ctx->pitch_td.r = (fabsf(pitch_error) > 0.15f) ? 1500.0f : 900.0f;
    ctx->pitch_td.r = 1200.0f;
    td_calculate(&ctx->pitch_td, ctx->data._target_pitch_angle);

    // --- 第二步：状态观测 (ESO) ---
    // 输入：当前 IMU 角度，上一帧的扭矩输出
    // 这步会更新 z1(估算角), z2(估算速), z3(总扰动)
    eso_update(&ctx->yaw_eso, ctx->data._current_imu_yaw_angle, ctx->data._output_yaw_torque);
    eso_update(&ctx->pitch_eso, ctx->data._current_imu_pitch_angle, ctx->data._output_pitch_torque);

    // --- 第三步：前馈项计算 (Kalman + TD微分) ---
    float yaw_error_factor = std::clamp(fabsf(yaw_error) * 2.0f, 0.0f, 1.0f);
    float yaw_combined_ff = (ctx->auto_ctx.kalman_yaw_v * 0.7f) + (ctx->yaw_td.x2 * 0.3f * yaw_error_factor);
    float pitch_speed_ff = (ctx->auto_ctx.kalman_pitch_v * 0.6f);

    // --- 第四步：控制量计算与 ADRC 补偿 ---

    // 1. Yaw 轴
    ctx->data._target_yaw_speed = ctx->cfg.pid_ctx.auto_yaw_position_pid->calculate(
            ctx->yaw_td.x1, ctx->data._current_imu_yaw_angle) + yaw_combined_ff;

    // 基础 PID 扭矩 u0 (这里不再需要手动加 YAW_STATIC_FRICTION)
    float yaw_u0 = -ctx->cfg.pid_ctx.auto_yaw_speed_pid->calculate(
            ctx->data._target_yaw_speed, ctx->data._current_imu_yaw_speed);

    // ADRC 补偿项: u = u0 - z3/b0
    // 注意：这里的 z3/b0 已经包含了摩擦力、线缆拉力等所有扰动
    ctx->data._output_yaw_torque = yaw_u0 - (ctx->yaw_eso.z3 / ctx->yaw_eso.b0);
    ctx->data._output_yaw_torque = std::clamp(ctx->data._output_yaw_torque, -3.0f, 3.0f);

    // 2. Pitch 轴
    ctx->data._target_pitch_speed = ctx->cfg.pid_ctx.auto_pitch_position_pid->calculate(
             ctx->pitch_td.x1, ctx->data._current_imu_pitch_angle) + pitch_speed_ff;

    float pitch_u0 = ctx->cfg.pid_ctx.auto_pitch_speed_pid->calculate(
        ctx->data._target_pitch_speed, ctx->data._current_imu_pitch_speed);

    // 计算重力补偿 (这是你原本拟合的模型)
    float x = ctx->data.pitch_motor_angle;
    ctx->data.gravity_compensate = -0.762f + 0.455f * sinf(2.12f * x - 0.58f) + 0.038f * sinf(10.5f * x + 1.85f);
    if (fabsf(ctx->data.gravity_compensate) > 1.35f) ctx->data.gravity_compensate = (ctx->data.gravity_compensate > 0 ? 1.35f : -1.35f);

    // Pitch 最终输出 = PID + 重力模型前馈 - ESO扰动补偿
    // ESO 会负责修正重力模型拟合不准的部分以及 0.5kg 负载带来的额外惯性
    ctx->data._output_pitch_torque = pitch_u0 + ctx->data.gravity_compensate - (ctx->pitch_eso.z3 / ctx->pitch_eso.b0);
    ctx->data._output_pitch_torque = std::clamp(ctx->data._output_pitch_torque, -3.0f, 3.0f);
}

void uav_gimbal_t::send_motor_command(const gimbal_ctx_t *ctx)
{
     // ctx->cfg.motor_ctx.yaw_motor->send_torque(ctx->data._output_yaw_torque);
     ctx->cfg.motor_ctx.yaw_motor->send_torque(0.0f);
     ctx->cfg.motor_ctx.pitch_motor->send_torque(ctx->data._output_pitch_torque);
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
