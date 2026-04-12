#include "pyro_uav_gimbal.h"
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
    gimbal_ctx.cfg.motor_ctx.yaw_motor = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_5,can_hub_t::can1);
    gimbal_ctx.cfg.motor_ctx.pitch_motor = new dm_motor_drv_t(0x01,0x00,can_hub_t::can1);

    static_cast<dm_motor_drv_t *>(gimbal_ctx.cfg.motor_ctx.pitch_motor)->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.cfg.motor_ctx.pitch_motor)->set_rotate_range(-30, 30);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.cfg.motor_ctx.pitch_motor)->set_torque_range(-10, 10);

    gimbal_ctx.cfg.pid_ctx.yaw_position_pid = new pid_t(16.4f,0.008f,0.0005,1.0f,
                8.0f,60,25,4);
    gimbal_ctx.cfg.pid_ctx.pitch_position_pid = new pid_t(10.28f,0.0065f,0.00048f,0.8f,
                5.0f,60,35,4);

    gimbal_ctx.cfg.pid_ctx.yaw_speed_pid = new pid_t(2.08f,0.0025f,0.0008f,1.0f,
                3.0f,60,20,4);
    gimbal_ctx.cfg.pid_ctx.pitch_speed_pid = new pid_t(1.02f,0.0058f,0.00053f,0.8f,
                10.0f,50,30,4);

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

    gimbal_ctx.data.pitch_motor_angle = gimbal_ctx.cfg.motor_ctx.pitch_motor->get_current_position();

    //读取IMU获得当前角度
    gimbal_ins->get_rads_n(&gimbal_ctx.data._current_imu_yaw_angle,
                                          &gimbal_ctx.data._current_imu_pitch_angle,
                                          &gimbal_ctx.data._current_imu_roll_angle);

    gimbal_ins->get_gyro_b(&gimbal_ctx.data._current_imu_yaw_speed,
                                          &gimbal_ctx.data._current_imu_pitch_speed,
                                          &gimbal_ctx.data._current_imu_roll_speed);

    //对current_imu_angle作归一化
    gimbal_ctx.data.yaw_real_min_limit_angle = gimbal_ctx.data._current_imu_yaw_angle + gimbal_ctx.data.yaw_motor_angle - yaw_motor_max_value;
    gimbal_ctx.data.yaw_real_max_limit_angle = gimbal_ctx.data._current_imu_yaw_angle + gimbal_ctx.data.yaw_motor_angle - yaw_motor_min_value;
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

void uav_gimbal_t::gimbal_control(gimbal_ctx_t *ctx)
{
    ctx->data._target_yaw_speed = ctx->cfg.pid_ctx.yaw_position_pid->calculate(
            ctx->data._target_yaw_angle,  ctx->data._current_imu_yaw_angle);

    ctx->data._target_pitch_speed = ctx->cfg.pid_ctx.pitch_position_pid->calculate(
             ctx->data._target_pitch_angle, ctx->data._current_imu_pitch_angle);

    ctx->data._output_yaw_torque = - ctx->cfg.pid_ctx.yaw_speed_pid->calculate(
            ctx->data._target_yaw_speed,ctx->data._current_imu_yaw_speed);

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
}

void uav_gimbal_t::send_motor_command(const gimbal_ctx_t *ctx)
{
     ctx->cfg.motor_ctx.yaw_motor->send_torque(ctx->data._output_yaw_torque);

     ctx->cfg.motor_ctx.pitch_motor->send_torque(ctx->data._output_pitch_torque);
    // ctx->motor.pitch_motor->send_torque(0);
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

float uav_gimbal_t::get_current_yaw_angle() const
{
    return gimbal_ctx.data._current_imu_yaw_angle;
}

float uav_gimbal_t::get_current_pitch_angle() const
{
    return gimbal_ctx.data._current_imu_pitch_angle;
}

float uav_gimbal_t::get_current_roll_angle() const
{
    return gimbal_ctx.data._current_imu_roll_angle;
}

}