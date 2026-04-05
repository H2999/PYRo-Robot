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
    gimbal_ctx.motor.yaw_motor = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_5,can_hub_t::can1);
    gimbal_ctx.motor.pitch_motor = new dm_motor_drv_t(0x11,0x10,can_hub_t::can1);
    gimbal_ctx.motor.roll_motor = new dm_motor_drv_t(0x01,0x00,can_hub_t::can1);

    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.roll_motor)->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.roll_motor)->set_rotate_range(-30, 30);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.roll_motor)->set_torque_range(-10, 10);

    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.pitch_motor)->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.pitch_motor)->set_rotate_range(-15.333333, 15.333333);
    static_cast<dm_motor_drv_t *>(gimbal_ctx.motor.pitch_motor)->set_torque_range(-3, 3);

    gimbal_ctx.motor.yaw_motor->enable();
    gimbal_ctx.motor.pitch_motor->enable();
    gimbal_ctx.motor.roll_motor->enable();

    gimbal_ctx.pid.yaw_position_pid = new pid_t(10.4f,0.008f,0.003,1.0f,
                8.0f,60,25,4);
    gimbal_ctx.pid.pitch_position_pid = new pid_t(12.38f,0.0092f,0.0005f,0.8f,
                5.0f,60,35,4);
    gimbal_ctx.pid.roll_position_pid = new pid_t(5.28f,0.0006f,0.0008f,1.0f,
                5.0f,80,60,4);

    gimbal_ctx.pid.yaw_speed_pid = new pid_t(0.89f,0.0025f,0.0008f,1.0f,
                3.0f,60,20,4);
    gimbal_ctx.pid.pitch_speed_pid = new pid_t(1.265f,0.0065f,0.0005f,0.8f,
                3.0f,100,50,4);
    gimbal_ctx.pid.roll_speed_pid = new pid_t(0.75f,0.0005f,0.0004f,1.0f,
                8.0f,40,20,4);

    return PYRO_OK;
}

void uav_gimbal_t::_update_feedback()
{
    gimbal_ctx.auto_ctx.auto_enable = gimbal_ctx.cmd->auto_flag;

    gimbal_ctx.motor.yaw_motor->update_feedback();
    gimbal_ctx.motor.pitch_motor->update_feedback();
    gimbal_ctx.motor.roll_motor->update_feedback();

    float current_yaw_angle =
     gimbal_ctx.motor.yaw_motor->get_current_position() - YAW_OFFSET_RAD;
    normalize_angle(current_yaw_angle);
    gimbal_ctx.data.yaw_motor_angle = current_yaw_angle;

    gimbal_ctx.data.pitch_motor_angle = gimbal_ctx.motor.pitch_motor->get_current_position();
    gimbal_ctx.data.roll_motor_angle = gimbal_ctx.motor.roll_motor->get_current_position();

    //读取IMU获得当前角度
    gimbal_ins->get_rads_n(&gimbal_ctx.data._current_imu_yaw_angle,
                                          &gimbal_ctx.data._current_imu_pitch_angle,
                                          &gimbal_ctx.data._current_imu_roll_angle);

    gimbal_ins->get_gyro_b(&gimbal_ctx.data._current_imu_yaw_speed,
                                          &gimbal_ctx.data._current_imu_pitch_speed,
                                          &gimbal_ctx.data._current_imu_roll_speed);

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
    ctx->data._target_yaw_speed = ctx->pid.yaw_position_pid->calculate(
            ctx->data._target_yaw_angle,  ctx->data._current_imu_yaw_angle);

    ctx->data._target_pitch_speed = ctx->pid.pitch_position_pid->calculate(
             ctx->data._target_pitch_angle, ctx->data._current_imu_pitch_angle);

    ctx->data._target_roll_speed = ctx->pid.roll_position_pid->calculate(
            0,ctx->data._current_imu_roll_angle);

    ctx->data._output_yaw_torque = - ctx->pid.yaw_speed_pid->calculate(
            ctx->data._target_yaw_speed,ctx->data._current_imu_yaw_speed);

    ctx->data._output_pitch_torque = ctx->pid.pitch_speed_pid->calculate(
            ctx->data._target_pitch_speed,ctx->data._current_imu_pitch_speed);

    ctx->data._output_roll_torque = - ctx->pid.roll_speed_pid->calculate(
            ctx->data._target_roll_speed,ctx->data._current_imu_roll_speed);
}

void uav_gimbal_t::send_motor_command(const gimbal_ctx_t *ctx)
{
     ctx->motor.yaw_motor->send_torque(ctx->data._output_yaw_torque);
     // ctx->motor.yaw_motor->send_torque(0);

     // ctx->motor.pitch_motor->send_torque(ctx->data._output_pitch_torque);
     ctx->motor.pitch_motor->send_torque(0);

     // ctx->motor.roll_motor->send_torque(ctx->data._output_roll_torque);
    ctx->motor.roll_motor->send_torque(0);
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