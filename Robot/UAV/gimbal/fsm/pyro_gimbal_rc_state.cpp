#include "pyro_uav_gimbal.h"

using namespace pyro;

void uav_gimbal_t::fsm_active_t::state_rc_t::enter(uav_gimbal_t *owner)
{
}

void uav_gimbal_t::fsm_active_t::state_rc_t::execute(uav_gimbal_t *owner)
{
    static float current_pitch_kp = 0.0f;
    float target_kp = 18.0f;
    float ramp_speed = 0.004f;

    if (current_pitch_kp < target_kp)
    {
        current_pitch_kp += ramp_speed;
    }
    else
    {
        current_pitch_kp = target_kp;
    }

    // 更新 PID 参数
    owner->gimbal_ctx.cfg.pid_ctx.pitch_position_pid->set_gains(
        current_pitch_kp, 0.01f, 0.004f);

    owner->gimbal_ctx.data._target_yaw_angle += owner->gimbal_ctx.cmd->yaw_delta_angle;

    if (owner->gimbal_ctx.data._target_yaw_angle > owner->gimbal_ctx.data.yaw_real_max_limit_angle)
    {
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data.yaw_real_max_limit_angle;
    }
    if (owner->gimbal_ctx.data._target_yaw_angle < owner->gimbal_ctx.data.yaw_real_min_limit_angle)
    {
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data.yaw_real_min_limit_angle;
    }
    const float yaw_error = owner->gimbal_ctx.data._target_yaw_angle - owner->gimbal_ctx.data._current_imu_yaw_angle;
    if (yaw_error > PI)
    {
        owner->gimbal_ctx.data._target_yaw_angle -= 2.0f * PI;
    }
    else if (yaw_error < -PI)
    {
        owner->gimbal_ctx.data._target_yaw_angle += 2.0f * PI;
    }


    //pitch目标值 归一化到-pi到pi之间
    if (owner->gimbal_ctx.data.pitch_motor_angle > 0.02f)
    {
        owner->gimbal_ctx.cmd->pitch_delta_angle = 0.0f;
    }
    owner->gimbal_ctx.data._target_pitch_angle += owner->gimbal_ctx.cmd->pitch_delta_angle;

    if (owner->gimbal_ctx.data._target_pitch_angle > pitch_max_value)
    {
        owner->gimbal_ctx.data._target_pitch_angle = pitch_max_value;
    }
    else if (owner->gimbal_ctx.data._target_pitch_angle < pitch_min_value)
    {
        owner->gimbal_ctx.data._target_pitch_angle = pitch_min_value;
    }
    // const float pitch_error = owner->gimbal_ctx.data._target_pitch_angle - owner->gimbal_ctx.data._current_imu_pitch_angle;
    // if (pitch_error > PI)
    // {
    //     owner->gimbal_ctx.data._target_pitch_angle -= 2.0f * PI;
    // }
    // else if (pitch_error < -PI)
    // {
    //     owner->gimbal_ctx.data._target_pitch_angle += 2.0f * PI;
    // }

    rc_gimbal_control(&owner->gimbal_ctx);
    send_motor_command(&owner->gimbal_ctx);
}

void uav_gimbal_t::fsm_active_t::state_rc_t::exit(uav_gimbal_t *owner)
{
}
