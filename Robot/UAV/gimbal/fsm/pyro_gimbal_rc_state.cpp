#include "pyro_uav_gimbal.h"

using namespace pyro;

void uav_gimbal_t::fsm_active_t::state_rc_t::enter(uav_gimbal_t *owner)
{

}

void uav_gimbal_t::fsm_active_t::state_rc_t::execute(uav_gimbal_t *owner)
{
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
    owner->gimbal_ctx.data._target_pitch_angle += owner->gimbal_ctx.cmd->pitch_delta_angle;

    if (owner->gimbal_ctx.data._target_pitch_angle > pitch_max_value)
    {
        owner->gimbal_ctx.data._target_pitch_angle = pitch_max_value;
    }
    else if (owner->gimbal_ctx.data._target_pitch_angle < pitch_min_value)
    {
        owner->gimbal_ctx.data._target_pitch_angle = pitch_min_value;
    }
    const float pitch_error = owner->gimbal_ctx.data._target_pitch_angle - owner->gimbal_ctx.data._current_imu_pitch_angle;
    if (pitch_error > PI)
    {
        owner->gimbal_ctx.data._target_pitch_angle -= 2.0f * PI;
    }
    else if (pitch_error < -PI)
    {
        owner->gimbal_ctx.data._target_pitch_angle += 2.0f * PI;
    }

    // owner->gimbal_ctx.data._target_roll_angle += owner->gimbal_ctx.cmd->roll_delta_angle;
    // normalize_angle(owner->gimbal_ctx.data._target_roll_angle);
    // static float yaw_to_roll = 0.0f;
    // if (owner->gimbal_ctx.data.pitch_motor_angle < 2.78f && owner->gimbal_ctx.data.pitch_motor_angle > 2.3f)
    // {
    //     owner->gimbal_ctx.data.final_roll_angle = owner->gimbal_ctx.data._target_roll_angle;
    // }
    // else
    // {
    //     if (owner->gimbal_ctx.data.pitch_motor_angle < 3.04f && owner->gimbal_ctx.data.pitch_motor_angle > 2.78f)
    //     {
    //         yaw_to_roll = 0.04;
    //     }
    //     else
    //     {
    //         yaw_to_roll = 0.065f;
    //     }
    //     if (owner->gimbal_ctx.data._current_imu_yaw_speed > 0.4f)
    //     {
    //         owner->gimbal_ctx.data.final_roll_angle = owner->gimbal_ctx.data._target_roll_angle
    //             - owner->gimbal_ctx.data._current_imu_yaw_speed * owner->gimbal_ctx.data._current_imu_yaw_speed * yaw_to_roll;
    //     }
    //     else if (owner->gimbal_ctx.data._current_imu_yaw_speed < -0.4f)
    //     {
    //         owner->gimbal_ctx.data.final_roll_angle = owner->gimbal_ctx.data._target_roll_angle
    //             + owner->gimbal_ctx.data._current_imu_yaw_speed * owner->gimbal_ctx.data._current_imu_yaw_speed * yaw_to_roll;
    //     }
    //     else
    //     {
    //         owner->gimbal_ctx.data.final_roll_angle = owner->gimbal_ctx.data._target_roll_angle;
    //     }
    // }
    //
    // if (owner->gimbal_ctx.data._target_roll_angle > roll_max_value)
    // {
    //     owner->gimbal_ctx.data._target_roll_angle = roll_max_value;
    // }
    // else if (owner->gimbal_ctx.data._target_roll_angle < roll_min_value)
    // {
    //     owner->gimbal_ctx.data._target_roll_angle = roll_min_value;
    // }

    gimbal_control(&owner->gimbal_ctx);
    send_motor_command(&owner->gimbal_ctx);
}

void uav_gimbal_t::fsm_active_t::state_rc_t::exit(uav_gimbal_t *owner)
{

}


