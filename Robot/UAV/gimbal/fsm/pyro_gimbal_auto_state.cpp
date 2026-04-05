#include "pyro_uav_gimbal.h"

using namespace pyro;

void uav_gimbal_t::fsm_active_t::state_auto_t::enter(uav_gimbal_t *owner)
{

}

void uav_gimbal_t::fsm_active_t::state_auto_t::execute(uav_gimbal_t *owner)
{
    if (owner->gimbal_ctx.cmd->yaw_target_angle > 10.0f)
    {
        owner->gimbal_ctx.data._target_yaw_angle += owner->gimbal_ctx.cmd->yaw_delta_angle;
    }
    else
    {
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.cmd->yaw_target_angle;
    }


    if (owner->gimbal_ctx.data._target_yaw_angle > owner->gimbal_ctx.data.yaw_real_max_limit_angle)
    {
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data.yaw_real_max_limit_angle;
    }
    if (owner->gimbal_ctx.data._target_yaw_angle < owner->gimbal_ctx.data.yaw_real_min_limit_angle)
    {
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data.yaw_real_min_limit_angle;
    }

    if (owner->gimbal_ctx.cmd->pitch_target_angle > 10.0f)
    {
        owner->gimbal_ctx.data._target_pitch_angle += owner->gimbal_ctx.cmd->pitch_delta_angle;
    }
    else
    {
        owner->gimbal_ctx.data._target_pitch_angle = - owner->gimbal_ctx.cmd->pitch_target_angle;
    }

    gimbal_control(&owner->gimbal_ctx);
    send_motor_command(&owner->gimbal_ctx);
}

void uav_gimbal_t::fsm_active_t::state_auto_t::exit(uav_gimbal_t *owner)
{

}