#include "pyro_uav_gimbal.h"
#include "forecast_v.h"

using namespace pyro;

void uav_gimbal_t::fsm_active_t::state_auto_t::enter(uav_gimbal_t *owner)
{
    // 初始化目标角度为当前IMU角度，避免切换时抖动
    owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data._current_imu_yaw_angle;
    owner->gimbal_ctx.data._target_pitch_angle = owner->gimbal_ctx.data._current_imu_pitch_angle;
}

void uav_gimbal_t::fsm_active_t::state_auto_t::execute(uav_gimbal_t *owner)
{
    //没识别到目标的时候用遥控器控制 识别到目标的时候由自瞄控制
    if (owner->gimbal_ctx.cmd->yaw_target_angle > 80.0f || owner->gimbal_ctx.cmd->yaw_target_angle == 0.0f)
    {
        owner->gimbal_ctx.data._target_yaw_angle += owner->gimbal_ctx.cmd->yaw_delta_angle;
    }
    else
    {
        owner->gimbal_ctx.ui_ctx.is_aiming_locked = true;
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.cmd->yaw_target_angle;
    }
    //限位
    if (owner->gimbal_ctx.data._target_yaw_angle > owner->gimbal_ctx.data.yaw_real_max_limit_angle)
    {
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data.yaw_real_max_limit_angle;
    }
    if (owner->gimbal_ctx.data._target_yaw_angle < owner->gimbal_ctx.data.yaw_real_min_limit_angle)
    {
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data.yaw_real_min_limit_angle;
    }

    if (owner->gimbal_ctx.cmd->pitch_target_angle > 80.0f || owner->gimbal_ctx.cmd->pitch_target_angle == 0.0f)
    {
        owner->gimbal_ctx.data._target_pitch_angle += owner->gimbal_ctx.cmd->pitch_delta_angle;
    }
    else
    {
        owner->gimbal_ctx.data._target_pitch_angle = - owner->gimbal_ctx.cmd->pitch_target_angle;
    }

    if (owner->gimbal_ctx.data._target_pitch_angle > pitch_max_value)
    {
        owner->gimbal_ctx.data._target_pitch_angle = pitch_max_value;
    }
    else if (owner->gimbal_ctx.data._target_pitch_angle < pitch_min_value)
    {
        owner->gimbal_ctx.data._target_pitch_angle = pitch_min_value;
    }

    auto_aim_gimbal_control(&owner->gimbal_ctx);
    send_motor_command(&owner->gimbal_ctx);
}

void uav_gimbal_t::fsm_active_t::state_auto_t::exit(uav_gimbal_t *owner)
{

}