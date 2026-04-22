#include "pyro_uav_gimbal.h"

namespace pyro
{

void uav_gimbal_t::state_passive_t::enter(uav_gimbal_t *owner)
{
    owner->gimbal_ctx.cfg.pid_ctx.yaw_position_pid->clear();
    owner->gimbal_ctx.cfg.pid_ctx.pitch_position_pid->clear();

    owner->gimbal_ctx.cfg.pid_ctx.yaw_speed_pid->clear();
    owner->gimbal_ctx.cfg.pid_ctx.pitch_speed_pid->clear();

    owner->gimbal_ctx.cfg.motor_ctx.yaw_motor->disable();
    owner->gimbal_ctx.cfg.motor_ctx.pitch_motor->disable();
}

void uav_gimbal_t::state_passive_t::execute(uav_gimbal_t *owner)
{
    owner->gimbal_ctx.data._output_yaw_torque = 0;
    owner->gimbal_ctx.data._output_pitch_torque = 0;

    //下力时停在当前位置 防止下次上力的时候抽动回下力的位置
    owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data._current_imu_yaw_angle;
    owner->gimbal_ctx.data._target_pitch_angle = owner->gimbal_ctx.data._current_imu_pitch_angle;

    owner->gimbal_ctx.data._target_yaw_speed = 0;
    owner->gimbal_ctx.data._target_pitch_speed = 0;

    send_motor_command(&owner->gimbal_ctx);
}

void uav_gimbal_t::state_passive_t::exit(uav_gimbal_t *owner)
{
}

} // namespace pyro
