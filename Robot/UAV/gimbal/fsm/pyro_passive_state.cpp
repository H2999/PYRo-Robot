#include "pyro_uav_gimbal.h"

namespace pyro
{

void uav_gimbal_t::state_passive_t::enter(uav_gimbal_t *owner)
{
    owner->gimbal_ctx.pid.yaw_position_pid->clear();
    owner->gimbal_ctx.pid.pitch_position_pid->clear();
    owner->gimbal_ctx.pid.roll_position_pid->clear();

    owner->gimbal_ctx.pid.yaw_speed_pid->clear();
    owner->gimbal_ctx.pid.pitch_speed_pid->clear();
    owner->gimbal_ctx.pid.roll_speed_pid->clear();

    owner->gimbal_ctx.motor.yaw_motor->disable();
    owner->gimbal_ctx.motor.pitch_motor->disable();
    owner->gimbal_ctx.motor.roll_motor->disable();
}

void uav_gimbal_t::state_passive_t::execute(uav_gimbal_t *owner)
{
    owner->gimbal_ctx.data._output_yaw_torque = 0;
    owner->gimbal_ctx.data._output_pitch_torque = 0;
    owner->gimbal_ctx.data._output_roll_torque = 0;

    //下力时停在当前位置 防止下次上力的时候抽动回下力的位置
    owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data._current_imu_yaw_angle;
    owner->gimbal_ctx.data._target_pitch_angle = owner->gimbal_ctx.data._current_imu_pitch_angle;
    owner->gimbal_ctx.data._target_roll_angle = owner->gimbal_ctx.data._current_imu_roll_angle;

    owner->gimbal_ctx.data._target_yaw_speed = 0;
    owner->gimbal_ctx.data._target_pitch_speed = 0;
    owner->gimbal_ctx.data._target_roll_speed = 0;

    // gimbal_control(&owner->gimbal_ctx);
    send_motor_command(&owner->gimbal_ctx);
}

void uav_gimbal_t::state_passive_t::exit(uav_gimbal_t *owner)
{
}

} // namespace pyro
