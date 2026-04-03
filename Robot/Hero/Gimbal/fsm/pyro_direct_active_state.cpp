#include "pyro_direct_gimbal.h"

namespace pyro
{
// =========================================================
// 状态实现 - Active (控制模式)
// =========================================================

void direct_gimbal_t::state_active_t::enter(owner *owner)
{
    static_cast<dm_motor_drv_t*>(owner->_ctx.motor.pitch)->clear_error();
    owner->_ctx.motor.pitch->enable();
    owner->_ctx.motor.yaw->enable();
}

void direct_gimbal_t::state_active_t::execute(owner *owner)
{
    owner->_ctx.data.target_pitch_rad += owner->_ctx.cmd->pitch_delta_angle;
    owner->_ctx.data.target_yaw_rad += owner->_ctx.cmd->yaw_delta_angle;

    // 限幅
    if (owner->_ctx.data.target_pitch_rad > PITCH_MAX_RAD)
    {
        owner->_ctx.data.target_pitch_rad = PITCH_MAX_RAD;
    }
    else if (owner->_ctx.data.target_pitch_rad < PITCH_MIN_RAD)
    {
        owner->_ctx.data.target_pitch_rad = PITCH_MIN_RAD;
    }

    // 处理 Yaw 轴过零点
    const float yaw_error =
        owner->_ctx.data.target_yaw_rad - owner->_ctx.data.current_yaw_rad;
    if (yaw_error > PI)
    {
        owner->_ctx.data.target_yaw_rad -= 2.0f * PI;
    }
    else if (yaw_error < -PI)
    {
        owner->_ctx.data.target_yaw_rad += 2.0f * PI;
    }


    _gimbal_control(&owner->_ctx);

    _send_motor_command(&owner->_ctx);
}

void direct_gimbal_t::state_active_t::exit(owner *owner)
{
}
} // namespace pyro