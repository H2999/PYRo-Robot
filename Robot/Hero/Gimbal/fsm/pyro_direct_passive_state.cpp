#include "pyro_direct_gimbal.h"

namespace pyro
{
// =========================================================
// 状态实现 - Passive (不输出)
// =========================================================

void direct_gimbal_t::state_passive_t::enter(owner *owner)
{
    owner->_ctx.pid.pitch_pos->clear();
    owner->_ctx.pid.pitch_spd->clear();
    owner->_ctx.pid.yaw_pos->clear();
    owner->_ctx.pid.yaw_spd->clear();

    owner->_ctx.motor.pitch->disable();
    owner->_ctx.motor.yaw->disable();
}

void direct_gimbal_t::state_passive_t::execute(owner *owner)
{
    owner->_ctx.data.out_pitch_torque = 0;
    owner->_ctx.data.out_yaw_torque   = 0;

    owner->_ctx.data.target_pitch_rad = owner->_ctx.data.current_pitch_rad;
    owner->_ctx.data.target_yaw_rad   = owner->_ctx.data.current_yaw_rad;

    owner->_ctx.motor.pitch->send_torque(0);
    owner->_ctx.motor.yaw->send_torque(0);
}

void direct_gimbal_t::state_passive_t::exit(owner *owner)
{
}
} // namespace pyro