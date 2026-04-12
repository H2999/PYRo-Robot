#include "pyro_uav_gimbal.h"

namespace pyro
{
void uav_gimbal_t::fsm_active_t::on_enter(uav_gimbal_t *owner)
{
    owner->gimbal_ctx.cfg.motor_ctx.yaw_motor->enable();
    owner->gimbal_ctx.cfg.motor_ctx.pitch_motor->enable();
}

void uav_gimbal_t::fsm_active_t::on_execute(uav_gimbal_t *owner)
{
    if (owner->gimbal_ctx.auto_ctx.auto_enable)
    {
        change_state(&auto_state);
    }
    else
    {
        change_state(&rc_state);
    }

}

void uav_gimbal_t::fsm_active_t::on_exit(uav_gimbal_t *owner)
{

}

} // namespace pyro
