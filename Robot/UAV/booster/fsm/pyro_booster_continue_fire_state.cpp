#include "pyro_uav_booster.h"
using namespace pyro;

void uav_booster_t::fsm_active_t::shoot_continue_bullet_t::enter(uav_booster_t *owner)
{

}

void uav_booster_t::fsm_active_t::shoot_continue_bullet_t::execute(uav_booster_t *owner)
{
    if (owner->booster_ctx.cmd->continue_mode)
    {
        owner->booster_ctx.data_ctx.target_trigger_radps = 8.0f; //待定
    }
    else
    {
        owner->booster_ctx.data_ctx.target_trigger_radps = 0.0f;
        request_switch(&owner->active_state.interim_state);
    }

    const float trigger_error = owner->booster_ctx.data_ctx.target_trigger_rad - owner->booster_ctx.data_ctx.current_trigger_rad;
    if (trigger_error > PI)
    {
        owner->booster_ctx.data_ctx.target_trigger_rad -= 2 * PI;
    }
    if (trigger_error < -PI)
    {
        owner->booster_ctx.data_ctx.target_trigger_rad += 2 * PI;
    }

    owner->trigger_speed_control();
    owner->send_trigger_command();
}

void uav_booster_t::fsm_active_t::shoot_continue_bullet_t::exit(uav_booster_t *owner)
{

}