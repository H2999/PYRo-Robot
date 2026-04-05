#include "pyro_uav_booster.h"

using namespace pyro;

void uav_booster_t::fsm_active_t::shoot_single_bullet_t::enter(uav_booster_t *owner)
{

}

void uav_booster_t::fsm_active_t::shoot_single_bullet_t::execute(uav_booster_t *owner)
{
    if (owner->booster_ctx.cmd->single_mode)
    {
        owner->booster_ctx.cmd->single_mode = false;
        owner->booster_ctx.data_ctx.target_trigger_rad += PI / 4;
        request_switch(&owner->active_state.interim_state);
    }

    if (owner->booster_ctx.data_ctx.target_trigger_rad > PI)
    {
        owner->booster_ctx.data_ctx.target_trigger_rad -= 2 * PI;
    }
    if (owner->booster_ctx.data_ctx.target_trigger_rad < -PI)
    {
        owner->booster_ctx.data_ctx.target_trigger_rad += 2 * PI;
    }

    const float error = owner->booster_ctx.data_ctx.target_trigger_rad - owner->booster_ctx.data_ctx.current_trigger_rad;
    if (error > PI)
    {
        owner->booster_ctx.data_ctx.target_trigger_rad -= 2 * PI;
    }
    if (error < -PI)
    {
        owner->booster_ctx.data_ctx.target_trigger_rad += 2 * PI;
    }

    owner->trigger_position_control();
    owner->send_trigger_command();
}

void uav_booster_t::fsm_active_t::shoot_single_bullet_t::exit(uav_booster_t *owner)
{

}