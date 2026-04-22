#include "pyro_uav_booster.h"

using namespace pyro;

void uav_booster_t::fsm_active_t::shoot_auto_aim_t::enter(uav_booster_t *owner)
{

}

void uav_booster_t::fsm_active_t::shoot_auto_aim_t::execute(uav_booster_t *owner)
{

    if (owner->booster_ctx.cmd->booster_auto_flag)
    {
        owner->booster_ctx.data_ctx.target_trigger_radps = 6.0f;
    }
    else
    {
        owner->booster_ctx.data_ctx.target_trigger_radps = 0.0f;
        request_switch(&owner->active_state.middle_state);
    }

    owner->trigger_speed_control();
    owner->send_trigger_command();
}

void uav_booster_t::fsm_active_t::shoot_auto_aim_t::exit(uav_booster_t *owner)
{

}
