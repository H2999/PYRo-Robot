#include "pyro_uav_booster.h"
using namespace pyro;

void uav_booster_t::fsm_active_t::state_middle_t::enter(uav_booster_t *owner)
{
    owner->booster_ctx.data_ctx.target_trigger_rad = owner->booster_ctx.data_ctx.current_trigger_rad;
}

void uav_booster_t::fsm_active_t::state_middle_t::execute(uav_booster_t *owner)
{
    if (abs(owner->booster_ctx.data_ctx.current_fric_mps[0] -
            owner->booster_ctx.data_ctx.target_fric_mps[0]) < 0.7f &&
        abs(owner->booster_ctx.data_ctx.current_fric_mps[1] -
            owner->booster_ctx.data_ctx.target_fric_mps[1]) < 0.7f)
    {
        if (owner->booster_ctx.cmd->trigger_enable)
        {
            if (!owner->booster_ctx.shoot_data.is_reset_finished)
            {
                request_switch(&owner->active_state.reset_state);
            }
            else
            {
                if (owner->booster_ctx.cmd->single_mode)
                {
                    request_switch(&owner->active_state.single_state);
                }

                if (owner->booster_ctx.cmd->continue_mode)
                {
                    request_switch(&owner->active_state.continue_state);
                }

                if (owner->booster_ctx.cmd->booster_auto_flag)
                {
                    request_switch(&owner->active_state.auto_aim_state);
                }
            }
        }
    }

    owner->trigger_position_control();
    owner->send_trigger_command();

}

void uav_booster_t::fsm_active_t::state_middle_t::exit(uav_booster_t *owner)
{

}