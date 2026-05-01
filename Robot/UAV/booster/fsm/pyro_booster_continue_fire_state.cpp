#include "pyro_uav_booster.h"
using namespace pyro;

void uav_booster_t::fsm_active_t::shoot_continue_bullet_t::enter(uav_booster_t *owner)
{

}

void uav_booster_t::fsm_active_t::shoot_continue_bullet_t::execute(uav_booster_t *owner)
{
    if (owner->booster_ctx.cmd->continue_mode)
    {
        // 直接调用，无需switch
        const int level = owner->booster_ctx.shoot_data.robot_level;
        const float Q_res = owner->booster_ctx.shoot_data.Q_res;

        owner->booster_ctx.data_ctx.target_trigger_radps = owner->heat_control(5, Q_res);
    }
    else
    {
        owner->booster_ctx.data_ctx.target_trigger_radps = 0.0f;
        request_switch(&owner->active_state.middle_state);
    }

    owner->trigger_speed_control();
    owner->send_trigger_command();
}

void uav_booster_t::fsm_active_t::shoot_continue_bullet_t::exit(uav_booster_t *owner)
{

}
