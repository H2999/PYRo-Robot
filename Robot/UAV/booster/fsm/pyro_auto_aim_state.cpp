#include "pyro_uav_booster.h"

using namespace pyro;

void uav_booster_t::fsm_active_t::shoot_auto_aim_t::enter(uav_booster_t *owner)
{

}

void uav_booster_t::fsm_active_t::shoot_auto_aim_t::execute(uav_booster_t *owner)
{

    owner->booster_ctx.data_ctx.target_trigger_radps = 6.0f;

    // const float trigger_error = owner->booster_ctx.data_ctx.target_trigger_rad - owner->booster_ctx.data_ctx.current_trigger_rad;
    // if (trigger_error > PI)
    // {
    //     owner->booster_ctx.data_ctx.target_trigger_rad -= 2 * PI;
    // }
    // if (trigger_error < -PI)
    // {
    //     owner->booster_ctx.data_ctx.target_trigger_rad += 2 * PI;
    // }

    owner->trigger_speed_control();
    owner->send_trigger_command();
}

void uav_booster_t::fsm_active_t::shoot_auto_aim_t::exit(uav_booster_t *owner)
{

}
