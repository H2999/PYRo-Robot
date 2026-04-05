#include "pyro_uav_booster.h"

namespace pyro
{
void uav_booster_t::passive_state_t::enter(uav_booster_t *owner)
{
    owner->booster_ctx.cfg.pid_cfg.trigger_position_pid->clear();
    owner->booster_ctx.cfg.pid_cfg.trigger_speed_pid->clear();
}

void uav_booster_t::passive_state_t::execute(uav_booster_t *owner)
{
    owner->booster_ctx.data_ctx.target_fric_mps[0] = 0.0f;
    owner->booster_ctx.data_ctx.target_fric_mps[1] = 0.0f;

    if (abs(owner->booster_ctx.data_ctx.current_fric_mps[0]) < 0.3f)
        owner->booster_ctx.data_ctx.fric_output_torque[0] = 0.0f;
    if (abs(owner->booster_ctx.data_ctx.current_fric_mps[1]) < 0.3f)
        owner->booster_ctx.data_ctx.fric_output_torque[1] = 0.0f;

    owner->fric_control();
    owner->send_fric_command();

    owner->booster_ctx.data_ctx.target_trigger_rad = owner->booster_ctx.data_ctx.current_trigger_rad;
    owner->trigger_position_control();

    if (abs(owner->booster_ctx.data_ctx.current_trigger_radps) < 0.4f)
    {
        owner->booster_ctx.data_ctx.trigger_output_torque = 0.0f;
    }

    owner->trigger_speed_control();
    owner->send_trigger_command();
}

void uav_booster_t::passive_state_t::exit(uav_booster_t *owner)
{

}

}


