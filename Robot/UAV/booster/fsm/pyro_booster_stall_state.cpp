#include "pyro_uav_booster.h"
using namespace pyro;

void uav_booster_t::fsm_active_t::shoot_stall_t::enter(uav_booster_t *owner)
{
    owner->booster_ctx.data_ctx.target_trigger_rad = owner->booster_ctx.data_ctx.current_trigger_rad;
    owner->booster_ctx.data_ctx.target_trigger_radps = 0;

    static uint8_t stall_count = 0;

    if (&owner->active_state.stall_state == owner->active_state._last_state)
    {
        if (stall_count  < 3)
        {
            owner->booster_ctx.data_ctx.target_trigger_rad += PI / 12;
            stall_count ++;
        }
        else
        {
            stall_count = 0;
        }
    }
    else
    {
        owner->booster_ctx.data_ctx.target_trigger_rad += PI / 12;
    }
    // if (&owner->active_state.stall_state ==
    //          owner->active_state._last_state)
    // {
    // }
    // else
    // {
    //     owner->booster_ctx.data_ctx.target_trigger_rad += 0.5f;
    // }
}

void uav_booster_t::fsm_active_t::shoot_stall_t::execute(uav_booster_t *owner)
{
    // 回到合适角度后，切换回拨弹状态
    if (fabs(owner->booster_ctx.data_ctx.current_trigger_rad -
             owner->booster_ctx.data_ctx.target_trigger_rad) < 0.1f)
    {
        request_switch(&owner->active_state.interim_state);
    }

    owner->trigger_position_control();
    owner->send_trigger_command();
}

void uav_booster_t::fsm_active_t::shoot_stall_t::exit(uav_booster_t *owner)
{

}