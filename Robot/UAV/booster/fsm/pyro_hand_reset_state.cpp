#include "pyro_dwt_drv.h"
#include "pyro_uav_booster.h"

using namespace pyro;

void uav_booster_t::fsm_active_t::hand_reset_state_t::enter(uav_booster_t *owner)
{

}

void uav_booster_t::fsm_active_t::hand_reset_state_t::execute(uav_booster_t *owner)
{
    owner->trigger_position_control();
    owner->send_trigger_command();

    // 判定是否到达目标位置，到达后再切回中间态
    const float error = abs(owner->booster_ctx.data_ctx.target_trigger_rad - owner->booster_ctx.data_ctx.current_trigger_rad);
    float now_ms = dwt_drv_t::get_timeline_ms();

    if (error < 0.01f)// || ((now_ms - start_time) > timeout_ms))
    {
        owner->booster_ctx.cmd->trigger_enable = false;
        request_switch(&owner->active_state.middle_state);
    }
}

void uav_booster_t::fsm_active_t::hand_reset_state_t::exit(uav_booster_t *owner)
{

}