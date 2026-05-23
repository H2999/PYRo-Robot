#include "pyro_uav_booster.h"
using namespace pyro;

void uav_booster_t::fsm_active_t::shoot_continue_bullet_t::enter(uav_booster_t *owner)
{
    owner->booster_ctx.heat_control_ctx.continue_fresh_referee = true;
}

void uav_booster_t::fsm_active_t::shoot_continue_bullet_t::execute(uav_booster_t *owner)
{
    if (owner->booster_ctx.cmd->continue_mode)
    {
        if (owner->booster_ctx.heat_control_ctx.allow_bullet_count > 0)
        {
            owner->booster_ctx.data_ctx.target_trigger_radps = 10.0f;
        }
        else
        {
            // 热量满了 急停
            owner->booster_ctx.data_ctx.target_trigger_radps = 0.0f;
        }
    }
    else
    {
        // 停止逻辑
        owner->booster_ctx.data_ctx.target_trigger_radps = 0.0f;
        request_switch(&owner->active_state.middle_state);
    }
    owner->trigger_speed_control();
    // 最终执行输出
    owner->send_trigger_command();
}
void uav_booster_t::fsm_active_t::shoot_continue_bullet_t::exit(uav_booster_t *owner)
{
    owner->booster_ctx.heat_control_ctx.continue_fresh_referee = false;
}
