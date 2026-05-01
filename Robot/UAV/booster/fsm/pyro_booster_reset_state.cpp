#include "pyro_dwt_drv.h"
#include "pyro_uav_booster.h"
using namespace pyro;

// void uav_booster_t::fsm_active_t::reset_state_t::execute(uav_booster_t *owner)
// {
//     const float turnback_time =
//             dwt_drv_t::get_timeline_ms() - turnback_start_time;
//     if (turnback_time > 1200.0f)
//     {
//         // owner->booster_ctx.data_ctx.target_trigger_rad = PI / 5;
//         // owner->trigger_position_control();
//         // // owner->send_trigger_command();
//         // if (owner->booster_ctx.data_ctx.target_trigger_rad - owner->booster_ctx.data_ctx.current_trigger_rad < 0.2f)
//         // {
//             // owner->booster_ctx.shoot_data.is_reset_finished = true;
//             request_switch(&owner->active_state.middle_state);
//             return;
//         // }
//     }
//     else
//     {
//         owner->booster_ctx.data_ctx.target_trigger_radps = - 5.0f;
//         owner->trigger_speed_control();
//     }
//     owner->send_trigger_command();
// }

void uav_booster_t::fsm_active_t::reset_state_t::enter(uav_booster_t *owner)
{
    turnback_start_time = dwt_drv_t::get_timeline_ms();
}

void uav_booster_t::fsm_active_t::reset_state_t::execute(uav_booster_t *owner)
{
    const float turnback_time = dwt_drv_t::get_timeline_ms() - turnback_start_time;

    // 反转持续时间
    if (turnback_time < 1000.0f)
    {
        owner->booster_ctx.data_ctx.target_trigger_radps = -5.0f;
        owner->trigger_speed_control();
    }
    else
    {
        owner->booster_ctx.shoot_data.is_reset_finished = true;
        request_switch(&owner->active_state.middle_state);
        return;
    }

    owner->send_trigger_command();
}

void uav_booster_t::fsm_active_t::reset_state_t::exit(uav_booster_t *owner)
{
    // 退出时清理速度目标，防止干扰其他状态
    owner->booster_ctx.data_ctx.target_trigger_radps = 0.0f;
}