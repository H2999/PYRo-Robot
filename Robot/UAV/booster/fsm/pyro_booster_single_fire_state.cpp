#include "pyro_uav_booster.h"

using namespace pyro;

void uav_booster_t::fsm_active_t::shoot_single_bullet_t::enter(uav_booster_t *owner)
{
    // 计算一次目标值
    owner->booster_ctx.data_ctx.target_trigger_rad += 5.0f * PI / 18.0f;
    normalize_angle(owner->booster_ctx.data_ctx.target_trigger_rad);
    // 清除指令，防止连发
    owner->booster_ctx.cmd->single_mode = false;
}

void uav_booster_t::fsm_active_t::shoot_single_bullet_t::execute(uav_booster_t *owner)
{
    owner->trigger_position_control();
    owner->send_trigger_command();


    // 判定是否到达目标位置，到达后再切回中间态
    const float error = abs(owner->booster_ctx.data_ctx.target_trigger_rad - owner->booster_ctx.data_ctx.current_trigger_rad);
    if (error < 0.05f)
    {
        request_switch(&owner->active_state.middle_state);
    }
}

void uav_booster_t::fsm_active_t::shoot_single_bullet_t::exit(uav_booster_t *owner)
{

}