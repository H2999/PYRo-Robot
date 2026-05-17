#include "pyro_uav_booster.h"

using namespace pyro;

void uav_booster_t::fsm_active_t::shoot_auto_aim_t::enter(uav_booster_t *owner)
{

}

void uav_booster_t::fsm_active_t::shoot_auto_aim_t::execute(uav_booster_t *owner)
{
    if (owner->booster_ctx.cmd->booster_auto_flag)
    {
        const float Q_now = owner->booster_ctx.shoot_data.Q_now_no_referee;
        const float Q_res = owner->booster_ctx.shoot_data.Q_max - Q_now;

        // const float Q_res = owner->booster_ctx.shoot_data.Q_res;

        owner->booster_ctx.data_ctx.target_trigger_radps = owner->heat_control_no_referee(4, Q_res);
        // owner->booster_ctx.data_ctx.target_trigger_radps = 10.0f;
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

void uav_booster_t::fsm_active_t::shoot_auto_aim_t::exit(uav_booster_t *owner)
{

}

// void uav_booster_t::fsm_active_t::shoot_auto_aim_t::execute(uav_booster_t *owner)
// {
//     if (owner->booster_ctx.cmd->booster_auto_flag)// && owner->booster_ctx.cmd->continue_mode)
//     {
//         const int level = owner->booster_ctx.shoot_data.robot_level;
//         const float Q_res = owner->booster_ctx.shoot_data.Q_res;
//
//         owner->booster_ctx.data_ctx.target_trigger_radps = owner->heat_control(1, Q_res);
//         owner->trigger_speed_control();
//         // }
//
//         // else if (owner->booster_ctx.cmd->booster_auto_flag)// && owner->booster_ctx.cmd->single_mode)
//         // {
//         //     owner->booster_ctx.data_ctx.target_trigger_rad += PI / 4.0f;
//         //     float angle_error = owner->booster_ctx.data_ctx.target_trigger_rad - owner->booster_ctx.data_ctx.current_trigger_rad;
//         //     if (angle_error > PI)
//         //     {
//         //         owner->booster_ctx.data_ctx.target_trigger_rad -= 2.0f * PI;
//         //     }
//         //     else if (angle_error < -PI)
//         //     {
//         //         owner->booster_ctx.data_ctx.target_trigger_rad += 2.0f * PI;
//         //     }
//         //
//         //     owner->trigger_position_control();
//         //     owner->send_trigger_command();
//         //
//         //     // 判定是否到达目标位置，到达后再切回中间态
//         //     const float error = abs(owner->booster_ctx.data_ctx.target_trigger_rad - owner->booster_ctx.data_ctx.current_trigger_rad);
//         //     if (error < 0.2f)
//         //     {
//         //         owner->booster_ctx.cmd->trigger_enable = false;
//         //         request_switch(&owner->active_state.middle_state);
//         //     }
//         // }
//         // else
//         // {
//         //     owner->booster_ctx.data_ctx.target_trigger_radps = 0.0f;
//         owner->trigger_speed_control();
//
//         if (!owner->booster_ctx.cmd->booster_auto_flag)
//         {
//             request_switch(&owner->active_state.middle_state);
//         }
//     }
//
//     owner->send_trigger_command();
// }

