#include "pyro_dwt_drv.h"
#include "pyro_uav_booster.h"

namespace pyro
{
void uav_booster_t::fsm_active_t::on_enter(uav_booster_t *owner)
{
    owner->booster_ctx.cfg.motor_cfg.fric_wheel[0]->enable();
    owner->booster_ctx.cfg.motor_cfg.fric_wheel[1]->enable();
    owner->booster_ctx.cfg.motor_cfg.trigger_wheel->enable();

    owner->booster_ctx.shoot_data.is_reset_finished = false;
    change_state(&middle_state);
}

void uav_booster_t::fsm_active_t::on_execute(uav_booster_t *owner)
{
    if (owner->booster_ctx.cmd->fric_enable)
    {
        owner->speed_control();

        owner->booster_ctx.data_ctx.target_fric_mps[0] = owner->booster_ctx.shoot_data.fric_mps;
        owner->booster_ctx.data_ctx.target_fric_mps[1] = -owner->booster_ctx.shoot_data.fric_mps;
    }
    else
    {
        owner->booster_ctx.data_ctx.target_fric_mps[0] = 0.0f;
        owner->booster_ctx.data_ctx.target_fric_mps[1] = 0.0f;
    }

    owner->fric_control();
    owner->send_fric_command();

    constexpr float STALL_TIME_THRESHOLD   = 600.0f; // 堵转时间阈值
    constexpr float STALL_TORQUE_THRESHOLD = 6.0f;   // 堵转扭矩阈值
    constexpr float STALL_SPEED_THRESHOLD  = 0.5f;   // 堵转速度阈值

     static float stall_start_time          = 0.0f;
     if (_active_state != &reset_state && _active_state != &middle_state)
     {
         if (abs(owner->booster_ctx.data_ctx.current_trigger_radps) < STALL_SPEED_THRESHOLD &&
         abs(owner->booster_ctx.data_ctx.current_trigger_torque) > STALL_TORQUE_THRESHOLD)
         {
             if (stall_start_time == 0.0f)
             {
                 stall_start_time = dwt_drv_t::get_timeline_ms();
             }
             else
             {
                 const float elapsed_time =
                     dwt_drv_t::get_timeline_ms() - stall_start_time;
                 if (elapsed_time >= STALL_TIME_THRESHOLD)
                 {
                     // 进入堵转状态
                     change_state(&stall_state);
                     if (_active_state == &stall_state)
                     {
                         reset();
                     }
                     stall_start_time = 0.0f; // 重置堵转计时
                 }
             }
         }
         else
         {
             stall_start_time = 0.0f;
         }
     }
}

void uav_booster_t::fsm_active_t::on_exit(uav_booster_t *owner)
{
}

}

