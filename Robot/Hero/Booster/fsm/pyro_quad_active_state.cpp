#include "pyro_quad_booster.h"
#include "pyro_dwt_drv.h"

namespace pyro
{

void quad_booster_t::fsm_active_t::on_enter(owner *owner)
{
    owner->_ctx.motor.trigger_wheel->enable();
    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.motor.fric_wheels[i]->enable();
    }
    change_state(&_homing_state);
}

void quad_booster_t::fsm_active_t::on_execute(owner *owner)
{
    if (owner->_ctx.cmd->reset_trig)
    {
        change_state(&_homing_state);
    }
    // 1. 摩擦轮控制
    if (owner->_ctx.cmd->speed_contorl_en)
    {
        owner->_speed_contorl();
    }

    if (owner->_ctx.cmd->fric_on)
    {
        owner->_ctx.data.target_fric_mps[0] = owner->_ctx.shoot_data.fric2_mps;
        owner->_ctx.data.target_fric_mps[2] = -owner->_ctx.shoot_data.fric2_mps;
        owner->_ctx.data.target_fric_mps[1] = owner->_ctx.shoot_data.fric1_mps;
        owner->_ctx.data.target_fric_mps[3] = -owner->_ctx.shoot_data.fric1_mps;

        // owner->_ctx.data.target_fric_mps[0] = 0;
        // owner->_ctx.data.target_fric_mps[2] = 0;
        // owner->_ctx.data.target_fric_mps[1] = 0;
        // owner->_ctx.data.target_fric_mps[3] = 0;

        owner->_fric_control();
    }
    else
    {
        owner->_ctx.data.target_fric_mps[0] = 0.0f;
        owner->_ctx.data.target_fric_mps[1] = 0.0f;
        owner->_ctx.data.target_fric_mps[2] = 0.0f;
        owner->_ctx.data.target_fric_mps[3] = 0.0f;
        owner->_fric_control();
        for (int i = 0; i < 4; i++)
        {
            if (abs(owner->_ctx.data.current_fric_mps[i]) < 0.3f)
                owner->_ctx.data.out_fric_torque[i] = 0.0f;
        }
    }
    owner->_send_fric_command();

    // 2. 发弹延迟计算
    // 通过外级摩擦轮转速和扭矩判断是否发弹
    // 计算信号发生时间（在ready状态中获取）到当前时间的差值
    owner->_launch_delay_calculate();


    // 3. 拨弹盘堵转判断
    // 通过拨盘电机的速度和扭矩判断是否堵转
    constexpr float STALL_TIME_THRESHOLD   = 500.0f; // 堵转时间阈值
    constexpr float HEAT_TIME              = 2000.0f;
    constexpr float HEAT_TORQUE            = 10.0f;
    constexpr float STALL_TORQUE_THRESHOLD = 5.0f;   // 堵转扭矩阈值
    constexpr float STALL_SPEED_THRESHOLD  = 0.3f;   // 堵转速度阈值

    static float stall_start_time          = 0.0f;
    static uint16_t clear_stall_counter    = 0;      // 新增：用于防抖计时的计数器

    if (abs(owner->_ctx.data.current_trig_radps) < STALL_SPEED_THRESHOLD &&
        abs(owner->_ctx.data.current_trig_torque) > STALL_TORQUE_THRESHOLD)
    {
        // 只要满足堵转条件，立刻清空“退出堵转”的计数器
        clear_stall_counter = 0;

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
                change_state(&_stall_state);
                if (_active_state == &_stall_state)
                {
                    reset();
                }
                stall_start_time = 0.0f; // 触发反转后重置堵转计时
            }
        }
    }
    else
    {
        // 不满足堵转条件时，防抖逻辑启动
        if (stall_start_time != 0.0f)
        {
            clear_stall_counter++;
            if (clear_stall_counter >= 8) // 连续20个周期不满足堵转条件
            {
                stall_start_time = 0.0f;   // 真正重置堵转计时
                clear_stall_counter = 0;   // 计数器归零
            }
        }
        else
        {
            clear_stall_counter = 0; // 平时未触发堵转检测时，保持计数器为0
        }
    }

    // 4. 拨弹盘热量（过载）保护逻辑
    static float heat_stall_time = 0.0f;
    static uint16_t clear_heat_counter = 0; // 新增：用于热量保护防抖的计数器

    if (abs(owner->_ctx.data.current_trig_torque) > HEAT_TORQUE)
    {
        // 只要扭矩超标，立刻清空“退出过载”的计数器
        clear_heat_counter = 0;

        if (heat_stall_time == 0.0f)
        {
            heat_stall_time = dwt_drv_t::get_timeline_ms();
        }
        else
        {
            const float elapsed_time =
                dwt_drv_t::get_timeline_ms() - heat_stall_time;
            if (elapsed_time >= HEAT_TIME)
            {
                // 持续高扭矩超过 1500ms，切断电机输出以保护硬件
                owner->_ctx.motor.trigger_wheel->disable();
            }
        }
    }
    else
    {
        // 扭矩回落到安全范围内时，防抖逻辑启动
        if (heat_stall_time != 0.0f)
        {
            clear_heat_counter++;
            if (clear_heat_counter >= 10) // 连续20个周期扭矩低于阈值
            {
                heat_stall_time = 0.0f;   // 真正重置热量计时
                clear_heat_counter = 0;   // 计数器归零
            }
        }
        else
        {
            clear_heat_counter = 0; // 平时未触发时，保持计数器为0
        }
    }
}

void quad_booster_t::fsm_active_t::on_exit(owner *owner)
{
}

} // namespace pyro