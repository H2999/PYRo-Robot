#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_direct_gimbal.h"
#include "pyro_com_cantx.h"
#include "pyro_quad_booster.h"
#include "pyro_com_canrx.h"
#include "struct.h"

using namespace pyro;

extern StateBytes state_bytes;
extern float read_time;

// 定义任务通知的位掩码 (Event Bits)
constexpr uint32_t EVENT_BIT_FRIC_TOGGLE   = (1 << 0);
constexpr uint32_t EVENT_BIT_FIRE          = (1 << 1);
constexpr uint32_t EVENT_BIT_FRIC1_ADJ     = (1 << 2);
constexpr uint32_t EVENT_BIT_FRIC2_ADJ     = (1 << 3);
constexpr uint32_t EVENT_BIT_SPEED_CTRL    = (1 << 4);
constexpr uint32_t EVENT_BIT_RESET_TRIG    = (1 << 5);

static TaskHandle_t booster_task_handle               = nullptr;
static pyro::quad_booster_t *quad_booster_ptr         = nullptr;
static pyro::quad_booster_cmd_t *quad_booster_cmd_ptr = nullptr;
static bool flush_flag                                = false;

void booster_dr162cmd(uint32_t notify_val)
{
    pyro::read_scope_lock lock(pyro::dr16_drv_t::instance().get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (pyro::sw_pos_t::DOWN == vrc.switches.right.current_pos)
    {
        quad_booster_cmd_ptr->mode        = pyro::cmd_base_t::mode_t::PASSIVE;
        quad_booster_cmd_ptr->fric_on     = false;
        return;
    }
    
    quad_booster_cmd_ptr->mode         = pyro::cmd_base_t::mode_t::ACTIVE;
    quad_booster_cmd_ptr->target_speed = 11.5f; // 可调节
    
    // 摩擦轮控制
    if (notify_val & EVENT_BIT_FRIC_TOGGLE)
    {
        quad_booster_cmd_ptr->fric_on = !quad_booster_cmd_ptr->fric_on;
    }

    if (pyro::sw_pos_t::MID == vrc.switches.right.current_pos)
    {
        // 只有在中档时才响应摇杆的单发开火事件
        if (notify_val & EVENT_BIT_FIRE)
        {
            quad_booster_cmd_ptr->fire_enable = true;
        }
    }
    else // SW_UP
    {
        static bool autoaim_fire_flag = false;
        if (pyro::sw_pos_t::UP == vrc.switches.right.current_pos)
        {
            if (state_bytes.input_data.fire == 0)
            {
                autoaim_fire_flag = true;
            }
            if (autoaim_fire_flag)
            {
                if (state_bytes.input_data.fire == 1)
                {
                    quad_booster_cmd_ptr->fire_enable = true;
                }
            }
        }
    }
}

void booster_vt032cmd(uint32_t notify_val)
{
    pyro::read_scope_lock lock(pyro::vt03_drv_t::instance().get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    // Ctrl + C 清弹标志位连续状态
    flush_flag = vrc.keys.ctrl.current_level && vrc.keys.c.current_level;

    // 组合键：修饰键 G + R / F 调节摩擦轮初速度
    if (notify_val & EVENT_BIT_FRIC1_ADJ)
    {
        if (vrc.keys.g.current_level)
            quad_booster_ptr->get_ctx().shoot_data.fric1_mps -= 0.1f;
        else
            quad_booster_ptr->get_ctx().shoot_data.fric1_mps += 0.1f;
    }

    if (notify_val & EVENT_BIT_FRIC2_ADJ)
    {
        if (vrc.keys.g.current_level)
            quad_booster_ptr->get_ctx().shoot_data.fric2_mps -= 0.1f;
        else
            quad_booster_ptr->get_ctx().shoot_data.fric2_mps += 0.1f;
    }

    // V 键切换控速
    if (notify_val & EVENT_BIT_SPEED_CTRL)
    {
        quad_booster_cmd_ptr->speed_contorl_en = !quad_booster_cmd_ptr->speed_contorl_en;
    }

    // B 键重置拨弹轮 (产生单帧脉冲)
    if (notify_val & EVENT_BIT_RESET_TRIG)
    {
        quad_booster_cmd_ptr->reset_trig = true;
    }

    if (pyro::sw_pos_t::UP == vrc.switches.gear.current_pos) // GEAR_LEFT
    {
        quad_booster_cmd_ptr->mode        = pyro::cmd_base_t::mode_t::PASSIVE;
        quad_booster_cmd_ptr->fric_on     = false;
        return;
    }
    
    quad_booster_cmd_ptr->mode         = pyro::cmd_base_t::mode_t::ACTIVE;
    quad_booster_cmd_ptr->target_speed = 11.5f; // 可调节
    
    // 摩擦轮开关控制 (Fn_L 或 Q)
    if (notify_val & EVENT_BIT_FRIC_TOGGLE)
    {
        quad_booster_cmd_ptr->fric_on = !quad_booster_cmd_ptr->fric_on;
    }

    // Auto-aim 开火逻辑
    static bool autoaim_fire_flag = false;
    if (pyro::sw_pos_t::DOWN == vrc.switches.gear.current_pos) // GEAR_RIGHT
    {
        if (state_bytes.input_data.fire == 0)
        {
            autoaim_fire_flag = true;
        }
        if (autoaim_fire_flag)
        {
            if (state_bytes.input_data.fire == 1)
            {
                quad_booster_cmd_ptr->fire_enable = true;
            }
        }
    }
    
    // 手动单发开火控制
    if (notify_val & EVENT_BIT_FIRE)
    {
        quad_booster_cmd_ptr->fire_enable = true;
    }
}

void booster2ui()
{
    pyro::can_tx_drv_t::clear(0x110);
    static auto fric1_speed = static_cast<int8_t>(
        quad_booster_ptr->get_ctx().shoot_data.fric1_mps * 10);
    static auto fric2_speed = static_cast<int8_t>(
        quad_booster_ptr->get_ctx().shoot_data.fric2_mps * 10);
    static bool fric_on               = false;
    static bool speed_control_enabled = false;
    static bool fric1_online;
    static bool fric2_online;
    
    if (quad_booster_cmd_ptr->fric_on)
    {
        fric_on     = true;
        fric1_speed = static_cast<int8_t>(
            quad_booster_ptr->get_ctx().shoot_data.fric1_mps * 10);
        fric2_speed = static_cast<int8_t>(
            quad_booster_ptr->get_ctx().shoot_data.fric2_mps * 10);
        // 注意：保留结构体里的原始拼写 speed_contorl_en
        speed_control_enabled = quad_booster_cmd_ptr->speed_contorl_en;
        
        if (quad_booster_ptr->get_ctx().data.fric_online[1] &&
            quad_booster_ptr->get_ctx().data.fric_online[3])
        {
            fric1_online = true;
        }
        else
        {
            fric1_online = false;
        }
        if (quad_booster_ptr->get_ctx().data.fric_online[0] &&
            quad_booster_ptr->get_ctx().data.fric_online[2])
        {
            fric2_online = true;
        }
        else
        {
            fric2_online = false;
        }
    }
    else
    {
        fric_on     = false;
        fric1_speed = 0;
        fric2_speed = 0;
    }
    
    pyro::can_tx_drv_t::add_data(0x110, 8, fric1_speed);
    pyro::can_tx_drv_t::add_data(0x110, 8, fric2_speed);
    pyro::can_tx_drv_t::add_data(0x110, 1, fric_on);
    pyro::can_tx_drv_t::add_data(0x110, 1, flush_flag);
    pyro::can_tx_drv_t::add_data(0x110, 1, speed_control_enabled);
    pyro::can_tx_drv_t::add_data(0x110, 1, fric1_online);
    pyro::can_tx_drv_t::add_data(0x110, 1, fric2_online);

    pyro::can_tx_drv_t::send(0x110,
                             pyro::can_hub_t::get_instance()->hub_get_can_obj(
                                 pyro::can_hub_t::which_can::can3));
}

extern "C"
{
    void hero_booster_thread(void *argument)
    {
        while (true)
        {
            uint32_t notify_val = 0;
            // 非阻塞提取任务通知（超时为0），捕获按键与拨杆脉冲
            xTaskNotifyWait(0x00, 0xFFFFFFFF, &notify_val, 0);

            // 【架构精髓】：在此处默认重置所有脉冲触发变量，形成严格的 1 帧电平脉冲
            quad_booster_cmd_ptr->fire_enable = false;
            quad_booster_cmd_ptr->reset_trig  = false;

            if (vt03_drv_t::instance().check_online())
            {
                booster_vt032cmd(notify_val);
            }
            else if (dr16_drv_t::instance().check_online())
            {
                booster_dr162cmd(notify_val);
            }
            booster2ui();
            quad_booster_ptr->set_command(*quad_booster_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_booster_init(void *argument)
    {
        quad_booster_ptr     = pyro::quad_booster_t::instance();
        quad_booster_cmd_ptr = new pyro::quad_booster_cmd_t();
        
        quad_booster_ptr->start();

        // 1. 获取任务句柄
        xTaskCreate(hero_booster_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, &booster_task_handle);

        // 2. 利用泛型 Broker 登记所有会触发动作的事件
        auto &vrc = pyro::rc_drv_t::read();
        
        // --- VT03 按键绑定 ---
        pyro::btn_broker::subscribe(&vrc.buttons.fn_l, pyro::btn_event_t::PRESS_DOWN, booster_task_handle, EVENT_BIT_FRIC_TOGGLE);
        pyro::btn_broker::subscribe(&vrc.keys.q, pyro::btn_event_t::PRESS_DOWN, booster_task_handle, EVENT_BIT_FRIC_TOGGLE);
        
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::PRESS_DOWN, booster_task_handle, EVENT_BIT_FIRE);
        pyro::btn_broker::subscribe(&vrc.buttons.press_l, pyro::btn_event_t::PRESS_DOWN, booster_task_handle, EVENT_BIT_FIRE);
        
        pyro::btn_broker::subscribe(&vrc.keys.r, pyro::btn_event_t::PRESS_DOWN, booster_task_handle, EVENT_BIT_FRIC1_ADJ);
        pyro::btn_broker::subscribe(&vrc.keys.f, pyro::btn_event_t::PRESS_DOWN, booster_task_handle, EVENT_BIT_FRIC2_ADJ);
        
        pyro::btn_broker::subscribe(&vrc.keys.v, pyro::btn_event_t::PRESS_DOWN, booster_task_handle, EVENT_BIT_SPEED_CTRL);
        pyro::btn_broker::subscribe(&vrc.keys.b, pyro::btn_event_t::PRESS_DOWN, booster_task_handle, EVENT_BIT_RESET_TRIG);

        // --- DR16 拨杆绑定 ---
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::UP_TO_MID, booster_task_handle, EVENT_BIT_FRIC_TOGGLE);
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::DOWN_TO_MID, booster_task_handle, EVENT_BIT_FIRE);

        vTaskDelete(nullptr);
    }
}