#include <pyro_dr16_rc_drv.h>

#include "pyro_module_base.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
// #include <pyro_uart_message.h>
#include <pyro_core_config.h>

#include "pyro_autoaim_drv.h"
#include "pyro_dwt_drv.h"
#include "pyro_uav_booster.h"

using namespace pyro;

constexpr uint32_t DR16_FRIC_ENABLE                    = (1 << 4);
constexpr uint32_t DR16_TRIG_ENABLE                    = (1 << 5);
constexpr uint32_t DR16_TRIG_AND_FRIC_DISABLE          = (1 << 6);

constexpr uint32_t VT03_FRIC_TOGGLE                    = (1 << 0);
constexpr uint32_t VT03_TRIGGER_SINGLE                 = (1 << 1);
constexpr uint32_t VT03_TRIGGER_CONTINUE               = (1 << 2);
constexpr uint32_t VT03_TRIGGER_DISABLE                = (1 << 3);

constexpr uint32_t MOUSE_SINGLE                        = (1 << 7);
constexpr uint32_t MOUSE_CONTINUE                      = (1 << 8);
constexpr uint32_t MOUSE_DISABLE_TRIGGER               = (1 << 9);
constexpr uint32_t MOUSE_ENTER_AUTO                    = (1 << 10);
constexpr uint32_t MOUSE_DISABLE_FRIC                  = (1 << 11);
constexpr uint32_t KEY_FRIC_TOGGLE                     = (1 << 12);


static TaskHandle_t booster_task_handle       = nullptr;
uav_booster_t *uav_booster_ptr                = nullptr;
static uav_booster_cmd_t *uav_booster_cmd_ptr = nullptr;
extern autoaim_drv_t::rx_data_t rx_data;

extern "C"
{
    void booster_dr16rcmd(uint32_t notify_val)
    {
        read_scope_lock lock(dr16_drv_t::get_lock());
        const auto &vrc = rc_drv_t::read();

        static float last_up_start_tick = 0;

        if (sw_pos_t::UP == vrc.switches.right.current_pos)
        {
            uav_booster_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;

            uav_booster_cmd_ptr->fric_enable = false;
            uav_booster_cmd_ptr->trigger_enable = false;
            uav_booster_cmd_ptr->single_mode = false;
            uav_booster_cmd_ptr->continue_mode = false;
            uav_booster_cmd_ptr->booster_auto_flag = false;
            return;
        }

        uav_booster_cmd_ptr->mode = cmd_base_t::mode_t::ACTIVE;
        uav_booster_cmd_ptr->single_mode = false;

        if (sw_pos_t::MID == vrc.switches.right.current_pos)
        {
            if (notify_val & DR16_FRIC_ENABLE)
            {
                uav_booster_cmd_ptr->fric_enable = true;
                last_up_start_tick = dwt_drv_t::get_timeline_ms();
            }

            // 根据在上档停留的时间决定打一发还是扫射。
            if (notify_val & DR16_TRIG_ENABLE)
            {
                // 安全检查：只有摩擦轮开了才允许拨弹
                if (uav_booster_cmd_ptr->fric_enable)
                {
                    float stay_time = dwt_drv_t::get_timeline_ms() - last_up_start_tick;

                    uav_booster_cmd_ptr->trigger_enable = true; // 开启拨弹电机使能

                    if (stay_time < 1500)
                    {
                        // 短促拨动 (<800ms)：单发一次
                        uav_booster_cmd_ptr->single_mode = true;
                        uav_booster_cmd_ptr->continue_mode = false;
                    }
                    else
                    {
                        // 长时间停留 (>800ms)：进入连发状态
                        uav_booster_cmd_ptr->continue_mode = true;
                        uav_booster_cmd_ptr->single_mode = false;
                    }
                    last_up_start_tick = 0;
                }
            }

            if (notify_val & DR16_TRIG_AND_FRIC_DISABLE)
            {
                uav_booster_cmd_ptr->fric_enable = false;
                uav_booster_cmd_ptr->trigger_enable = false;
                uav_booster_cmd_ptr->single_mode = false;
                uav_booster_cmd_ptr->continue_mode = false;
                uav_booster_cmd_ptr->booster_auto_flag = false;
                last_up_start_tick = 0;
            }
        }

        if (sw_pos_t::DOWN == vrc.switches.right.current_pos)
        {
            // if (rx_data.fire)
            // {
            //     uav_booster_cmd_ptr->booster_auto_flag = true;
            // }
            //
            // if (notify_val & AUTO_MODE_TRIG_AND_FRIC_DISABLE)
            // {
            //     uav_booster_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;
            //     uav_booster_cmd_ptr->fric_enable = false;
            //     uav_booster_cmd_ptr->trigger_enable = false;
            //     uav_booster_cmd_ptr->booster_auto_flag = false;
            // }
            if (notify_val & DR16_FRIC_ENABLE)
            {
                uav_booster_cmd_ptr->fric_enable = true;
                last_up_start_tick = dwt_drv_t::get_timeline_ms();
            }
            if (notify_val & DR16_TRIG_ENABLE)
            {
                if (uav_booster_cmd_ptr->fric_enable)
                {
                    float stay_time = dwt_drv_t::get_timeline_ms() - last_up_start_tick;

                    uav_booster_cmd_ptr->trigger_enable = true; // 开启拨弹电机使能

                    if (stay_time < 1500)
                    {
                        // 短促拨动 (<800ms)：单发一次
                        uav_booster_cmd_ptr->single_mode = true;
                        uav_booster_cmd_ptr->continue_mode = false;
                    }
                    else
                    {
                        // 长时间停留 (>800ms)：进入连发状态
                        uav_booster_cmd_ptr->continue_mode = true;
                        uav_booster_cmd_ptr->single_mode = false;
                    }
                    last_up_start_tick = 0;
                }
            }
            if (notify_val & DR16_TRIG_AND_FRIC_DISABLE)
            {
                uav_booster_cmd_ptr->fric_enable = false;
                uav_booster_cmd_ptr->trigger_enable = false;
                uav_booster_cmd_ptr->single_mode = false;
                uav_booster_cmd_ptr->continue_mode = false;
                uav_booster_cmd_ptr->booster_auto_flag = false;
                last_up_start_tick = 0;
            }
        }
    }

void booster_vt03rcmd(uint32_t notify_val)
{
    read_scope_lock lock(vt03_drv_t::get_lock());
    const auto &vrc = rc_drv_t::read();

    if (sw_pos_t::UP == vrc.switches.gear.current_pos)
    {
        uav_booster_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;

        uav_booster_cmd_ptr->fric_enable = false;
        uav_booster_cmd_ptr->trigger_enable = false;
        uav_booster_cmd_ptr->single_mode = false;
        uav_booster_cmd_ptr->continue_mode = false;
        uav_booster_cmd_ptr->booster_auto_flag = false;
        return;
    }

    uav_booster_cmd_ptr->mode = cmd_base_t::mode_t::ACTIVE;
    uav_booster_cmd_ptr->single_mode = false;

    if (sw_pos_t::MID == vrc.switches.gear.current_pos)
    {
        if (notify_val & VT03_FRIC_TOGGLE || notify_val & KEY_FRIC_TOGGLE)
        {
            uav_booster_cmd_ptr->fric_enable = !uav_booster_cmd_ptr->fric_enable;
        }

        if (notify_val & VT03_TRIGGER_SINGLE || notify_val & MOUSE_SINGLE)
        {
            uav_booster_cmd_ptr->trigger_enable = true;
            uav_booster_cmd_ptr->single_mode = true;
        }

        if (notify_val & VT03_TRIGGER_CONTINUE || notify_val & MOUSE_CONTINUE)
        {
            uav_booster_cmd_ptr->trigger_enable = true;
            uav_booster_cmd_ptr->continue_mode = true;
            // uav_booster_cmd_ptr->booster_auto_flag = true;
        }

        if (notify_val & VT03_TRIGGER_DISABLE || notify_val & MOUSE_DISABLE_TRIGGER)
        {
            uav_booster_cmd_ptr->trigger_enable = false;
            uav_booster_cmd_ptr->single_mode = false;
            uav_booster_cmd_ptr->continue_mode = false;
            // uav_booster_cmd_ptr->booster_auto_flag = false;
        }
    }

    if (sw_pos_t::DOWN == vrc.switches.gear.current_pos || notify_val & MOUSE_ENTER_AUTO)
    {
        if (notify_val & VT03_FRIC_TOGGLE  || notify_val & KEY_FRIC_TOGGLE)
        {
            uav_booster_cmd_ptr->fric_enable = !uav_booster_cmd_ptr->fric_enable;
        }

        //如果发生意外 可以点鼠标左键停止开火
        if (notify_val & VT03_TRIGGER_DISABLE || notify_val & MOUSE_DISABLE_TRIGGER)
        {
            uav_booster_cmd_ptr->trigger_enable = false;
            uav_booster_cmd_ptr->single_mode = false;
            uav_booster_cmd_ptr->continue_mode = false;
            uav_booster_cmd_ptr->booster_auto_flag = false;
        }
        else
        {
            if (rx_data.fire)
            {
                uav_booster_cmd_ptr->trigger_enable = true;
                uav_booster_cmd_ptr->booster_auto_flag = true;
            }
            //自瞄没发开火位时才允许点按鼠标开火 防止误触
            else
            {
                if (notify_val & VT03_TRIGGER_SINGLE || notify_val & MOUSE_SINGLE)
                {
                    uav_booster_cmd_ptr->trigger_enable = true;
                    uav_booster_cmd_ptr->single_mode = true;
                }

                if (notify_val & VT03_TRIGGER_CONTINUE || notify_val & MOUSE_CONTINUE)
                {
                    uav_booster_cmd_ptr->trigger_enable = true;
                    uav_booster_cmd_ptr->continue_mode = true;
                }
            }
        }
    }
}

    void uav_booster_thread(void *argument)
    {
        while (true)
        {

            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, 0xFFFFFFFF, &notify_val, 0);

            if (dr16_drv_t::instance().check_online())
            {
                booster_dr16rcmd(notify_val);
            }
            else if (vt03_drv_t::instance().check_online())
            {
                booster_vt03rcmd(notify_val);
            }

            uav_booster_ptr->set_command(*uav_booster_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void uav_booster_init(void *argument)
    {
        uav_booster_ptr     = uav_booster_t::instance();
        uav_booster_cmd_ptr = new uav_booster_cmd_t();
        uav_booster_ptr->start();
        auto &vrc = rc_drv_t::read();

        xTaskCreate(uav_booster_thread, "uav_booster_main_thread", 512, nullptr,
                    configMAX_PRIORITIES - 3, &booster_task_handle);

        //dr16
        sw_broker::subscribe(&vrc.switches.left, sw_event_t::MID_TO_UP, booster_task_handle, DR16_FRIC_ENABLE);
        sw_broker::subscribe(&vrc.switches.left, sw_event_t::UP_TO_MID, booster_task_handle, DR16_TRIG_ENABLE);
        sw_broker::subscribe(&vrc.switches.left, sw_event_t::MID_TO_DOWN, booster_task_handle, DR16_TRIG_AND_FRIC_DISABLE);

        //vt03
        btn_broker::subscribe(&vrc.buttons.fn_l, btn_event_t::PRESS_DOWN, booster_task_handle, VT03_FRIC_TOGGLE);
        btn_broker::subscribe(&vrc.buttons.trigger, btn_event_t::PRESS_DOWN, booster_task_handle, VT03_TRIGGER_SINGLE);
        btn_broker::subscribe(&vrc.buttons.fn_r, btn_event_t::LONG_PRESS_START, booster_task_handle, VT03_TRIGGER_CONTINUE);
        btn_broker::subscribe(&vrc.buttons.fn_r, btn_event_t::PRESS_UP, booster_task_handle, VT03_TRIGGER_DISABLE);

        //mouse
        btn_broker::subscribe(&vrc.keys.q, btn_event_t::PRESS_DOWN, booster_task_handle, KEY_FRIC_TOGGLE);
        btn_broker::subscribe(&vrc.buttons.press_l, btn_event_t::PRESS_DOWN,booster_task_handle , MOUSE_SINGLE);
        btn_broker::subscribe(&vrc.buttons.press_l, btn_event_t::LONG_PRESS_START,booster_task_handle , MOUSE_CONTINUE);
        btn_broker::subscribe(&vrc.buttons.press_l, btn_event_t::PRESS_UP,booster_task_handle , MOUSE_DISABLE_TRIGGER);
        btn_broker::subscribe(&vrc.buttons.press_r, btn_event_t::LONG_PRESS_START,booster_task_handle , MOUSE_ENTER_AUTO);
        btn_broker::subscribe(&vrc.buttons.press_r, btn_event_t::PRESS_UP,booster_task_handle , MOUSE_DISABLE_FRIC);


        vTaskDelete(nullptr);
    }
}