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
constexpr uint32_t MOUSE_EXIT_AUTO                    = (1 << 11);
constexpr uint32_t MOUSE_DISABLE_FRIC                  = (1 << 12);
constexpr uint32_t KEY_FRIC_TOGGLE                     = (1 << 13);


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

        uav_booster_ptr->get_data()->data_ctx.distance = rx_data.shoot_dist;

        //存储鼠标状态
        static bool mouse_aiming = false;
        // 收到长按开始信号
        if (notify_val & MOUSE_ENTER_AUTO) mouse_aiming = true;
        // 收到弹起信号
        if (notify_val & MOUSE_EXIT_AUTO)  mouse_aiming = false;

        //把自瞄挡放这 先响应自瞄挡
        /*
        // ==================== 2. 核心状态机判定 ====================
        // 判断当前是否处于有效的“可开火控制状态”（拨码向下 OR 鼠标右键长按自瞄）
//         if (sw_pos_t::DOWN == vrc.switches.gear.current_pos || mouse_aiming)
//         {
//             // 如果摩擦轮开了，才允许计算开火逻辑
//             if (uav_booster_cmd_ptr->fric_enable)
//             {
//                 // 优先响应手动单发/连发通知（用 else if 隔开，防止被 rx_data.fire 覆盖）
//                 if (notify_val & VT03_TRIGGER_SINGLE || notify_val & MOUSE_SINGLE)
//                 {
//                     uav_booster_cmd_ptr->trigger_enable   = true;
//                     uav_booster_cmd_ptr->single_mode      = true;
//                     uav_booster_cmd_ptr->continue_mode    = false;
//                     uav_booster_cmd_ptr->booster_auto_flag = false; // 手动优先
//                 }
//                 else if (notify_val & VT03_TRIGGER_CONTINUE || notify_val & MOUSE_CONTINUE)
//                 {
//                     uav_booster_cmd_ptr->trigger_enable   = true;
//                     uav_booster_cmd_ptr->continue_mode    = true;
//                     uav_booster_cmd_ptr->single_mode      = false;
//                     uav_booster_cmd_ptr->booster_auto_flag = false; // 手动优先
//                 }
//                 // 如果没有手动开火通知，则看视觉自瞄是否下发了开火指令
//                 else
//                 {
//                     if (rx_data.fire)
//                     {
//                         uav_booster_cmd_ptr->trigger_enable    = true;
//                         uav_booster_cmd_ptr->booster_auto_flag = true;
//                     }
//                     else
//                     {
//                         // 既没手动打弹，自瞄也没识别到目标，停转拨弹盘
//                         uav_booster_cmd_ptr->trigger_enable    = false;
//                         uav_booster_cmd_ptr->booster_auto_flag = false;
//                     }
//                 }
//             }
//             else
//             {
//                 // 摩擦轮关闭状态：虽然在自瞄状态，但强行关闭拨弹盘
//                 uav_booster_cmd_ptr->trigger_enable    = false;
//                 uav_booster_cmd_ptr->single_mode       = false;
//                 uav_booster_cmd_ptr->continue_mode     = false;
//                 uav_booster_cmd_ptr->booster_auto_flag = false;
//             }
//         }
//         else
//         {
//             uav_booster_cmd_ptr->trigger_enable    = false;
//             uav_booster_cmd_ptr->single_mode       = false;
//             uav_booster_cmd_ptr->continue_mode     = false;
//             uav_booster_cmd_ptr->booster_auto_flag = false;
//
//             uav_booster_cmd_ptr->fric_enable       = false;
//         }
*/
        if (sw_pos_t::DOWN == vrc.switches.gear.current_pos || mouse_aiming)
        {
            if (notify_val & VT03_FRIC_TOGGLE || notify_val & KEY_FRIC_TOGGLE)
            {
                uav_booster_cmd_ptr->fric_enable = !uav_booster_cmd_ptr->fric_enable;
            }

            // 如果摩擦轮开了，才允许计算开火逻辑
            if (uav_booster_cmd_ptr->fric_enable)
            {
                // 2. 核心联动：必须自瞄判定可开火（fire） 并且 满足人手长按
                // if (rx_data.fire && notify_val & MOUSE_CONTINUE)
                if (rx_data.fire)
                {
                    uav_booster_cmd_ptr->trigger_enable    = true;
                    uav_booster_cmd_ptr->continue_mode     = false;  // 连续开火
                    uav_booster_cmd_ptr->single_mode       = false;
                    uav_booster_cmd_ptr->booster_auto_flag = true;  // 成功进入自瞄开火状态
                }
                // 3. 纯手动单发兜底
                else if (notify_val & VT03_TRIGGER_SINGLE || notify_val & MOUSE_SINGLE)
                {
                    uav_booster_cmd_ptr->trigger_enable    = true;
                    uav_booster_cmd_ptr->single_mode       = true;
                    uav_booster_cmd_ptr->continue_mode     = false;
                    uav_booster_cmd_ptr->booster_auto_flag = false; // 纯手动单发
                }
                // 4. 条件不满足时的安全锁：自瞄丢目标了，或者手松开了，立刻退出自瞄开火
                else
                {
                    uav_booster_cmd_ptr->trigger_enable    = false;
                    uav_booster_cmd_ptr->single_mode       = false;
                    uav_booster_cmd_ptr->continue_mode     = false;
                    uav_booster_cmd_ptr->booster_auto_flag = false; // 退出自瞄开火
                }
            }
            else
            {
                // 摩擦轮关闭状态：强行关闭拨弹盘
                uav_booster_cmd_ptr->trigger_enable    = false;
                uav_booster_cmd_ptr->single_mode       = false;
                uav_booster_cmd_ptr->continue_mode     = false;
                uav_booster_cmd_ptr->booster_auto_flag = false;
                uav_booster_cmd_ptr->fric_enable       = false;
            }
        }

        else if (sw_pos_t::MID == vrc.switches.gear.current_pos)
        {
            uav_booster_cmd_ptr->booster_auto_flag = false;

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
        btn_broker::subscribe(&vrc.buttons.press_l, btn_event_t::PRESS_DOWN,booster_task_handle , MOUSE_CONTINUE);
        btn_broker::subscribe(&vrc.buttons.press_l, btn_event_t::PRESS_UP,booster_task_handle , MOUSE_DISABLE_TRIGGER);
        btn_broker::subscribe(&vrc.buttons.press_r, btn_event_t::LONG_PRESS_START,booster_task_handle , MOUSE_ENTER_AUTO);
        btn_broker::subscribe(&vrc.buttons.press_r, btn_event_t::PRESS_UP,booster_task_handle , MOUSE_EXIT_AUTO);
        btn_broker::subscribe(&vrc.buttons.press_r, btn_event_t::PRESS_UP,booster_task_handle , MOUSE_DISABLE_FRIC);


        vTaskDelete(nullptr);
    }
}