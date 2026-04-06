#include <pyro_dr16_rc_drv.h>

#include "pyro_module_base.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include <pyro_uart_message.h>
#include <pyro_core_config.h>
#include "pyro_uav_booster.h"

using namespace pyro;

constexpr uint32_t EVENT_BIT_FRIC_ENABLE    = (1 << 0);
constexpr uint32_t EVENT_BIT_FRIC_DISABLE   = (1 << 1);
constexpr uint32_t EVENT_BIT_TRIG_ENABLE    = (1 << 2);
constexpr uint32_t EVENT_BIT_TRIG_DISABLE   = (1 << 3);

static TaskHandle_t booster_task_handle        = nullptr;
static uav_booster_t *uav_booster_ptr         = nullptr;
static uav_booster_cmd_t *uav_booster_cmd_ptr = nullptr;

extern OperateBytes operate_bytes;

extern "C"
{
void booster_dr162cmd(uint32_t notify_val)
{
    read_scope_lock lock(dr16_drv_t::get_lock());
    const auto &vrc = rc_drv_t::read();

    if (sw_pos_t::UP == vrc.switches.right.current_pos)
    {
        uav_booster_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;

        uav_booster_cmd_ptr->trigger_enable = false;
        uav_booster_cmd_ptr->single_mode = false;
        uav_booster_cmd_ptr->continue_mode = false;
        uav_booster_cmd_ptr->target_fric1_mps   = 0.0f;
        uav_booster_cmd_ptr->target_fric2_mps  = 0.0f;
        return;
    }

    if(sw_pos_t::DOWN == vrc.switches.right.current_pos)
    {
        uav_booster_cmd_ptr->mode      = cmd_base_t::mode_t::ACTIVE;
    }

    if (notify_val & EVENT_BIT_FRIC_ENABLE)
    {
        uav_booster_cmd_ptr->target_fric1_mps = -16.0f;
        uav_booster_cmd_ptr->target_fric2_mps = 16.0f;
    }
    if (notify_val & EVENT_BIT_FRIC_DISABLE)
    {
        uav_booster_cmd_ptr->target_fric1_mps = 0.0f;
        uav_booster_cmd_ptr->target_fric2_mps = 0.0f;
    }

    // uav_booster_cmd_ptr->booster_auto_flag = operate_bytes.output_data.fire;
    if (notify_val & EVENT_BIT_TRIG_ENABLE)
    {
        uav_booster_cmd_ptr->trigger_enable = true;
        uav_booster_cmd_ptr->single_mode = true;
    }
    if (notify_val & EVENT_BIT_TRIG_DISABLE)
    {
        uav_booster_cmd_ptr->trigger_enable = false;
        uav_booster_cmd_ptr->single_mode = false;
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
                booster_dr162cmd(notify_val);
            }
            if (vt03_drv_t::instance().check_online())
            {
                return;
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
                    configMAX_PRIORITIES - 1, &booster_task_handle);

        sw_broker::subscribe(&vrc.switches.left, sw_event_t::MID_TO_UP, booster_task_handle, EVENT_BIT_FRIC_ENABLE);
        sw_broker::subscribe(&vrc.switches.left, sw_event_t::UP_TO_MID, booster_task_handle, EVENT_BIT_FRIC_DISABLE);
        sw_broker::subscribe(&vrc.switches.left, sw_event_t::MID_TO_DOWN, booster_task_handle, EVENT_BIT_TRIG_ENABLE);
        sw_broker::subscribe(&vrc.switches.left, sw_event_t::DOWN_TO_MID, booster_task_handle, EVENT_BIT_TRIG_DISABLE);

        vTaskDelete(nullptr);
    }
}