#include "pyro_module_base.h"
#include "pyro_mec_chassis.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_direct_gimbal.h"
#include "pyro_com_cantx.h"
#include "pyro_uart_comm.h"
#include "struct.h"

using namespace pyro;

// 定义任务通知的位掩码 (Event Bits)
constexpr uint32_t EVENT_BIT_GYRO_TOGGLE = (1 << 0);

static TaskHandle_t gimbal_task_handle                = nullptr;
static pyro::direct_gimbal_t *direct_gimbal_ptr       = nullptr;
static pyro::direct_gimbal_cmd_t *direct_gimbal_cmd_ptr = nullptr;

static void gimbal_dr162cmd();
static void gimbal_vt032cmd();
static void chassis_dr162cmd();
static void chassis_vt032cmd(uint32_t notify_val);

extern StateBytes state_bytes;

extern "C"
{
    void hero_gimbal_thread(void *argument)
    {
        while (true)
        {
            uint32_t notify_val = 0;
            // 非阻塞提取任务通知（超时为0），并在退出时清空所有位
            xTaskNotifyWait(0x00, 0xFFFFFFFF, &notify_val, 0);

            if (vt03_drv_t::instance().check_online())
            {
                chassis_vt032cmd(notify_val);
                gimbal_vt032cmd();
            }
            else if (dr16_drv_t::instance().check_online())
            {
                chassis_dr162cmd();
                gimbal_dr162cmd();
            }
            direct_gimbal_ptr->set_command(*direct_gimbal_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_gimbal_init(void *argument)
    {
        direct_gimbal_cmd_ptr = new pyro::direct_gimbal_cmd_t();
        direct_gimbal_ptr     = pyro::direct_gimbal_t::instance();

        direct_gimbal_ptr->start();

        // 1. 创建任务并获取句柄
        xTaskCreate(hero_gimbal_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, &gimbal_task_handle);

        // 2. 将事件订阅到泛型 Broker 中
        auto &vrc = pyro::rc_drv_t::read();
        pyro::btn_broker::subscribe(&vrc.keys.shift, pyro::btn_event_t::PRESS_DOWN, gimbal_task_handle, EVENT_BIT_GYRO_TOGGLE);

        vTaskDelete(nullptr);
    }
}

void gimbal_dr162cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (pyro::sw_pos_t::DOWN == vrc.switches.right.current_pos)
    {
        direct_gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::PASSIVE;
        direct_gimbal_cmd_ptr->pitch_delta_angle = 0;
        direct_gimbal_cmd_ptr->yaw_delta_angle   = 0;
        return;
    }

    direct_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;

    if (pyro::sw_pos_t::UP == vrc.switches.right.current_pos)
    {
        direct_gimbal_cmd_ptr->auto_aim     = true;
        direct_gimbal_cmd_ptr->target_pitch = state_bytes.input_data.shoot_pitch;
        direct_gimbal_cmd_ptr->target_yaw   = state_bytes.input_data.shoot_yaw;
    }
    else
    {
        direct_gimbal_cmd_ptr->auto_aim          = false;
        direct_gimbal_cmd_ptr->pitch_delta_angle = -vrc.axes.ry * 0.002f;
        direct_gimbal_cmd_ptr->yaw_delta_angle   = -vrc.axes.rx * 0.0035f;
    }
}

void gimbal_vt032cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (pyro::sw_pos_t::UP == vrc.switches.gear.current_pos) // GEAR_LEFT
    {
        direct_gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::PASSIVE;
        direct_gimbal_cmd_ptr->pitch_delta_angle = 0;
        direct_gimbal_cmd_ptr->yaw_delta_angle   = 0;
        return;
    }

    direct_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;

    if (pyro::sw_pos_t::DOWN == vrc.switches.gear.current_pos) // GEAR_RIGHT
    {
        direct_gimbal_cmd_ptr->auto_aim = true;
    }
    else
    {
        direct_gimbal_cmd_ptr->auto_aim = false;
    }

    if (direct_gimbal_cmd_ptr->auto_aim)
    {
        direct_gimbal_cmd_ptr->target_pitch = state_bytes.input_data.shoot_pitch;
        direct_gimbal_cmd_ptr->target_yaw   = state_bytes.input_data.shoot_yaw;
    }
    else
    {
        direct_gimbal_cmd_ptr->pitch_delta_angle =
            -vrc.axes.ry * 0.0035f - vrc.mouse_axes.y * 0.25f;
        direct_gimbal_cmd_ptr->yaw_delta_angle =
            -vrc.axes.rx * 0.0035f - vrc.mouse_axes.x * 0.6f;
    }
}

void chassis_dr162cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    static int8_t vx      = 0;
    static int8_t vy      = 0;
    static int8_t wz      = 0;
    static uint8_t active = 0;

    pyro::can_tx_drv_t::clear(0x101);

    if (pyro::sw_pos_t::MID != vrc.switches.right.current_pos)
    {
        vx     = 0;
        vy     = 0;
        wz     = 0;
        active = 0;
        pyro::can_tx_drv_t::add_data(0x101, 8, vx);
        pyro::can_tx_drv_t::add_data(0x101, 8, vy);
        pyro::can_tx_drv_t::add_data(0x101, 8, wz);
        pyro::can_tx_drv_t::add_data(0x101, 1, active);
        pyro::can_tx_drv_t::send(
            0x101, pyro::can_hub_t::get_instance()->hub_get_can_obj(
                       pyro::can_hub_t::which_can::can3));
        return;
    }

    vx     = static_cast<int8_t>(vrc.axes.ly * 127);
    vy     = static_cast<int8_t>(-vrc.axes.lx * 127);
    wz     = 0;
    active = 1;

    pyro::can_tx_drv_t::add_data(0x101, 8, vx);
    pyro::can_tx_drv_t::add_data(0x101, 8, vy);
    pyro::can_tx_drv_t::add_data(0x101, 8, wz);
    pyro::can_tx_drv_t::add_data(0x101, 1, active);
    pyro::can_tx_drv_t::send(0x101,
                             pyro::can_hub_t::get_instance()->hub_get_can_obj(
                                 pyro::can_hub_t::which_can::can3));
}

void chassis_vt032cmd(uint32_t notify_val)
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    static int8_t vx      = 0;
    static int8_t vy      = 0;
    static int8_t wz      = 0;
    static uint8_t active = 0;

    pyro::can_tx_drv_t::clear(0x101);

    if (pyro::sw_pos_t::MID != vrc.switches.gear.current_pos) // 仅在中档为 Active
    {
        vx     = 0;
        vy     = 0;
        wz     = 0;
        active = 0;
        pyro::can_tx_drv_t::add_data(0x101, 8, vx);
        pyro::can_tx_drv_t::add_data(0x101, 8, vy);
        pyro::can_tx_drv_t::add_data(0x101, 8, wz);
        pyro::can_tx_drv_t::add_data(0x101, 1, active);
        pyro::can_tx_drv_t::send(
            0x101, pyro::can_hub_t::get_instance()->hub_get_can_obj(
                       pyro::can_hub_t::which_can::can3));
        return;
    }

    vx     = static_cast<int8_t>(vrc.keys.w.current_level   ? 127
                                 : vrc.keys.s.current_level ? -127
                                                            : vrc.axes.lx * 127);
    vy     = static_cast<int8_t>(vrc.keys.a.current_level   ? 127
                                 : vrc.keys.d.current_level ? -127
                                                            : -vrc.axes.ly * 127);

    static bool gyroscope_en = false;
    if (notify_val & EVENT_BIT_GYRO_TOGGLE)
    {
        gyroscope_en = !gyroscope_en;
    }

    wz = static_cast<int8_t>(vrc.axes.wheel * 127);
    if (gyroscope_en)
    {
        wz = 127;
    }
    active = 1;

    pyro::can_tx_drv_t::add_data(0x101, 8, vx);
    pyro::can_tx_drv_t::add_data(0x101, 8, vy);
    pyro::can_tx_drv_t::add_data(0x101, 8, wz);
    pyro::can_tx_drv_t::add_data(0x101, 1, active);
    pyro::can_tx_drv_t::send(0x101,
                             pyro::can_hub_t::get_instance()->hub_get_can_obj(
                                 pyro::can_hub_t::which_can::can3));
}