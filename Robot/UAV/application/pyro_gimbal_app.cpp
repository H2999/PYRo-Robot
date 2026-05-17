// #include <../autoaim/pyro_uart_message.h>

#include "../autoaim/pyro_autoaim_drv.h"
#include "pyro_uav_gimbal.h"

#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_com_cantx.h"

using namespace pyro;

uav_gimbal_t *gimbal_ptr               = nullptr;
uav_gimbal_cmd_t *gimbal_cmd_ptr       = nullptr;
extern autoaim_drv_t::rx_data_t rx_data;

static uint32_t KEY_CTRL                = (1 << 0);
static uint32_t KEY_SHIFT               = (1 << 1);
static uint32_t KEY_S                   = (1 << 2);

static TaskHandle_t gimbal_task_handle       = nullptr;
static constexpr float rc_sensitivity = 0.0025f;
//位控相当于对速度进行积分 所以是target+=current * delta t 这里直接把灵敏度和delta乘到一起了写成了 rc_sensitivity

extern "C"
{
void gimbal_dr16cmd(uint32_t notify_val)
{
    read_scope_lock lock(dr16_drv_t::get_lock());
    const auto &vrc = rc_drv_t::read();

    //右侧拨码朝上进入无力状态
    if (sw_pos_t::UP == vrc.switches.right.current_pos)
    {
        gimbal_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->yaw_delta_angle     = 0;
        gimbal_cmd_ptr->pitch_delta_angle   = 0;
        return;
    }
    gimbal_cmd_ptr->mode = cmd_base_t::mode_t::ACTIVE;
    //右侧拨码在下进入自瞄状态 自瞄状态下如果识别到目标交给自瞄控制 没识别到目标交给遥控器控制
    if (sw_pos_t::DOWN == vrc.switches.right.current_pos)
    {
        gimbal_cmd_ptr->auto_flag = true;

        gimbal_cmd_ptr->yaw_target_angle = rx_data.shoot_yaw;
        gimbal_cmd_ptr->pitch_target_angle = rx_data.shoot_pitch;
        //这里加负号是因为遥控器映射的是角度 与目标角度作闭环的是从imu中获取的角度
        //但是由于imu安装位置与正常的前x左y上z都相反 所以遥控器发相反的信号才能保持坐标系和imu一致
        gimbal_cmd_ptr->yaw_delta_angle   = - vrc.axes.rx * rc_sensitivity;
        gimbal_cmd_ptr->pitch_delta_angle = - vrc.axes.ry * rc_sensitivity;
    }
    //右侧拨码在中间由遥控器控制
    if (sw_pos_t::MID == vrc.switches.right.current_pos)
    {
        gimbal_cmd_ptr->auto_flag = false;

        gimbal_cmd_ptr->yaw_delta_angle   = - vrc.axes.rx * rc_sensitivity;
        gimbal_cmd_ptr->pitch_delta_angle = - vrc.axes.ry * rc_sensitivity;
    }
}

void gimbalvt03cmd(uint32_t notify_val)
{
    read_scope_lock lock(vt03_drv_t::get_lock());
    const auto &vrc = rc_drv_t::read();

    if (sw_pos_t::UP == vrc.switches.gear.current_pos)
    {
        gimbal_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->pitch_delta_angle = 0;
        gimbal_cmd_ptr->yaw_delta_angle   = 0;
        return;
    }

    gimbal_cmd_ptr->mode = cmd_base_t::mode_t::ACTIVE;

    if (sw_pos_t::DOWN == vrc.switches.gear.current_pos)
    {
        gimbal_cmd_ptr->auto_flag = true;

        gimbal_cmd_ptr->yaw_target_angle = rx_data.shoot_yaw;
        gimbal_cmd_ptr->pitch_target_angle = rx_data.shoot_pitch;

        gimbal_cmd_ptr->pitch_delta_angle =
            -vrc.axes.ry * 0.0015f - vrc.mouse_axes.y * 0.1f;
        gimbal_cmd_ptr->yaw_delta_angle =
            -vrc.axes.rx * 0.0015f - vrc.mouse_axes.x * 0.12f;
    }

    if (sw_pos_t::MID == vrc.switches.gear.current_pos)
    {
        gimbal_cmd_ptr->auto_flag = false;

        gimbal_cmd_ptr->pitch_delta_angle =
            -vrc.axes.ry * 0.0015f - vrc.mouse_axes.y * 0.04f;
        gimbal_cmd_ptr->yaw_delta_angle =
            -vrc.axes.rx * 0.0015f - vrc.mouse_axes.x * 0.12f;
    }
    }


void uav_gimbal_main_thread(void *argument)
{
    while (true)
    {
        uint32_t notify_val = 0;
        xTaskNotifyWait(0x00, 0xFFFFFFFF, &notify_val, 0);

        if (dr16_drv_t::instance().check_online())
        {
            gimbal_dr16cmd(notify_val);
        }
        else if (vt03_drv_t::instance().check_online())
        {
            gimbalvt03cmd(notify_val);
        }

        gimbal_ptr->set_command(*gimbal_cmd_ptr);
        vTaskDelay(1);
    }
}

void uav_gimbal_init(void *argument)
{
    gimbal_cmd_ptr     = new uav_gimbal_cmd_t();
    gimbal_ptr = uav_gimbal_t::instance();
    auto &vrc =rc_drv_t::read();
    gimbal_ptr->start();

    xTaskCreate(uav_gimbal_main_thread, "uav_gimbal_main_thread", 256, nullptr,
                configMAX_PRIORITIES - 3, &gimbal_task_handle);

    btn_broker::subscribe(&vrc.keys.ctrl, btn_event_t::PRESS_DOWN, gimbal_task_handle, KEY_CTRL);
    btn_broker::subscribe(&vrc.keys.shift, btn_event_t::PRESS_DOWN, gimbal_task_handle, KEY_SHIFT);
    btn_broker::subscribe(&vrc.keys.s, btn_event_t::PRESS_DOWN, gimbal_task_handle, KEY_S);

    vTaskDelete(nullptr);
}
}