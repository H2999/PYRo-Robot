#include <pyro_uart_message.h>
#include <pyro_uav_gimbal.h>

#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_com_cantx.h"

using namespace pyro;
uav_gimbal_t *gimbal_ptr                       = nullptr;
uav_gimbal_cmd_t *gimbal_cmd_ptr               = nullptr;

extern OperateBytes operate_bytes;
static constexpr float rc_sensitivity = 0.0025f;

extern "C"
{
void gimbal_dr162cmd()
{
    read_scope_lock lock(dr16_drv_t::get_lock());
    const auto &vrc = rc_drv_t::read();

    //如果右侧拨码拨到上面 就进入无力模式
    if (sw_pos_t::UP == vrc.switches.right.current_pos)
    {
        gimbal_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->yaw_delta_angle     = 0;
        gimbal_cmd_ptr->pitch_delta_angle   = 0;
        gimbal_cmd_ptr->roll_delta_angle    = 0;

        return;
    }

    gimbal_cmd_ptr->mode = cmd_base_t::mode_t::ACTIVE;
    if (sw_pos_t::DOWN == vrc.switches.right.current_pos)
    {
        gimbal_cmd_ptr->auto_flag = true;

        gimbal_cmd_ptr->yaw_target_angle = operate_bytes.output_data.shoot_yaw;
        gimbal_cmd_ptr->pitch_target_angle = operate_bytes.output_data.shoot_pitch;
        gimbal_cmd_ptr->yaw_delta_angle   = - vrc.axes.rx * rc_sensitivity;
        gimbal_cmd_ptr->pitch_delta_angle = - vrc.axes.ry * rc_sensitivity;
        gimbal_cmd_ptr->roll_delta_angle  = - vrc.axes.lx * rc_sensitivity;
    }
    else if (sw_pos_t::MID == vrc.switches.right.current_pos)
    {
        gimbal_cmd_ptr->auto_flag = false;

        gimbal_cmd_ptr->yaw_delta_angle   = - vrc.axes.rx * rc_sensitivity;
        gimbal_cmd_ptr->pitch_delta_angle = - vrc.axes.ry * rc_sensitivity;
        gimbal_cmd_ptr->roll_delta_angle  = - vrc.axes.lx * rc_sensitivity;
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
            gimbal_dr162cmd();
        }

        gimbal_ptr->set_command(*gimbal_cmd_ptr);
        vTaskDelay(1);
    }
}

void uav_gimbal_init(void *argument)
{
    gimbal_cmd_ptr     = new uav_gimbal_cmd_t();
    gimbal_ptr = uav_gimbal_t::instance();
    gimbal_ptr->start();

    xTaskCreate(uav_gimbal_main_thread, "uav_gimbal_main_thread", 256, nullptr,
                configMAX_PRIORITIES - 1, nullptr);

    rc_drv_t::read();

    vTaskDelete(nullptr);
}
}