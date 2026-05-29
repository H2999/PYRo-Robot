#include "pyro_dwt_drv.h"
#include "../autoaim/pyro_autoaim_drv.h"
#include "pyro_uav_gimbal.h"
#include "pyro_uav_booster.h"

using namespace pyro;

extern uav_booster_t *uav_booster_ptr;
extern uav_gimbal_t *gimbal_ptr;

static TaskHandle_t uav_autoaim_app_handle = nullptr;
static autoaim_drv_t *autoaim_drv_ptr = nullptr;

autoaim_drv_t::rx_data_t rx_data{};
autoaim_drv_t::tx_data_t tx_data{};
static uint8_t enemy_color = 0;

void update_and_send_feedback()
{
    if (gimbal_ptr == nullptr) return;
    autoaim_drv_ptr->get_tx_data() = tx_data;

    //yaw pitch roll
    float angle[3];
    gimbal_ptr->gimbal_ins->get_rads_n(angle,(angle+1),(angle+2));

    // 根据原逻辑装载发送参数
    tx_data.curr_yaw    = angle[0];
    tx_data.curr_pitch  = - angle[1];
    tx_data.curr_roll   = angle[2];
    tx_data.curr_speed  = uav_booster_ptr->get_data()->shoot_data.now_bullet_speed_mps;

    tx_data.shoot_delay = 0;
    tx_data.autoaim     = 1;
    const uint8_t robot_id = uav_booster_ptr->get_robot_id();
    if (robot_id > 100)
    {
        enemy_color = 1;
    }

    tx_data.enemy_color = enemy_color;

    autoaim_drv_ptr->send_data();
}

extern "C"
{
    void uav_autoaim_app_thread(void *argument)
    {
        vTaskDelay(500);

        while (true)
        {
            if (autoaim_drv_ptr->check_online())
            {
                rx_data = autoaim_drv_ptr->get_target_data();
            }
            // 发送云台状态回传给 PC
            update_and_send_feedback();

            vTaskDelay(1);
        }
    }

    void uav_autoaim_app_init(void *argument)
    {
        autoaim_drv_ptr = &autoaim_drv_t::get_instance();

        autoaim_drv_ptr->start_rx();

        xTaskCreate(uav_autoaim_app_thread, "uav_pc_com_main_thread", 512,
                    nullptr, configMAX_PRIORITIES - 3, &uav_autoaim_app_handle);

        vTaskDelete(nullptr);
    }
}
