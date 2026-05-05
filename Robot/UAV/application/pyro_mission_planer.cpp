#include "pyro_core_def.h"
#include "pyro_core_config.h"
#include "FreeRTOS.h"
#include "task.h"
extern "C"
{
    extern void pyro_init_thread(void *argument);
    extern void start_debug_task(void *arg);

    extern void uav_gimbal_init(void *argument);
    extern void uav_booster_init(void *argument);
    extern void uav_ui_init(void *argument);
    extern void uav_pc_com_init(void *argument);
    extern void uav_autoaim_app_init(void *argument);


    void start_mission_planer_task(void const *argument)
    {
        xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
            configMAX_PRIORITIES - 1, nullptr);

        xTaskCreate(uav_gimbal_init, "pyro_gimbal_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
        xTaskCreate(uav_booster_init, "pyro_booster_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
        xTaskCreate(uav_autoaim_app_init, "uav_autoaim_app_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
        xTaskCreate(uav_ui_init,"uav_ui_init",512,nullptr,
                    configMAX_PRIORITIES - 3, nullptr);
        xTaskCreate(start_debug_task,"start_debug_task", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
        vTaskDelete(nullptr);
    }
}