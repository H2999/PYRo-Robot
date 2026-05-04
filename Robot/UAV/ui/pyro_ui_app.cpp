// #include "pyro_core_def.h"
// #include "pyro_uav_gimbal.h"
//
// ReSharper disable CppExpressionWithoutSideEffects
// using namespace pyro;
// #include "pyro_com_canrx.h"
// #include "pyro_module_base.h"
// #include "pyro_referee.h"
// #include "pyro_ui_drv.h"
// #include "pyro_uav_booster.h"
// #include "pyro_uart_message.h"
//
// using namespace pyro;
//
// static referee_drv_t *referee_ptr                         = nullptr;
// static ui_drv_t *ui_ptr                                   = nullptr;
//
// static float fric1_mps                                    = 0.0f;
// static float fric2_mps                                    = 0.0f;
// static bool fric_en                                       = false;
// static bool flush_flag                                    = false;
// static bool speed_control_en                              = false;
// static bool fric1_online                                  = false;
// static bool fric2_online                                  = false;
//
// /**
//  * @brief 静态 UI 绘制（仅在初始化或手动刷新时调用，使用 ADD）
//  */
// void ui_draw_static()
// {
//     // ui_ptr
//     //     ->draw_circle("C01", pyro::ui_operate::ADD, 1, pyro::ui_color::GREEN,
//     //     2,
//     //                   960, 540, 100)
//     // 准星竖线
//     //因为返回的是this指针 所以可以递归调用
//     ui_ptr->draw_line("L01", ui_operate::ADD, 1, ui_color::GREEN, 4,
//                     860, 615, 1060, 615)
//         .draw_line("L02", ui_operate::ADD, 1, ui_color::GREEN, 4, 860,
//                    585, 1060, 585);
//     //俩个拼在一起发送
//     ui_ptr->flush();
//     vTaskDelay(pdMS_TO_TICKS(50));
//
//     // 1. 绘制静态文本标签 (注意名字不能重复)
//     ui_ptr->draw_string("ST1", ui_operate::ADD, 3,ui_color::BLACK,
//                         20, 2, 110, 800, "FRIC1:");
//     vTaskDelay(pdMS_TO_TICKS(50));
//     ui_ptr->draw_string("ST2", ui_operate::ADD, 3, ui_color::BLACK,
//                         20, 2, 110, 750, "FRIC2:");
//     vTaskDelay(pdMS_TO_TICKS(50));
//
//     // 2. 为动态数值提前进行 ADD 占位，赋予初始值，方便后续直接 MODIFY
//     ui_ptr->draw_float("DF1", ui_operate::ADD, 4, ui_color::WHITE, 20,
//                      2, 230, 800, 0.0f)
//         .draw_float("DF2", ui_operate::ADD, 4, ui_color::WHITE, 20,
//                     2, 230, 750, 0.0f);
//
//     //两个包拼在一起发送
//     ui_ptr->flush();
//     vTaskDelay(pdMS_TO_TICKS(50));
// }
//
// /**
//  * @brief 动态 UI 更新（定时刷新，必须使用 MODIFY）
//  */
// void ui_update_dynamic()
// {
//     // 修改摩擦轮速度值 MODIFY
//     if (fric_en)
//     {
//         if (fric1_online)
//         {
//             if (speed_control_en)
//             {
//                 ui_ptr->draw_float("DF1", ui_operate::MODIFY, 4,
//                                    ui_color::YELLOW, 20, 2, 230, 800,
//                                    fric1_mps);
//             }
//             else
//             {
//                 ui_ptr->draw_float("DF1", ui_operate::MODIFY, 4,
//                                    ui_color::WHITE, 20, 2, 230, 800,
//                                    fric1_mps);
//             }
//         }
//         else
//         {
//             ui_ptr->draw_float("DF1", ui_operate::MODIFY, 4,
//                                ui_color::BLACK, 20, 2, 230, 800, 0.0f);
//         }
//         if (fric2_online)
//         {
//             if (speed_control_en)
//             {
//                 ui_ptr->draw_float("DF2", ui_operate::MODIFY, 4,
//                                    ui_color::YELLOW, 20, 2, 230, 750,
//                                    fric2_mps);
//             }
//             else
//             {
//                 ui_ptr->draw_float("DF2", ui_operate::MODIFY, 4,
//                                    ui_color::WHITE, 20, 2, 230, 750,
//                                    fric2_mps);
//             }
//         }
//         else
//         {
//             ui_ptr->draw_float("DF2", ui_operate::MODIFY, 4,
//                                ui_color::BLACK, 20, 2, 230, 750, 0.0f);
//         }
//     }
//     else
//     {
//         ui_ptr->draw_float("DF1", ui_operate::MODIFY, 4,
//                                ui_color::WHITE, 20, 2, 230, 800, 0.0f);
//         ui_ptr->draw_float("DF2", ui_operate::MODIFY, 4,
//                                ui_color::WHITE, 20, 2, 230, 750, 0.0f);
//     }
//     ui_ptr->flush();
// }
//
// extern "C"
// {
//     void uav_ui_thread(void *argument)
//     {
//         // 1. 阻塞等待裁判系统链路连通，并确保获取到了真实机器人ID
//         while (!referee_ptr->is_online() || referee_ptr->get_robot_id() == 0)
//         {
//             vTaskDelay(pdMS_TO_TICKS(500));
//         }
//
//         // 2. 初始清理操作，并绘制静态结构
//         ui_ptr->clear_all();
//         vTaskDelay(pdMS_TO_TICKS(200));
//         ui_draw_static();
//         vTaskDelay(pdMS_TO_TICKS(100));
//
//         while (true)
//         {
//             if (referee_ptr->is_online())
//             {
//                 if (flush_flag)
//                 {
//                     ui_ptr->clear_all();
//                     vTaskDelay(pdMS_TO_TICKS(200));
//                     ui_draw_static();
//                     vTaskDelay(pdMS_TO_TICKS(100));
//                     flush_flag = false;
//                 }
//                 else
//                 {
//                     ui_update_dynamic();
//                 }
//             }
//             vTaskDelay(pdMS_TO_TICKS(50));
//         }
//     }
//
//     void uav_ui_init(void *argument)
//     {
//         referee_ptr = referee_drv_t::get_instance();
//         ui_ptr      = new ui_drv_t(referee_ptr);
//
//         xTaskCreate(uav_ui_thread, "uav_ui_thread", 512, nullptr,
//                     configMAX_PRIORITIES - 3, nullptr);
//
//         vTaskDelete(nullptr);
//     }
// }

#include "pyro_ui_drv.h"
#include "pyro_uav_booster.h"
#include "pyro_referee.h"
#include "pyro_module_base.h"
#include <FreeRTOS.h>
#include <task.h>

using namespace pyro;

static referee_drv_t *referee_ptr = nullptr;
static ui_drv_t *ui_ptr          = nullptr;

static float fric1_mps           = 0.0f;
static float fric2_mps           = 0.0f;
static bool fric_en              = false;
static bool flush_flag           = false;
static bool speed_control_en     = false;
static bool fric1_online         = false;
static bool fric2_online         = false;

/**
 * @brief 绘制静态装饰和标签
 * 修正：取消链式调用，逐行调用以适配返回 bool 的接口
 */
void ui_draw_static()
{
    // 1. 准星参考线
    ui_ptr->draw_line("L01", ui_operate::ADD, 1, ui_color::GREEN, 2, 860, 615, 1060, 615);
    ui_ptr->draw_line("L02", ui_operate::ADD, 1, ui_color::GREEN, 2, 860, 585, 1060, 585);
    ui_ptr->flush();
    vTaskDelay(pdMS_TO_TICKS(50));

    // 2. 静态文字
    ui_ptr->draw_string("ST1", ui_operate::ADD, 3, ui_color::CYAN, 20, 2, 110, 800, "FRIC1:");
    ui_ptr->draw_string("ST2", ui_operate::ADD, 3, ui_color::CYAN, 20, 2, 110, 750, "FRIC2:");
    ui_ptr->flush();
    vTaskDelay(pdMS_TO_TICKS(50));

    // 3. 动态数值占位
    ui_ptr->draw_float("DF1", ui_operate::ADD, 4, ui_color::WHITE, 20, 2, 230, 800, 0.0f);
    ui_ptr->draw_float("DF2", ui_operate::ADD, 4, ui_color::WHITE, 20, 2, 230, 750, 0.0f);
    ui_ptr->flush();
    vTaskDelay(pdMS_TO_TICKS(50));
}

/**
 * @brief 动态更新数值和颜色
 */
void ui_update_dynamic()
{
    ui_color color1 = ui_color::WHITE;
    float val1 = 0.0f;

    if (!fric1_online) {
        color1 = ui_color::BLACK;
    } else if (!fric_en) {
        color1 = ui_color::WHITE;
    } else {
        val1 = fric1_mps;
        color1 = speed_control_en ? ui_color::YELLOW : ui_color::GREEN;
    }

    ui_color color2 = ui_color::WHITE;
    float val2 = 0.0f;

    if (!fric2_online) {
        color2 = ui_color::BLACK;
    } else if (!fric_en) {
        color2 = ui_color::WHITE;
    } else {
        val2 = fric2_mps;
        color2 = speed_control_en ? ui_color::YELLOW : ui_color::GREEN;
    }

    // 修正：分开调用
    ui_ptr->draw_float("DF1", ui_operate::MODIFY, 4, color1, 20, 2, 230, 800, val1);
    ui_ptr->draw_float("DF2", ui_operate::MODIFY, 4, color2, 20, 2, 230, 750, val2);

    ui_ptr->flush();
}

extern "C"
{
    void uav_ui_thread(void *argument)
    {
        uav_booster_t* booster_ptr = uav_booster_t::instance();

        while (!referee_ptr->is_online() || referee_ptr->get_robot_id() == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        ui_ptr->clear_all();
        vTaskDelay(pdMS_TO_TICKS(200));
        ui_draw_static();

        while (true)
        {
            if (referee_ptr->is_online())
            {
                if (booster_ptr != nullptr)
                {
                    auto* ctx = booster_ptr->get_data();

                    fric_en = ctx->cmd->fric_enable;
                    fric1_mps = ctx->data_ctx.current_fric_mps[0];
                    fric2_mps = ctx->data_ctx.current_fric_mps[1];
                    speed_control_en = (ctx->data_ctx.target_fric_mps[0] > 0.1f);

                    // 修正：如果基类没有 check_online，通常使用电机的 get_online 接口
                    // 或者查看 pyro_dji_motor_drv.h 里的具体成员名
                    fric1_online = ctx->cfg.motor_cfg.fric_wheel[0]->is_online();
                    fric2_online = ctx->cfg.motor_cfg.fric_wheel[1]->is_online();
                }

                if (flush_flag)
                {
                    ui_ptr->clear_all();
                    vTaskDelay(pdMS_TO_TICKS(200));
                    ui_draw_static();
                    flush_flag = false;
                }
                else
                {
                    ui_update_dynamic();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    void uav_ui_init(void *argument)
    {
        referee_ptr = referee_drv_t::get_instance();
        ui_ptr      = new ui_drv_t(referee_ptr);

        xTaskCreate(uav_ui_thread, "uav_ui_thread", 512, nullptr,
                    configMAX_PRIORITIES - 3, nullptr);

        vTaskDelete(nullptr);
    }
}