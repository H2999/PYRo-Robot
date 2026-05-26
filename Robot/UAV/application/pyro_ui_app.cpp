// #include "pyro_com_canrx.h"
// #include "pyro_module_base.h"
// #include "pyro_rc_base_drv.h"
// #include "pyro_referee.h"
// #include "pyro_ui_drv.h"
// #include "pyro_uav_booster.h"
// #include "pyro_uav_gimbal.h"
// #include "pyro_vt03_rc_drv.h"
// #include "led.h"
// #include <cmath>
//
// #include "pyro_autoaim_drv.h"
//
// using namespace pyro;
//
// // ReSharper disable CppExpressionWithoutSideEffects
// // 1. 全局指针与状态上下文定义
// static referee_drv_t *referee_ptr = nullptr;
// static ui_drv_t *ui_ptr           = nullptr;
// extern uav_booster_t *uav_booster_ptr;
// extern uav_gimbal_t *gimbal_ptr;
// extern autoaim_drv_t::rx_data_t rx_data;
//
//
// static TaskHandle_t ui_task_handle = nullptr;
//
// // 构建强类型的数据上下文
// struct ui_context_t {
//     float yaw = 0.0f;
//     float pitch = 0.0f;
//     float distance = 0.0f;
//     float allow_bullet = 0.0f;
//     float heat_max = 0.0f;
//     float heat_res = 0.0f;
//     bool fric_enable = false;
//     bool trigger_enable = false;
//
//     bool key_w_pressed = false;
//     bool key_a_pressed = false;
//     bool key_s_pressed = false;
//     bool key_d_pressed = false;
//
//     bool refresh_request = false; // 手动强制刷新标志
// };
//
// //通过当前与上回对比 不同才动态刷新ui
// static ui_context_t g_ctx;           // 当前最新数据缓冲区
// static ui_context_t g_last_ctx;      // 上一次渲染的数据缓冲区
// static bool g_force_refresh = false; // 全局静态图层重构标记
//
// // 按键控制掩码
// static uint32_t KEY_CTRL                           = (1 << 0);
// static uint32_t KEY_SHIFT                          = (1 << 1);
// static uint32_t KEY_Z                              = (1 << 2);
// static uint32_t KEY_W_ON                           = (1 << 3);
// static uint32_t KEY_A_ON                           = (1 << 4);
// static uint32_t KEY_S_ON                           = (1 << 5);
// static uint32_t KEY_D_ON                           = (1 << 6);
// static uint32_t KEY_W_OFF                          = (1 << 7);
// static uint32_t KEY_A_OFF                          = (1 << 8);
// static uint32_t KEY_S_OFF                          = (1 << 9);
// static uint32_t KEY_D_OFF                          = (1 << 10);
//
// // 2. 弹道插值配置表 用来画无自瞄时候的瞄准基准
// struct BallisticPoint {
//     float distance;
//     int32_t bias_x;
//     int32_t bias_y;
// };
//
// //如果发现无自瞄时打静止靶准星要往左上偏 那就把叉画在右下 这样把叉对准目标就相当于把中心移到左上了
// static constexpr BallisticPoint ballistic_table[] = {
//     {8.0f,  20, -30},
//     {10.0f, 25, -45},
//     {12.0f, 30, -50},
//     {13.0f, 35, -60},
//     {14.0f, 40, -65},
//     {15.0f, 45, -70},
//     {16.0f, 40, -70}
// };
// static constexpr size_t TABLE_SIZE = sizeof(ballistic_table) / sizeof(BallisticPoint);
//
// // 3. UI 静态图层绘制 (仅在初始化或重置时触发一次)
// void ui_draw_static()
// {
//     if (!ui_ptr) return;
//
//     // 1. 基础几何背景 (处于最底层)
//     ui_ptr->draw_rect("R01", ui_operate::ADD, 1, ui_color::GREEN, 3, 940, 520, 980, 560)//自瞄框
//            .draw_circle("SF1", ui_operate::ADD, 1, ui_color::YELLOW, 4, 1700, 750, 30)      //摩擦轮开启指示
//            .draw_circle("SF2", ui_operate::ADD, 1, ui_color::YELLOW, 4, 1580, 750, 30)
//            .draw_circle("ST1", ui_operate::ADD, 1, ui_color::MAGENTA, 4, 1820, 750, 30);    //波弹盘开启指示
//     ui_ptr->flush();
//
//     // 2. 静态文本标签 (压在几何背景上方，防止被盖住)
//     ui_ptr->draw_string("LB0", pyro::ui_operate::ADD, 2, pyro::ui_color::PINK,
//                                         20, 4, 300, 700, " YAW :");
//     ui_ptr->draw_string("LB1", pyro::ui_operate::ADD, 2, pyro::ui_color::PINK,
//                                         20, 4, 300, 620, "PITCH:");
//     ui_ptr->draw_string("LB2", pyro::ui_operate::ADD, 2, pyro::ui_color::GREEN,
//                                         20, 4, 1540, 620, " DIS :");
//     ui_ptr->draw_string("LB3", pyro::ui_operate::ADD, 2, pyro::ui_color::GREEN,
//                                         20, 4, 1540, 540, "BULLET:");
//     ui_ptr->draw_string("LB4", pyro::ui_operate::ADD, 2, pyro::ui_color::GREEN,
//                                         20, 4, 750, 315, "HEAT:");
//     ui_ptr->draw_string("LB5", pyro::ui_operate::ADD, 2, pyro::ui_color::YELLOW,
//                                         20, 4, 980, 315, "RES:");
//     ui_ptr->flush();
//     //操控状态/按键静态占位
//     ui_ptr->draw_string("KW_", pyro::ui_operate::ADD, 3, pyro::ui_color::WHITE,
//                                         30, 5, 960, 940, "W");
//     ui_ptr->draw_string("KA_", pyro::ui_operate::ADD, 3, pyro::ui_color::WHITE,
//                                         30, 5, 910, 840, "A");
//     ui_ptr->draw_string("KS_", pyro::ui_operate::ADD, 3, pyro::ui_color::WHITE,
//                                         30, 5, 960, 740, "S");
//     ui_ptr->draw_string("KD_", pyro::ui_operate::ADD, 3, pyro::ui_color::WHITE,
//                                         30, 5, 1010, 840, "D");
//     ui_ptr->draw_circle("F3", ui_operate::ADD, 3, ui_color::WHITE, 1, 1700, 750, 10) //开启的话空心圆被填满
//            .draw_circle("F4", ui_operate::ADD, 3, ui_color::WHITE, 1, 1580, 750, 10)
//            .draw_circle("T1", ui_operate::ADD, 3, ui_color::WHITE, 1, 1820, 750, 10);
//     ui_ptr->flush();
//
//     // 3. 高频动态数据数值 ADD 占位
//     ui_ptr->draw_float("DF3", pyro::ui_operate::ADD, 4, pyro::ui_color::WHITE, 20, 2, 510, 700, 0.0f) // Yaw值
//           .draw_float("DF4", pyro::ui_operate::ADD, 4, pyro::ui_color::WHITE, 20, 2, 510, 620, 0.0f) // Pitch值
//           .draw_float("HTM", pyro::ui_operate::ADD, 4, pyro::ui_color::ALLY,  20, 3, 850, 315, g_ctx.heat_max)
//           .draw_float("HTR", pyro::ui_operate::ADD, 4, pyro::ui_color::GREEN, 20, 3, 1065, 315, 0.0f)
//           .draw_float("DIS", pyro::ui_operate::ADD, 4, pyro::ui_color::CYAN,  20, 3, 1670, 620, 0.0f) // 距离
//           .draw_float("BLT", pyro::ui_operate::ADD, 4, pyro::ui_color::PINK,  20, 3, 1670, 540, 0.0f); // 弹量
//
//     // 4. 动态图层状态与交叉准星线占位(高频刷新 放在顶层)
//     ui_ptr->draw_line("XL1",  ui_operate::ADD, 5, ui_color::YELLOW, 5, 948, 552, 972, 548)
//           .draw_line("XL2",  ui_operate::ADD, 5, ui_color::YELLOW, 5, 948, 548, 972, 552);
//     //自瞄框
//     // .draw_circle("auto",ui_operate::ADD,6,ui_color::GREEN,2,960,540,4);
//     ui_ptr->flush();
// }
//
//
// // 4. 增量动态刷新业务逻辑
// void update_gimbal_ui()
// {
//     // 云台角度变化死区控制：超出 0.1 度或强制刷新才产生 放在第四层防止被覆盖
//     if (std::abs(g_ctx.yaw - g_last_ctx.yaw) > 0.1f || g_force_refresh)
//     {
//         ui_ptr->draw_float("DF3", ui_operate::MODIFY, 4, ui_color::ALLY, 20, 4, 510, 700, g_ctx.yaw);
//     }
//     if (std::abs(g_ctx.pitch - g_last_ctx.pitch) > 0.1f || g_force_refresh)
//     {
//         ui_ptr->draw_float("DF4", ui_operate::MODIFY, 4, ui_color::ALLY, 20, 4, 510, 620, g_ctx.pitch);
//     }
//     // ui_ptr->flush();
// }
//
// void update_shoot_ui()
// {
//     // 距离发生显着改变，或者触发强制重绘 放在第四层
//     if (std::abs(g_ctx.distance - g_last_ctx.distance) > 0.05f || g_force_refresh)
//     {
//         ui_ptr->draw_float("DIS", ui_operate::MODIFY, 4, ui_color::CYAN, 20, 3, 1670, 620, g_ctx.distance);
//
//         int32_t center_x = 1920 / 2;
//         int32_t center_y = 1080 / 2;
//         int32_t target_bias_x = 0;
//         int32_t target_bias_y = 0;
//
//         // 线性插值计算
//         if (g_ctx.distance <= ballistic_table[0].distance)
//         {
//             target_bias_x = ballistic_table[0].bias_x;
//             target_bias_y = ballistic_table[0].bias_y;
//         }
//         else if (g_ctx.distance >= ballistic_table[TABLE_SIZE - 1].distance)
//         {
//             target_bias_x = ballistic_table[TABLE_SIZE - 1].bias_x;
//             target_bias_y = ballistic_table[TABLE_SIZE - 1].bias_y;
//         }
//         else
//         {
//             for (size_t i = 0; i < TABLE_SIZE - 1; ++i)
//             {
//                 if (g_ctx.distance >= ballistic_table[i].distance && g_ctx.distance <= ballistic_table[i+1].distance)
//                 {
//                     float t = (g_ctx.distance - ballistic_table[i].distance) / (ballistic_table[i+1].distance - ballistic_table[i].distance);
//                     target_bias_x = ballistic_table[i].bias_x + static_cast<int32_t>(t * static_cast<float>(ballistic_table[i+1].bias_x - ballistic_table[i].bias_x));
//                     target_bias_y = ballistic_table[i].bias_y + static_cast<int32_t>(t * static_cast<float>(ballistic_table[i+1].bias_y - ballistic_table[i].bias_y));
//                     break;
//                 }
//             }
//         }
//
//         center_x += target_bias_x;
//         center_y += target_bias_y;
//         uint16_t cross_size = 12;
//
//         // 核心准星线 放在顶层
//         ui_ptr->draw_line("XL1", ui_operate::MODIFY, 5, ui_color::ORANGE, 3, center_x - cross_size, center_y + cross_size, center_x + cross_size, center_y - cross_size)
//                .draw_line("XL2", ui_operate::MODIFY, 5, ui_color::ORANGE, 3, center_x - cross_size, center_y - cross_size, center_x + cross_size, center_y + cross_size);
//     }
//
//     // ui_ptr->flush();
// }
//
// void update_booster_ui()
// {
//     // 1. 发弹量数值更新
//     if (std::abs(g_ctx.allow_bullet - g_last_ctx.allow_bullet) > 0.5f || g_force_refresh)
//     {
//         ui_ptr->draw_float("BLT", ui_operate::MODIFY, 4, ui_color::CYAN, 20, 3, 1670, 540, g_ctx.allow_bullet);
//     }
//
//     // 2. 状态圆圈按需更新 (只有状态开关切换时发送)
//     if (g_ctx.fric_enable != g_last_ctx.fric_enable || g_ctx.trigger_enable != g_last_ctx.trigger_enable || g_force_refresh)
//     {
//         if (g_ctx.fric_enable)
//         {
//             //线宽25大于半径 所以会变成实现圆点 利用这一点来提示
//             ui_ptr->draw_circle("F3", ui_operate::MODIFY, 3, ui_color::GREEN, 25, 1700, 750, 10)
//                   .draw_circle("F4", ui_operate::MODIFY, 3, ui_color::GREEN, 25, 1580, 750, 10);
//             if (g_ctx.trigger_enable)
//             {
//                 ui_ptr->draw_circle("T1", ui_operate::MODIFY, 3, ui_color::MAGENTA, 25, 1820, 750, 10);
//             }
//         }
//         else
//         {
//             ui_ptr->draw_circle("F3", ui_operate::MODIFY, 3, ui_color::WHITE, 1, 1700, 750, 10)
//                   .draw_circle("F4", ui_operate::MODIFY, 3, ui_color::WHITE, 1, 1580, 750, 10)
//                   .draw_circle("T1", ui_operate::MODIFY, 3, ui_color::WHITE, 1, 1820, 750, 10);
//         }
//     }
//     // ui_ptr->flush();
// }
//
// void update_keyboard_ui()
// {
//     // W 键状态更新
//     if (g_ctx.key_w_pressed != g_last_ctx.key_w_pressed || g_force_refresh)
//     {
//         pyro::ui_color color = g_ctx.key_w_pressed ? pyro::ui_color::GREEN : pyro::ui_color::WHITE;
//         ui_ptr->draw_string("KW_", pyro::ui_operate::MODIFY, 3, color, 30, 5, 960, 940, "W");
//     }
//     // A 键状态更新
//     if (g_ctx.key_a_pressed != g_last_ctx.key_a_pressed || g_force_refresh)
//     {
//         pyro::ui_color color = g_ctx.key_a_pressed ? pyro::ui_color::GREEN : pyro::ui_color::WHITE;
//         ui_ptr->draw_string("KA_", pyro::ui_operate::MODIFY, 3, color, 30, 5, 910, 840, "A");
//     }
//     // S 键状态更新
//     if (g_ctx.key_s_pressed != g_last_ctx.key_s_pressed || g_force_refresh)
//     {
//         pyro::ui_color color = g_ctx.key_s_pressed ? pyro::ui_color::GREEN : pyro::ui_color::WHITE;
//         ui_ptr->draw_string("KS_", pyro::ui_operate::MODIFY, 3, color, 30, 5, 960, 740, "S");
//     }
//     // D 键状态更新
//     if (g_ctx.key_d_pressed != g_last_ctx.key_d_pressed || g_force_refresh)
//     {
//         pyro::ui_color color = g_ctx.key_d_pressed ? pyro::ui_color::GREEN : pyro::ui_color::WHITE;
//         ui_ptr->draw_string("KD_", pyro::ui_operate::MODIFY, 3, color, 30, 5, 1010, 840, "D");
//     }
//
//     // ui_ptr->flush();
// }
//
// // 统一的增量动态绘图调用入口
// void ui_update_dynamic()
// {
//     update_gimbal_ui();
//     update_booster_ui();
//     update_shoot_ui();
//     update_keyboard_ui();
//
//     ui_ptr->flush(); // 统一在最末尾进行图层合并冲刷
//     g_force_refresh = false; // 清除强刷标志
// }
//
// static void info_context_collect()
// {
//     //循环比较
//     g_last_ctx = g_ctx;
//
//     //实时获取数据
//     g_ctx.distance       = rx_data.shoot_dist;
//     g_ctx.allow_bullet   = uav_booster_ptr->get_data()->heat_control_ctx.allow_bullet_count;
//     g_ctx.fric_enable    = uav_booster_ptr->get_data()->cmd->fric_enable;
//     g_ctx.trigger_enable = uav_booster_ptr->get_data()->cmd->trigger_enable;
//     g_ctx.yaw            = gimbal_ptr->get_data()->data._current_imu_yaw_angle;
//     g_ctx.pitch          = gimbal_ptr->get_data()->data._current_imu_pitch_angle;
//     g_ctx.heat_max       = uav_booster_ptr->get_data()->shoot_data.Q_max;
// }
//
// // 彻底清空画布并重建
// void ui_global_refresh()
// {
//     ui_ptr->clear_all();
//     vTaskDelay(pdMS_TO_TICKS(100)); // 给裁判系统内核留出反应时间
//     info_context_collect();
//     g_force_refresh = true;         // 激活向下传递的强刷标志
//     ui_draw_static();
//     ui_update_dynamic();
// }
//
// void vt03_control(uint32_t notify_val)
// {
//     read_scope_lock lock(vt03_drv_t::get_lock());
//
//     if (notify_val & KEY_SHIFT)
//     {
//         g_ctx.refresh_request = true; // 触发手动刷新请求
//     }
//     if (notify_val & KEY_W_ON)
//     {
//         g_ctx.key_w_pressed = true;
//         move_forward();
//     }
//     if (notify_val & KEY_A_ON)
//     {
//         g_ctx.key_a_pressed = true;
//         turn_left();
//     }
//     if (notify_val & KEY_S_ON)
//     {
//         g_ctx.key_s_pressed = true;
//         move_backward();
//     }
//     if (notify_val & KEY_D_ON)
//     {
//         g_ctx.key_d_pressed = true;
//         turn_right();
//     }
//
//     if (notify_val & KEY_W_OFF) g_ctx.key_w_pressed = false;
//     if (notify_val & KEY_A_OFF) g_ctx.key_a_pressed = false;
//     if (notify_val & KEY_S_OFF) g_ctx.key_s_pressed = false;
//     if (notify_val & KEY_D_OFF) g_ctx.key_d_pressed = false;
//
//     if ((notify_val & KEY_W_OFF) || (notify_val & KEY_A_OFF)
//         || (notify_val & KEY_S_OFF) || (notify_val & KEY_D_OFF))
//     {
//         led_off();
//     }
// }
//
//     // 6. FreeRTOS UI 核心执行线程
//     void uav_ui_thread(void *argument)
//     {
//         // 1. 阻塞等待，直到裁判系统链路正式连通
//         while (!referee_ptr->is_online() || referee_ptr->get_robot_id() == 0)
//         {
//             vTaskDelay(pdMS_TO_TICKS(500));
//         }
//
//         led_init();
//         // 2. 首次进入执行一次全局重绘制
//         info_context_collect();
//         ui_global_refresh();
//
//         ui_ptr->draw_circle("auto",ui_operate::ADD,6,ui_color::GREEN,2,960,540,4);
//
//         while (true)
//         {
//             uint32_t notify_val = 0;
//             // 将超调时间改为稳定的 50ms 阻塞等待，取代死等延时
//             if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &notify_val, pdMS_TO_TICKS(50)) == pdTRUE)
//             {
//                 if (vt03_drv_t::instance().check_online())
//                 {
//                     vt03_control(notify_val);
//                 }
//             }
//
//             if (referee_ptr->is_online())
//             {
//                 info_context_collect(); // 填充 g_ctx
//
//                 // 判断是否按下了 Shift 键发起了强刷或者从小电脑收到了重构命令
//                 if (g_ctx.refresh_request != g_last_ctx.refresh_request || g_ctx.refresh_request == true)
//                 {
//                     ui_global_refresh();
//                     g_ctx.refresh_request = false; // 重置
//                 }
//                 else
//                 {
//                     ui_update_dynamic();
//                 }
//             }
//         }
//     }
//
// extern "C"
// {
//     void uav_ui_init(void *argument)
//     {
//         referee_ptr = referee_drv_t::get_instance();
//         ui_ptr      = new ui_drv_t(referee_ptr);
//
//         auto &vrc = rc_drv_t::read();
//         gimbal_ptr->start();
//
//         xTaskCreate(uav_ui_thread, "uav_ui_thread",1024,nullptr,
//                     configMAX_PRIORITIES - 3, &ui_task_handle);
//
//         btn_broker::subscribe(&vrc.keys.ctrl,  btn_event_t::PRESS_DOWN, ui_task_handle, KEY_CTRL);
//         btn_broker::subscribe(&vrc.keys.shift, btn_event_t::PRESS_DOWN, ui_task_handle, KEY_SHIFT);
//         btn_broker::subscribe(&vrc.keys.z,     btn_event_t::PRESS_DOWN, ui_task_handle, KEY_Z);
//
//         // 灯珠指示绑定
//         btn_broker::subscribe(&vrc.keys.w, btn_event_t::LONG_PRESS_START, ui_task_handle, KEY_W_ON);
//         btn_broker::subscribe(&vrc.keys.a, btn_event_t::LONG_PRESS_START, ui_task_handle, KEY_A_ON);
//         btn_broker::subscribe(&vrc.keys.s, btn_event_t::LONG_PRESS_START, ui_task_handle, KEY_S_ON);
//         btn_broker::subscribe(&vrc.keys.d, btn_event_t::LONG_PRESS_START, ui_task_handle, KEY_D_ON);
//
//         btn_broker::subscribe(&vrc.keys.w, btn_event_t::PRESS_UP, ui_task_handle, KEY_W_OFF);
//         btn_broker::subscribe(&vrc.keys.a, btn_event_t::PRESS_UP, ui_task_handle, KEY_A_OFF);
//         btn_broker::subscribe(&vrc.keys.s, btn_event_t::PRESS_UP, ui_task_handle, KEY_S_OFF);
//         btn_broker::subscribe(&vrc.keys.d, btn_event_t::PRESS_UP, ui_task_handle, KEY_D_OFF);
//
//         vTaskDelete(nullptr);
//     }
// }

#include "pyro_com_canrx.h"
#include "pyro_module_base.h"
#include "pyro_rc_base_drv.h"
#include "pyro_referee.h"
#include "pyro_ui_drv.h"
#include "pyro_uav_booster.h"
#include "pyro_uav_gimbal.h"
#include "pyro_vt03_rc_drv.h"
#include "led.h"
#include "pyro_autoaim_drv.h"

using namespace pyro;
// ReSharper disable CppExpressionWithoutSideEffects
static referee_drv_t *referee_ptr                   = nullptr;
static ui_drv_t *ui_ptr                             = nullptr;
extern uav_booster_t *uav_booster_ptr;
extern uav_gimbal_t *gimbal_ptr;
extern autoaim_drv_t::rx_data_t rx_data;

static uint32_t KEY_CTRL                           = (1 << 0);
static uint32_t KEY_SHIFT                          = (1 << 1);
static uint32_t KEY_Z                              = (1 << 2);
static uint32_t KEY_W                              = (1 << 3);
static uint32_t KEY_A                              = (1 << 4);
static uint32_t KEY_S                              = (1 << 5);
static uint32_t KEY_D                              = (1 << 6);

static TaskHandle_t ui_task_handle                  = nullptr;
static bool flush_flag                              = true;

//静态ui绘制
void ui_draw_static()
{
    ui_ptr->
    //自瞄框范围
     draw_rect("R01", ui_operate::ADD, 1, ui_color::GREEN, 3,
                       9940, 520, 980, 560)
     //摩擦轮、拨弹盘是否开启显示
     .draw_circle("F1", ui_operate::ADD, 5, ui_color::YELLOW, 4,
                       1700, 750, 30)
     .draw_circle("F2", ui_operate::ADD, 6, ui_color::YELLOW, 4,
                       1580, 750, 30)
     .draw_circle("T1", ui_operate::ADD, 6, ui_color::MAGENTA, 4,
                       1820, 750, 30);

     ui_ptr->flush(); // 拼包发送

    // 1. 绘制静态文本标签 (注意名字不能重复)
    //两轴角度
    ui_ptr->draw_string("ST1", pyro::ui_operate::ADD, 3, pyro::ui_color::PINK,
                        20, 4, 300, 700, " YAW :");
    ui_ptr->draw_string("ST2", pyro::ui_operate::ADD, 3, pyro::ui_color::PINK,
                        20, 4, 300, 620, "PITCH:");
    //距离
    ui_ptr->draw_string("ST3",pyro::ui_operate::ADD,4,pyro::ui_color::GREEN,
        20,4,1540,620," DIS :");
    //允许发弹量
    ui_ptr->draw_string("ST4",pyro::ui_operate::ADD,4,pyro::ui_color::GREEN,
        20,4,1540,540,"BULLET:");
    //热量数据
    ui_ptr->draw_string("ST6", pyro::ui_operate::ADD, 4, pyro::ui_color::GREEN,
                       20, 4, 750, 315, "HEAT:");
    ui_ptr->draw_string("ST7", pyro::ui_operate::ADD, 4, pyro::ui_color::YELLOW,
                        20, 4, 980, 315, "RES:");
    ui_ptr->flush();

    // 2. 为动态数值提前进行 ADD 占位，赋予初始值，方便后续直接 MODIFY
    ui_ptr->draw_float("DF3", pyro::ui_operate::ADD, 4, pyro::ui_color::WHITE, 20,
                     2, 510, 620, 0.0f)//yaw
            .draw_float("DF4", pyro::ui_operate::ADD, 4, pyro::ui_color::WHITE, 20,
                    2, 510, 540, 0.0f)//pitch
            .draw_float("heat_max", pyro::ui_operate::ADD, 5, pyro::ui_color::ALLY, 20,
                3, 850, 315, uav_booster_ptr->get_data()->shoot_data.Q_max)//最大热量
            .draw_float("heat_res", pyro::ui_operate::ADD, 1, pyro::ui_color::GREEN, 20,
                3, 1065, 315, 0.0f)//剩余热量
            //距离
            .draw_float("distance",pyro::ui_operate::ADD, 4, pyro::ui_color::CYAN, 20,
                3, 1670, 620, 0.0f)
            //允许发弹量
            .draw_float("bullet",pyro::ui_operate::ADD, 5, pyro::ui_color::PINK, 20,
                3, 1670, 540, 0.0f);//热量限制下允许的发弹量

    ui_ptr->draw_circle("F3", ui_operate::ADD, 5, ui_color::YELLOW, 1,
                       1700, 750, 30)//F3、F4表示两个摩擦轮是否开启
            .draw_circle("F4", ui_operate::ADD, 6, ui_color::YELLOW, 1,
                       1580, 750, 30)
            .draw_circle("T1", ui_operate::ADD, 6, ui_color::MAGENTA, 1,
                       1820, 750, 30);//拨弹盘是否开启

    ui_ptr->flush(); // 拼包发送
}

//绘制发射机构ui 是否开启摩擦轮 以及当前弹速
void update_booster_ui()
{
    float distance = rx_data.shoot_dist;

    float total = uav_booster_ptr->get_data()->heat_control_ctx.allow_bullet_count;

    ui_ptr->draw_float("distance",pyro::ui_operate::MODIFY, 4, pyro::ui_color::CYAN, 20,
                3, 1670, 620, distance);
    ui_ptr->draw_float("bullet",pyro::ui_operate::MODIFY, 5, pyro::ui_color::CYAN, 20,
                3, 1670, 540, total);

    if (uav_booster_ptr->get_data()->cmd->fric_enable)
    {
        ui_ptr->draw_circle("F3", ui_operate::MODIFY, 5, ui_color::GREEN,
              50, 1700, 750, 6)
              .draw_circle("F4", ui_operate::MODIFY, 6, ui_color::GREEN,
              50, 1580, 750, 6);
        if (uav_booster_ptr->get_data()->cmd->trigger_enable)
        {
            ui_ptr->draw_circle("T1", ui_operate::MODIFY, 6, ui_color::MAGENTA,
              50, 1820, 750, 6);
        }
    }
    else
    {
        // 关闭时，将圆圈改为白色（或背景色），数值归零
        ui_ptr->draw_circle("F3", ui_operate::MODIFY, 5, ui_color::YELLOW, 1,
                       1700, 750, 30)
        .draw_circle("F4", ui_operate::MODIFY, 6, ui_color::YELLOW, 1,
                       1580, 750, 30)
        .draw_circle("T1", ui_operate::MODIFY, 6, ui_color::MAGENTA, 4,
                       1820, 750, 30);
    }
}

//绘制两轴角度
void update_gimbal_ui()
{
    static float last_yaw = 0.0f, last_pitch = 0.0f;
    float yaw = gimbal_ptr->get_data()->data._current_imu_yaw_angle;
    float pitch = gimbal_ptr->get_data()->data._current_imu_pitch_angle;

    // 只有角度变化超过 0.1 度才发包
    if (std::abs(yaw - last_yaw) > 0.1f || std::abs(pitch - last_pitch) > 0.1f)
    {
        ui_ptr->draw_float("DF3", ui_operate::MODIFY, 4, ui_color::ALLY, 20, 4, 1670, 620, yaw)
              .draw_float("DF4", ui_operate::MODIFY, 4, ui_color::ALLY, 20, 4, 1670, 540, pitch);
        last_yaw = yaw;
        last_pitch = pitch;
    }
}

//调用update_aim_ui、update_gimbal_ui等动态刷新ui
void ui_update_dynamic()
{
    update_gimbal_ui();

    update_booster_ui();

    ui_ptr->flush();
}
//键盘手动刷新ui 控制灯带指示飞手
void vt03_control(uint32_t notify_val)
{
    read_scope_lock lock(vt03_drv_t::get_lock());
    const auto &vrc = rc_drv_t::read();

    if (notify_val & KEY_SHIFT)
    {
        flush_flag = true;
    }

    if (notify_val & KEY_W)
    {
        move_forward();
    }
    if (notify_val & KEY_A)
    {
        turn_left();
    }
    if (notify_val & KEY_S)
    {
        move_backward();
    }
    if (notify_val & KEY_D)
    {
        turn_right();
    }
}

extern "C"
{
    void uav_ui_thread(void *argument)
    {
        // 1. 阻塞等待裁判系统链路连通，并确保获取到了真实机器人ID
        while (!referee_ptr->is_online() || referee_ptr->get_robot_id() == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        led_init();

        // 2. 初始清理操作，并绘制静态结构
        ui_ptr->clear_all();
        vTaskDelay(pdMS_TO_TICKS(200));
        ui_draw_static();
        vTaskDelay(pdMS_TO_TICKS(100));

        while (true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, 0xFFFFFFFF, &notify_val, 0);

            if (vt03_drv_t::instance().check_online())
            {
                vt03_control(notify_val);
            }

            if (referee_ptr->is_online())
            {
                if (flush_flag)
                {
                    ui_ptr->clear_all();
                    vTaskDelay(pdMS_TO_TICKS(200));
                    ui_draw_static();
                    vTaskDelay(pdMS_TO_TICKS(100));

                    // flush_flag = false;
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

        auto &vrc =rc_drv_t::read();
        gimbal_ptr->start();

        xTaskCreate(uav_ui_thread, "uav_ui_thread", 512, nullptr,
                    configMAX_PRIORITIES - 3, &ui_task_handle);

        //刷新ui
        btn_broker::subscribe(&vrc.keys.ctrl, btn_event_t::PRESS_DOWN, ui_task_handle, KEY_CTRL);
        btn_broker::subscribe(&vrc.keys.shift, btn_event_t::PRESS_DOWN, ui_task_handle, KEY_SHIFT);
        btn_broker::subscribe(&vrc.keys.z, btn_event_t::PRESS_DOWN, ui_task_handle, KEY_Z);

        //点亮灯珠 来提示飞手
        btn_broker::subscribe(&vrc.keys.w, btn_event_t::LONG_PRESS_START, ui_task_handle, KEY_W);
        btn_broker::subscribe(&vrc.keys.a, btn_event_t::LONG_PRESS_START, ui_task_handle, KEY_A);
        btn_broker::subscribe(&vrc.keys.s, btn_event_t::LONG_PRESS_START, ui_task_handle, KEY_S);
        btn_broker::subscribe(&vrc.keys.d, btn_event_t::LONG_PRESS_START, ui_task_handle, KEY_D);

        vTaskDelete(nullptr);
    }
}