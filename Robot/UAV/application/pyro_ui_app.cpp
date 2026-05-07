#include "pyro_com_canrx.h"
#include "pyro_module_base.h"
#include "pyro_rc_base_drv.h"
#include "pyro_referee.h"
#include "pyro_ui_drv.h"
#include "pyro_uav_booster.h"
#include "pyro_uav_gimbal.h"
#include "pyro_vt03_rc_drv.h"
#include "../WS2812/WS2812.h"

using namespace pyro;
// ReSharper disable CppExpressionWithoutSideEffects
static referee_drv_t *referee_ptr                   = nullptr;
static ui_drv_t *ui_ptr                             = nullptr;
static WS2812_drv_t* light_ptr                      = nullptr;
extern uav_booster_t *uav_booster_ptr;
extern uav_gimbal_t *gimbal_ptr;

static uint32_t KEY_CTRL                           = (1 << 0);
static uint32_t KEY_SHIFT                          = (1 << 1);
static uint32_t KEY_Z                              = (1 << 2);
static uint32_t KEY_W_ON                           = (1 << 3);
static uint32_t KEY_A_ON                           = (1 << 4);
static uint32_t KEY_S_ON                           = (1 << 5);
static uint32_t KEY_D_ON                           = (1 << 6);

static uint32_t KEY_W_OFF                          = (1 << 7);
static uint32_t KEY_A_OFF                          = (1 << 8);
static uint32_t KEY_S_OFF                          = (1 << 9);
static uint32_t KEY_D_OFF                          = (1 << 10);

static TaskHandle_t ui_task_handle                  = nullptr;

static float fric1_mps                              = 0.0f;
static float fric2_mps                              = 0.0f;
static bool flush_flag                              = false;
static bool heat_bar_initialized = false;

//静态ui绘制
void ui_draw_static()
{
    // 1. 绘制矩形 (RECT)
    // 参数：名字, 操作, 图层, 颜色, 线宽, 起点X, 起点Y, 终点X, 终点Y

    // ui_ptr
    //    ->draw_line("L01", pyro::ui_operate::ADD, 1, pyro::ui_color::GREEN, 4,
    //                860, 615, 1060, 615)
    //    .draw_line("L02", ui_operate::ADD, 1, pyro::ui_color::ORANGE, 4, 860,
    //               585, 1060, 585);

    ui_ptr->draw_rect("R01", ui_operate::ADD, 1, ui_color::GREEN, 3,
                      815, 455, 1105, 620)
    //热量进度条
    .draw_rect("HEAT_BG", ui_operate::ADD, 1, ui_color::CYAN, 1,
              715, 140, 1015, 230)
    .draw_rect("HEAT_BAR", ui_operate::ADD, 1, ui_color::GREEN, 3,
                  715, 140, 715, 230)
    .draw_line("FRIC1_SPEED", ui_operate::ADD, 1, ui_color::YELLOW, 4,
                            1520, 580, 1820,580)
    .draw_line("FRIC2_SPEED", ui_operate::ADD, 1, ui_color::YELLOW, 4,
                            1520, 410, 1820,410)
    .draw_circle("FRIC1_MODE", ui_operate::ADD, 1, ui_color::YELLOW, 4,
                        1580, 750, 30)
    .draw_circle("FRIC2_MODE", ui_operate::ADD, 1, ui_color::YELLOW, 4,
                        1700, 750, 30);

    ui_ptr->flush(); // 拼包发送
    // 1. 绘制静态文本标签 (注意名字不能重复)
    ui_ptr->draw_string("ST1", pyro::ui_operate::ADD, 3, pyro::ui_color::PINK,
                        20, 4, 1540, 620, " YAW :");
    ui_ptr->draw_string("ST2", pyro::ui_operate::ADD, 3, pyro::ui_color::PINK,
                        20, 4, 1540, 450, "PITCH:");

    ui_ptr->draw_string("ST3", pyro::ui_operate::ADD, 3, pyro::ui_color::ALLY,
                        20, 2, 110, 800, "FRIC1:");
    ui_ptr->draw_string("ST4", pyro::ui_operate::ADD, 3, pyro::ui_color::ALLY,
                        20, 2, 110, 750, "FRIC2:");

    ui_ptr->draw_string("ST5", pyro::ui_operate::ADD, 3, pyro::ui_color::GREEN,
                       20, 4, 800, 115, "HEAT:");
    ui_ptr->draw_string("ST6", pyro::ui_operate::ADD, 3, pyro::ui_color::GREEN,
                        20, 4, 920, 115, "PREC:");

    // 2. 为动态数值提前进行 ADD 占位，赋予初始值，方便后续直接 MODIFY
    ui_ptr
        ->draw_float("DF1", pyro::ui_operate::ADD, 4, pyro::ui_color::WHITE, 20,
                     2, 230, 800, 0.0f)
        .draw_float("DF2", pyro::ui_operate::ADD, 4, pyro::ui_color::WHITE, 20,
                    2, 230, 750, 0.0f)
        .draw_float("DF3", pyro::ui_operate::ADD, 4, pyro::ui_color::WHITE, 20,
                     2, 1600, 580, 0.0f)
        .draw_float("DF4", pyro::ui_operate::ADD, 4, pyro::ui_color::WHITE, 20,
                    2, 1600, 410, 0.0f);
    ui_ptr->flush(); // 拼包发送
}
//绘制自瞄ui
void update_aim_ui()
{
    static float ui_radius = 80.0f;
    // 记录图形是否存在 因为delete之后要先add 动态修改才用modify
    static bool is_on_screen = false;
    ui_color target_color = ui_color::WHITE;

    bool auto_aim_on = gimbal_ptr->get_data()->cmd->auto_flag;
    bool target_locked = gimbal_ptr->get_data()->ui_ctx.is_aiming_locked;

    if (!auto_aim_on)
    {
        if (is_on_screen)
        {
            // 只有当它在屏幕上时才执行删除
            ui_ptr->draw_circle("AIM", ui_operate::DELETE, 0, ui_color::WHITE, 0, 0, 0, 0);
            is_on_screen = false; // 重置标记
        }
        return;
    }

    // 3. 动效逻辑计算
    if (target_locked)
    {
        target_color = ui_color::MAGENTA;
        if (ui_radius > 30.0f) ui_radius -= 10.0f;
    }
    else
    {
        target_color = ui_color::CYAN;
        if (ui_radius < 60.0f) ui_radius += 5.0f;
    }

    // 4. 绘图：判断是该 ADD 还是 MODIFY
    if (!is_on_screen)
    {
        // 第一次开启自瞄，必须用 ADD
        ui_ptr->draw_circle("AIM", ui_operate::ADD, 2,
                            target_color, 4, 960, 540, static_cast<uint16_t>(ui_radius));
        is_on_screen = true;
    }
    else
    {
        // 图形已存在，执行平滑修改
        ui_ptr->draw_circle("AIM", ui_operate::MODIFY, 2,
                            target_color, 4, 960, 540, static_cast<uint16_t>(ui_radius));
    }
}
//绘制热量进度条
void update_heat_progress_bar(float current_heat, float max_heat)
{
    // 计算热量百分比
    float heat_percentage = 0.0f;
    if (max_heat > 0.0f)
    {
        heat_percentage = current_heat / max_heat;
    }

    heat_percentage = std::clamp(heat_percentage, 0.0f, 1.0f);

    const uint16_t start_x = 715;
    const uint16_t start_y = 140;
    const uint16_t end_x = 1015;
    const uint16_t end_y = 230;
    const uint16_t total_width = end_x - start_x;  // 300px

    // 计算当前进度条终点
    uint16_t current_end_x = start_x + static_cast<uint16_t>(total_width * heat_percentage);

    ui_color bar_color = ui_color::GREEN;
    if (heat_percentage > 0.8f) bar_color = ui_color::ORANGE;
    else if (heat_percentage > 0.5f) bar_color = ui_color::YELLOW;

    // 删除旧的进度条（如果存在）
    if (!heat_bar_initialized)
    {
        ui_ptr->draw_rect("HEAT_BAR", ui_operate::ADD, 1, bar_color, 3,
                         start_x, start_y, start_x, end_y);  // 初始宽度0
        heat_bar_initialized = true;
    }

    // 之后每次用 MODIFY 更新
    ui_ptr->draw_rect("HEAT_BAR", ui_operate::MODIFY, 1, bar_color, 3,
                     start_x, start_y, current_end_x, end_y);

    // 添加新的进度条（只有当进度>0时才绘制）
    if (current_end_x > start_x)
    {
        ui_ptr->draw_rect("HEAT_BAR", ui_operate::ADD, 1, bar_color, 3,
                         start_x, start_y, current_end_x, end_y);
        heat_bar_initialized = true;
    }
    else
    {
        heat_bar_initialized = false;
    }

    // 可选：显示热量数值和百分比
    ui_ptr->draw_float("HEAT_VAL", ui_operate::MODIFY, 4, ui_color::WHITE,
                       20, 2, 860, 115, current_heat)
          .draw_float("HEAT_PCT", ui_operate::MODIFY, 4, ui_color::WHITE,
                       20, 2, 980, 115, heat_percentage * 100.0f);
}
//绘制发射机构ui 是否开启摩擦轮 以及当前弹速
void update_booster_ui()
{
    fric1_mps = uav_booster_ptr->get_data()->data_ctx.current_fric_mps[0];
    fric2_mps = uav_booster_ptr->get_data()->data_ctx.current_fric_mps[1];

    // 获取热量数据
    float current_heat = uav_booster_ptr->get_data()->shoot_data.Q_now_no_referee;
    float max_heat = uav_booster_ptr->get_data()->shoot_data.Q_max;

    // 更新热量进度条
    update_heat_progress_bar(current_heat, max_heat);

    if (uav_booster_ptr->get_data()->cmd->fric_enable)
    {
        // 只有开启时才显示绿色，且只用 MODIFY
        ui_ptr->draw_circle("C03", ui_operate::MODIFY, 1, ui_color::GREEN,
              35, 1580, 750, 17)
              .draw_circle("C04", ui_operate::MODIFY, 1, ui_color::GREEN,
              35, 1700, 750, 17)
              .draw_float("DF1", ui_operate::MODIFY, 4, ui_color::YELLOW,
              20, 2, 230, 800, fric1_mps)
              .draw_float("DF2", ui_operate::MODIFY, 4, ui_color::YELLOW,
              20, 2, 230, 750, fric2_mps);
    }
    else
    {
        // 关闭时，将圆圈改为白色（或背景色），数值归零
        ui_ptr->draw_circle("C03", ui_operate::MODIFY, 1, ui_color::WHITE,
              1, 1580, 750, 17)
              .draw_circle("C04", ui_operate::MODIFY, 1, ui_color::WHITE,
              1, 1700, 750, 17)
              .draw_float("DF1", ui_operate::MODIFY, 4, ui_color::WHITE,
              20, 2, 230, 800, 0.0f)
              .draw_float("DF2", ui_operate::MODIFY, 4, ui_color::WHITE,
              20, 2, 230, 750, 0.0f);
    }
}
//绘制两轴角度
void update_gimbal_ui()
{
    float yaw_angle = gimbal_ptr->get_data()->data._current_imu_yaw_angle;
    float pitch_angle = gimbal_ptr->get_data()->data._current_imu_pitch_angle;

    ui_ptr->draw_float("DF3", pyro::ui_operate::MODIFY, 4,
                        pyro::ui_color::ALLY, 20, 4, 1600, 620, yaw_angle)
    .draw_float("DF4", pyro::ui_operate::MODIFY, 4,
                        pyro::ui_color::ALLY, 20, 4, 1600, 450, pitch_angle);
}
//调用update_aim_ui、update_gimbal_ui等动态刷新ui
void ui_update_dynamic()
{
    update_gimbal_ui();

    update_booster_ui();

    update_aim_ui();

    update_heat_progress_bar(uav_booster_ptr->get_data()->shoot_data.Q_now_no_referee,
                                                    uav_booster_ptr->get_data()->shoot_data.Q_max);

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

    if (notify_val & KEY_W_ON)
    {
        light_ptr->set_all(57, 255, 0); // 荧光绿
    }
    if (notify_val & KEY_S_ON)
    {
        light_ptr->set_all(255, 255, 0); //荧光黄
    }
    if (notify_val & KEY_A_ON)
    {
        light_ptr->set_all(255, 100, 0); //荧光橙
    }
    if (notify_val & KEY_D_ON)
    {
        light_ptr->set_all(255, 0, 255); //荧光粉
    }

    if ((notify_val & KEY_W_OFF) || (notify_val & KEY_S_OFF)
        || (notify_val & KEY_A_OFF) || (notify_val & KEY_D_OFF))
    {
        light_ptr->set_all(0, 0, 0); // 关闭
    }

    if (!light_ptr->WS2812_isbusy())
    {
        light_ptr->update_light();
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

        //初始让灯灭
        light_ptr->set_all(0, 0, 0);
        light_ptr->update_light();

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

                    heat_bar_initialized = false;
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
        light_ptr   = WS2812_drv_t::get_instance();
        ui_ptr      = new ui_drv_t(referee_ptr);

        light_ptr->WS2812_Init();

        auto &vrc =rc_drv_t::read();
        gimbal_ptr->start();

        xTaskCreate(uav_ui_thread, "uav_ui_thread", 512, nullptr,
                    configMAX_PRIORITIES - 4, &ui_task_handle);

        //刷新ui
        btn_broker::subscribe(&vrc.keys.ctrl, btn_event_t::PRESS_DOWN, ui_task_handle, KEY_CTRL);
        btn_broker::subscribe(&vrc.keys.shift, btn_event_t::PRESS_DOWN, ui_task_handle, KEY_SHIFT);
        btn_broker::subscribe(&vrc.keys.z, btn_event_t::PRESS_DOWN, ui_task_handle, KEY_Z);

        //点亮灯珠 来提示飞手
        btn_broker::subscribe(&vrc.keys.w, btn_event_t::LONG_PRESS_START, ui_task_handle, KEY_W_ON);
        btn_broker::subscribe(&vrc.keys.a, btn_event_t::LONG_PRESS_START, ui_task_handle, KEY_A_ON);
        btn_broker::subscribe(&vrc.keys.s, btn_event_t::LONG_PRESS_START, ui_task_handle, KEY_S_ON);
        btn_broker::subscribe(&vrc.keys.d, btn_event_t::LONG_PRESS_START, ui_task_handle, KEY_D_ON);

        btn_broker::subscribe(&vrc.keys.w, btn_event_t::PRESS_UP, ui_task_handle, KEY_W_OFF);
        btn_broker::subscribe(&vrc.keys.a, btn_event_t::PRESS_UP, ui_task_handle, KEY_A_OFF);
        btn_broker::subscribe(&vrc.keys.s, btn_event_t::PRESS_UP, ui_task_handle, KEY_S_OFF);
        btn_broker::subscribe(&vrc.keys.d, btn_event_t::PRESS_UP, ui_task_handle, KEY_D_OFF);

        vTaskDelete(nullptr);
    }
}