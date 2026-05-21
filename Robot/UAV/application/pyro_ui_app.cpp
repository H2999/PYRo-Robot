#include "pyro_com_canrx.h"
#include "pyro_module_base.h"
#include "pyro_rc_base_drv.h"
#include "pyro_referee.h"
#include "pyro_ui_drv.h"
#include "pyro_uav_booster.h"
#include "pyro_uav_gimbal.h"
#include "pyro_vt03_rc_drv.h"
#include "led.h"

using namespace pyro;
// ReSharper disable CppExpressionWithoutSideEffects
static referee_drv_t *referee_ptr                   = nullptr;
static ui_drv_t *ui_ptr                             = nullptr;
extern uav_booster_t *uav_booster_ptr;
extern uav_gimbal_t *gimbal_ptr;

static uint32_t KEY_CTRL                           = (1 << 0);
static uint32_t KEY_SHIFT                          = (1 << 1);
static uint32_t KEY_Z                              = (1 << 2);
static uint32_t KEY_W                              = (1 << 3);
static uint32_t KEY_A                              = (1 << 4);
static uint32_t KEY_S                              = (1 << 5);
static uint32_t KEY_D                              = (1 << 6);

static TaskHandle_t ui_task_handle                  = nullptr;
static bool flush_flag                              = true;

// 定义弹道测试数据点
struct BallisticPoint {
    float distance;     // 距离
    int32_t bias_x;     // 当中心对准目标时，实际落点偏离中心的X像素
    int32_t bias_y;     // 当中心对准目标时，实际落点偏离中心的Y像素
};

// ==================== 测试数据表 ====================
// 比如8米时，实际落点在 偏左20、偏下30
// 那么反向补偿：叉应该画在 偏右20（+20）、偏上30（-30，假设屏幕y向下为正）
static constexpr BallisticPoint ballistic_table[] = {
    {8.0f,  20, -30},     // 8米 (根据你的例子：实际落点偏左偏下，所以UI往右往上画)
    {10.0f, 25, -45},     // 10米
    {12.0f, 30, -50},
    {13.0f, 35, -60},
    {14.0f, 40, -65},
    {15.0f, 45, -70},
    {16.0f, 40, -70}
};
static constexpr size_t TABLE_SIZE = sizeof(ballistic_table) / sizeof(BallisticPoint);

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
                       1820, 750, 30)//拨弹盘是否开启
            .draw_line("XL1",ui_operate::ADD, 6, ui_color::YELLOW, 1,
                        0,0,0,0)
            .draw_line("XL2",ui_operate::ADD, 6, ui_color::YELLOW, 1,
                        0,0,0,0);

    ui_ptr->flush(); // 拼包发送
}

void update_shoot_ui()
{
    float distance = uav_booster_ptr->get_data()->data_ctx.distance;

    ui_ptr->draw_float("distance",pyro::ui_operate::MODIFY, 4, pyro::ui_color::CYAN, 20,
                3, 1670, 620, distance);

    int32_t center_x = 1920 / 2; // 960
    int32_t center_y = 1080 / 2; // 540

    int32_t target_bias_x = 0;
    int32_t target_bias_y = 0;

    // ==================== 线性插值计算当前距离的偏差 ====================
    if (distance <= ballistic_table[0].distance)
    {
        target_bias_x = ballistic_table[0].bias_x;
        target_bias_y = ballistic_table[0].bias_y;
    }
    else if (distance >= ballistic_table[TABLE_SIZE - 1].distance)
    {
        target_bias_x = ballistic_table[TABLE_SIZE - 1].bias_x;
        target_bias_y = ballistic_table[TABLE_SIZE - 1].bias_y;
    }
    else
    {
        // 在表格中间，进行线性插值
        for (size_t i = 0; i < TABLE_SIZE - 1; ++i)
        {
            if (distance >= ballistic_table[i].distance && distance <= ballistic_table[i+1].distance)
            {
                float t = (distance - ballistic_table[i].distance) /
                          (ballistic_table[i+1].distance - ballistic_table[i].distance);

                target_bias_x = ballistic_table[i].bias_x + static_cast<int32_t>(t * static_cast<float>(ballistic_table[i+1].bias_x - ballistic_table[i].bias_x));
                target_bias_y = ballistic_table[i].bias_y + static_cast<int32_t>(t * static_cast<float>(ballistic_table[i+1].bias_y - ballistic_table[i].bias_y));
                break;
            }
        }
    }

    // ==================== 计算最终 叉 在屏幕上的绝对像素坐标 ====================
    center_x += target_bias_x;
    center_y += target_bias_y;

    // ==================== 更新 叉 号 ====================
    uint16_t cross_size = 12; // 叉叉的大小（像素半径）

    ui_ptr->draw_line("XL1", ui_operate::MODIFY, 6, ui_color::ORANGE, 3,
                      center_x - cross_size, center_y + cross_size,
                      center_x + cross_size, center_y - cross_size)
           .draw_line("XL2", ui_operate::MODIFY, 6, ui_color::ORANGE, 3,
                      center_x - cross_size, center_y - cross_size,
                      center_x + cross_size, center_y + cross_size);
}

//绘制发射机构ui 是否开启摩擦轮 以及当前弹速
void update_booster_ui()
{
    float total = uav_booster_ptr->get_data()->heat_control_ctx.allow_bullet_count;
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

        auto &vrc =rc_drv_t::read();
        gimbal_ptr->start();

        xTaskCreate(uav_ui_thread, "uav_ui_thread", 512, nullptr,
                    configMAX_PRIORITIES - 4, &ui_task_handle);

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