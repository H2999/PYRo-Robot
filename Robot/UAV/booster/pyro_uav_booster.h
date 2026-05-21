#ifndef PYRO_UAV_BOOSTER_H
#define PYRO_UAV_BOOSTER_H

#include "protocol.h"
#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "pyro_referee.h"
#include "booster_config.h"

namespace pyro
{

#define test_speed 0

//命令模板
struct uav_booster_cmd_t final : public cmd_base_t
{
    bool fric_enable;
    bool trigger_enable;      // 拨弹开启

    float target_trigger_radps;
    bool single_mode;
    bool continue_mode;
    bool auto_mode;
    uint8_t booster_auto_flag;

    uav_booster_cmd_t()
        :fric_enable(false),target_trigger_radps(0),
        single_mode(false), continue_mode(false),auto_mode(false),booster_auto_flag(0)
    {
    }
};

struct uav_booster_cfg_t
{
    struct motor_cfg_t
    {
        motor_base_t *fric_wheel[2]{nullptr};
        motor_base_t *trigger_wheel{nullptr};
    };

    struct pid_cfg_t
    {
        pid_t *fric_pid[2]{nullptr};
        pid_t *trigger_position_pid{nullptr};
        pid_t *trigger_speed_pid{nullptr};

        pid_t *shoot_closed_pid{nullptr};
    };

    motor_cfg_t motor_cfg;
    pid_cfg_t pid_cfg;
};

//具体实现的模板类 继承自module模块
class uav_booster_t : public module_base_t<uav_booster_t,uav_booster_cmd_t,uav_booster_cfg_t>
{
    friend class module_base_t;
    friend class jcom_drv_t;

    struct data_ctx_t;
    struct booster_auto_ctx_t;
    struct booster_ctx_t;
    struct heat_control_t;
    struct shoot_data_t;
    struct referee_ctx_t;
    struct IIR_Filter_ctx_t;
    struct shoot_delay_ctx_t;


public:
    uav_booster_t(const uav_booster_t &) = delete;
    uav_booster_t & operator = (const uav_booster_t &) = delete;

    [[nodiscard]] uint8_t get_robot_id() const;
    booster_ctx_t* get_data();
    float heat_calculate();

private:
    uav_booster_t();
    ~uav_booster_t() override = default;

    //接口
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    //派生方法
    void fric_control();
    void trigger_position_control();
    void trigger_speed_control();
    void send_fric_command() const;
    void speed_control();
    void speed_filter();
    void send_trigger_command() const;

    //预测校准热量
    void heat_control(float current_time_ms);

    static float normalize_angle(float angle);

    struct data_ctx_t
    {
        float torque{};

        float last_motor_rad{0};
        float total_trigger_rad{0};

        //当前
        float current_fric_mps[2]{};
        float current_trigger_rad{0};
        float current_trigger_radps{};
        float current_trigger_torque{0};
        float current_fric_torque{};

        //目标
        float target_fric_mps[2]{};
        float target_trigger_rad{0};
        float target_trigger_radps{0};

        //输出扭矩
        float fric_output_torque[2]{};
        float trigger_output_torque{0};
        float last_trigger_rad{0};
        float accumulated_rad{};

        float accumulated_rad_shoot_delay{};

        float last_shot_time_ms{};
        float distance{};
    };

    struct heat_control_t
    {
        bool single_fresh_referee{false};
        bool continue_fresh_referee{false};
        bool auto_fresh_referee{false};

        int16_t allow_bullet_count{};

        float local_heat{};
        float last_shot_time_ms{};
    };

    struct shoot_delay_ctx_t
    {
        float aim_timestamp{0};           // 自瞄数据到达时间
        float shoot_cmd_timestamp{0};     // 发出射击指令时间
        float total_system_delay{0};      // 完整系统延迟
        bool waiting_for_launch{false};   // 是否在等待击发
    };

    struct shoot_data_t
    {
        //用来刚上电时反转矫正 防止空程和双发
        bool is_reset_finished{false};
        //用来给视觉发送enermy_color
        float robot_id{};
        uint8_t robot_level{};
        uint8_t step_flag{};
        float launching_frequency{};

        // heat_control_t HeatControlParams[11]
        // {
        //     {0},
        //     // 等级1: Q_max 100, 80开始减速，50降到w_min
        //     {100, 80.0f, 20, 4.5, 9.5},
        //     // 等级2: Q_max 110, 80开始
        //     {110, 80.0f, 30, 5.0, 10.5},
        //     // 等级3: Q_max 120, 80开始
        //     {120, 80.0f , 40, 5.0, 11.0},
        //     // 等级4: Q_max 130, 90开始
        //     {130, 90.0f, 50, 5.8, 11.9},
        //     // 等级5: Q_max 140
        //     {140, 90.0f, 60, 6.0, 13.0},
        //     // 等级6: Q_max 150, 100开始
        //     {150, 100.0f, 70,7.0, 12.5},
        //     // 等级7: Q_max 160, 110开始
        //     {160, 110.0f, 80, 7.5, 12.5},
        //     // 等级8: Q_max 170, 110开始
        //     {170, 110.0f, 90, 8.5, 14.8},
        //     // 等级9: Q_max 180, 120开始
        //     {180, 120.0f, 100, 8.5, 15.2},
        //     // 等级10: Q_max 200, 120开始
        //     {200, 120.0f, 120, 9.5, 15.8}
        // };

        //用来弹速闭环
        float last_bullet_speed_mps{0};
        float now_bullet_speed_mps{0};
        float ball_speed[3]{0.0f};
        float speed_increment{0};
        //目标转速
        float target_bullet_speed = 23.0f;

        float fric_mps = 19.3f;

        float Q_max{};
        float Q_cd{};
        float Q_now_referee{};
        float Q_now_no_referee{};
        float Q_res{};

        bool is_shooting_locked{};
        int16_t bullet_quota{};
        int16_t bullets_shot_in_burst{};
    };

    struct referee_ctx_t
    {
        //裁判系统数据 来获取弹速做弹速闭环
        referee_drv_t *referee_drv{nullptr};
        referee_data_t referee_data{};
    };

    struct IIR_Filter_ctx_t
    {
        float f[3]{};
    };

    struct booster_ctx_t
    {
        uav_booster_cfg_t cfg;
        data_ctx_t data_ctx;
        uav_booster_cmd_t *cmd{};
        shoot_data_t shoot_data{};
        referee_ctx_t referee_ctx{};
        IIR_Filter_ctx_t IIR_Filter_ctx[2]{};
        shoot_delay_ctx_t shoot_delay_ctx{};
        heat_control_t heat_control_ctx{};
    };

    booster_ctx_t booster_ctx;

    struct passive_state_t final : public state_t<uav_booster_t>
    {
        void enter(uav_booster_t *owner) override;
        void execute(uav_booster_t *owner) override;
        void exit(uav_booster_t *owner) override;
    };

    struct fsm_active_t final : public fsm_t<uav_booster_t>
    {
        struct reset_state_t final : public state_t<uav_booster_t>
        {
            void enter(uav_booster_t *owner) override;
            void execute(uav_booster_t *owner) override;
            void exit(uav_booster_t* ctx) override;
        private:
            float turnback_start_time{};
        };

        struct state_middle_t final : public state_t<uav_booster_t>
        {
            void enter(uav_booster_t *owner) override;
            void execute(uav_booster_t *owner) override;
            void exit(uav_booster_t *owner) override;
        };

        struct shoot_single_bullet_t final : public state_t<uav_booster_t>
        {
            void enter(uav_booster_t *owner) override;
            void execute(uav_booster_t *owner) override;
            void exit(uav_booster_t *owner) override;

        private:
            float start_time{};
            float timeout_ms = 10.0f;
        };

        struct shoot_continue_bullet_t final : public state_t<uav_booster_t>
        {
            void enter(uav_booster_t *owner) override;
            void execute(uav_booster_t *owner) override;
            void exit(uav_booster_t *owner) override;
        };

        struct shoot_stall_t final : public state_t<uav_booster_t>
        {
            void enter(uav_booster_t *owner) override;
            void execute(uav_booster_t *owner) override;
            void exit(uav_booster_t *owner) override;
        };

        struct shoot_auto_aim_t final : public state_t<uav_booster_t>
        {
            void enter(uav_booster_t *owner) override;
            void execute(uav_booster_t *owner) override;
            void exit(uav_booster_t *owner) override;
        };

        void on_enter(uav_booster_t *owner) override;
        void on_execute(uav_booster_t *owner) override;
        void on_exit(uav_booster_t *owner) override;

    private:
        reset_state_t reset_state;
        state_middle_t middle_state;
        shoot_single_bullet_t single_state;
        shoot_continue_bullet_t continue_state;
        shoot_stall_t stall_state;
        shoot_auto_aim_t auto_aim_state;
    };

    passive_state_t passive_state;
    fsm_active_t active_state;
    fsm_t<uav_booster_t> main_fsm;
};

};

#endif // PYRO_UAV_BOOSTER_H
