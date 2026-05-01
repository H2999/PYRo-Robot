#ifndef PYRO_UAV_BOOSTER_H
#define PYRO_UAV_BOOSTER_H

#include "protocol.h"
#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "pyro_referee.h"

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
    uint8_t booster_auto_flag;

    uav_booster_cmd_t()
        :fric_enable(false),target_trigger_radps(0),
        single_mode(false), continue_mode(false),booster_auto_flag(0)
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

public:
    uav_booster_t(const uav_booster_t &) = delete;
    uav_booster_t & operator = (const uav_booster_t &) = delete;

    [[nodiscard]] uint8_t get_robot_id() const;

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
    void send_fric_command();
    void speed_control();
    void speed_filter();
    void send_trigger_command();
    float heat_control(int level, float Q_res);

    static float normalize_angle(float angle);

    struct data_ctx_t
    {
        float last_motor_rad{0};
        float total_trigger_rad{0};

        //当前
        float current_fric_mps[2]{};
        float current_trigger_rad{0};
        float current_trigger_radps{};
        float current_trigger_torque{0};

        //目标
        float target_fric_mps[2]{};
        float target_trigger_rad{0};
        float target_trigger_radps{0};

        //输出扭矩
        float fric_output_torque[2]{};
        float trigger_output_torque{0};
    };

    struct booster_auto_ctx_t
    {
        uint8_t fire_enable;
        float avg_speed;
    };

    // struct heat_control_t
    // {
    //     //功率控制相关数据
    //     float Q_max{};
    //     float Q_cd{};
    //     float Q_now{};
    //     float Q_res{};
    //
    //     uint8_t Q_warn{200};
    //     uint8_t Q_saturation{50};
    //     uint8_t Q_threshold{20};
    //
    //     float speed_max{9.0f};
    //     float trigger_speed{};
    // };

    struct heat_control_t{
        float Q_max;         // 热量上限
        float cooling_rate; // 冷却速率
        float w_min;        // 与冷却速率持平的弹速
        float w_max;        // 最大转速
    };

    struct shoot_data_t
    {
        //用来刚上电时反转矫正 防止空程和双发
        bool is_reset_finished{false};
        //用来给视觉发送enermy_color
        float robot_id{};
        uint8_t robot_level{};

        heat_control_t HeatControlParams[11]
        {
            //测得的是给10rad/s 1s能打16发 这里以等级1为例 Q_max是100 所以最多打10发 所以最大弹速就是 10 * (10 / 16)
            {0},
            // 等级1
            {100, 20, 4.4, 9.5},
            // 等级2
            {110, 30, 4.8, 9.8},
            // 等级3
            {120, 40, 5.6, 11.2},
            // 等级4
            {130, 50, 6.0, 11.6},
            // 等级5
            {140, 60, 6.5, 12.3},
            // 等级6
            {150, 70, 6.5, 12.3},
            // 等级7
            {160, 80, 6.5, 12.3},
            // 等级8
            {170, 90, 6.5, 12.3},
            // 等级9
            {180, 100, 6.5, 12.3},
            // 等级10
            {200, 120, 6.5, 12.3}
        };
        //用来弹速闭环
        float last_bullet_speed_mps{0};
        float now_bullet_speed_mps{0};
        float ball_speed[5]{0.0f};
        float speed_increment{0};
        //目标转速
        float target_bullet_speed = 22.5f;
        //施加给摩擦轮的速度
        float fric_mps = 20.7f;

        float Q_max{};
        float Q_cd{};
        float Q_now{};
        float Q_res{};
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
        booster_auto_ctx_t auto_ctx{};
        shoot_data_t shoot_data{};
        referee_ctx_t referee_ctx{};
        IIR_Filter_ctx_t IIR_Filter_ctx[2]{};
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

    static constexpr float FRIC1_RADIUS = 0.03f;
    static constexpr float FRIC2_RADIUS = 0.03f;

    static constexpr float reduction_ratio = 36.0f;
    static constexpr float reciprocal_reduction_ratio =  0.0277777777777777f;

    static constexpr float filter_num[3] = {1.562916920892f, -0.6413063774028f, 0.07838945651057f};
};

};

#endif // PYRO_UAV_BOOSTER_H
