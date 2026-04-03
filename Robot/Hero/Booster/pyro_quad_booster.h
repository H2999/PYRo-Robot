#ifndef __PYRO_QUAD_BOOSTER_H__
#define __PYRO_QUAD_BOOSTER_H__

#include "pyro_algo_pid.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_module_base.h"
#include <cmath>

namespace pyro
{

// =========================================================
// 1. 命令定义
// =========================================================
struct quad_booster_cmd_t final : public cmd_base_t
{
    bool fric_on;     // 摩擦轮开启
    bool fire_enable; // 拨弹开启
    bool speed_contorl_en;
    bool reset_trig;
    float target_speed;

    quad_booster_cmd_t()
        : fric_on(false), fire_enable(false), speed_contorl_en(true), reset_trig(false), target_speed(0.0f)
    {
    }
};

struct quad_booster_cfg_t
{
};

// =========================================================
// 2. 四轮发射机构类
// =========================================================
class quad_booster_t final
    : public module_base_t<quad_booster_t, quad_booster_cmd_t,
                           quad_booster_cfg_t>
{
    friend class module_base_t<quad_booster_t, quad_booster_cmd_t,
                               quad_booster_cfg_t>;
    friend class jcom_drv_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct booster_ctx_t;

  public:
    quad_booster_t(const quad_booster_t &)            = delete;
    quad_booster_t &operator=(const quad_booster_t &) = delete;
    [[nodiscard]] booster_ctx_t& get_ctx();

  private:
    quad_booster_t();
    ~quad_booster_t() override = default;

    // --- 接口实现 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 内部辅助 ---
    void _speed_contorl();
    void _fric_control();
    bool _heat_control();
    void _trigger_position_control();
    void _trigger_speed_control();
    void _send_fric_command() const;
    void _send_trigger_command() const;
    void _launch_delay_calculate();

    // 角度归一化辅助函数
    static float _normalize_angle(float angle);

    // --- 成员变量 ---
    struct motor_ctx_t
    {
        motor_base_t *fric_wheels[4]{nullptr};
        motor_base_t *trigger_wheel{nullptr};
    };

    struct pid_ctx_t
    {
        pid_t *fric_pid[4]{nullptr};
        pid_t *trigger_pos_pid{nullptr};
        pid_t *trigger_spd_pid{nullptr};
        pid_t *ball_speed_pid{nullptr};
    };

    struct data_ctx_t
    {
        float launch_delay_timer[3]{}; // 发射延时计时器
        float avg_launch_delay{0};       // 平均发射延时
        float signal_timer{0};         // 信号持续时间计时器
        uint32_t fresh_timer{0};       // <-- 新增：发弹延迟计算的刷新计时器
        // 核心逻辑变量
        float last_rotor_rad{0}; // 上一次的转子角度
        float total_trig_rad{0}; // 累计的输出轴角度（未归一化）

        // 反馈
        float current_fric_mps[4]{};
        float current_trig_radps{0};
        float current_trig_torque{0};
        float current_trig_rad{0}; // -PI ~ PI (归一化后的输出)

        uint16_t current_heat{0};
        uint16_t current_heat_limit{0};

        // 目标
        float target_fric_mps[4]{};
        float target_trig_rad{0};
        float target_trig_radps{0};

        float current_fric_torque[4]{};
        bool fric_online[4]{false};
        // 输出
        float out_fric_torque[4]{};
        float out_trig_torque{0};
    };

    struct shoot_data_t
    {
        float ball_speed[3]{};
        float fric1_mps = 12.5f;
        float fric2_mps = 8.5f;
    };

    struct booster_ctx_t
    {
        motor_ctx_t motor;
        pid_ctx_t pid;
        data_ctx_t data;
        shoot_data_t shoot_data{};
        quad_booster_cmd_t *cmd{};
    };

    booster_ctx_t _ctx;

    // =====================================================
    // 状态机定义
    // =====================================================
    using owner = quad_booster_t;

    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;

      private:
        bool _trigger_stopped{false}; // 用于确保拨弹盘完全停止后发0
    };

    struct fsm_active_t final : public fsm_t<owner>
    {
        struct state_homing_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;

          private:
            float _homing_turnback_start_time{0.0f};
        };
        struct state_interim_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_ready_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_busy_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_stall_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

      private:
        state_homing_t _homing_state;
        state_interim_t _interim_state;
        state_ready_t _ready_state;
        state_busy_t _busy_state;
        state_stall_t _stall_state;
    };
    state_passive_t _state_passive;
    fsm_active_t _state_active;
    fsm_t<owner> _main_fsm;

    static constexpr float FRIC1_RADIUS = 0.03f;
    static constexpr float FRIC2_RADIUS = 0.03f;
};

} // namespace pyro
#endif