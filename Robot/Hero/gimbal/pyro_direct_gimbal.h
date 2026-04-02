#ifndef __PYRO_DIRECT_GIMBAL_H__
#define __PYRO_DIRECT_GIMBAL_H__

#include "pyro_algo_pid.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h" // 新增 DM 电机驱动
#include "pyro_module_base.h"
#include "pyro_motor_base.h"

namespace pyro
{

// =========================================================
// 1. 命令定义
// =========================================================
struct direct_gimbal_cmd_t final : public cmd_base_t
{
    float pitch_delta_angle; // 目标 Pitch 角度 (rad)
    float yaw_delta_angle;   // 目标 Yaw 角度 (rad)

    // 自瞄数据
    bool auto_aim;
    float target_pitch;
    float target_yaw;

    direct_gimbal_cmd_t() : pitch_delta_angle(0.0f), yaw_delta_angle(0.0f),auto_aim(false),target_pitch(0.0f),target_yaw(0.0f)
    {
    }
};

struct direct_gimbal_config_t
{
};

// =========================================================
// 2. 云台类
// =========================================================
class direct_gimbal_t final
    : public module_base_t<direct_gimbal_t, direct_gimbal_cmd_t,
                           direct_gimbal_config_t>
{
    friend class module_base_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct gimbal_context_t;

  public:
    [[nodiscard]] const gimbal_context_t& get_ctx() const;

  private:
    direct_gimbal_t();
    ~direct_gimbal_t() override = default;

    // --- 基类接口实现 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 私有辅助方法 ---
    static void _gimbal_control(gimbal_context_t *ctx);
    static void _send_motor_command(gimbal_context_t *ctx);

    // --- 成员变量 ---

    // 电机句柄
    struct motor_ctx_t
    {
        motor_base_t *pitch{nullptr};
        motor_base_t *yaw{nullptr};
    };

    // 算法对象 (串级 PID)
    struct pid_ctx_t
    {
        pid_t *pitch_pos{nullptr};
        pid_t *pitch_spd{nullptr};
        pid_t *yaw_pos{nullptr};
        pid_t *yaw_spd{nullptr};
    };

    // 运行时数据
    struct data_ctx_t
    {
        // 反馈 (含 Offset)
        float current_pitch_rad{0};
        float current_pitch_radps{0};
        float current_yaw_rad{0};
        float current_yaw_radps{0};
        float current_z_accel{0};

        // 暂未使用
        float current_roll_rad{0};
        float current_roll_radps{0};
        float current_x_accel{0};
        float current_y_accel{0};

        // 目标
        float target_pitch_rad{0};
        float target_pitch_radps{0};
        float target_yaw_rad{0};
        float target_yaw_radps{0};

        // 输出
        float out_pitch_torque{0};
        float out_gravity_torque{0};
        float out_yaw_torque{0};
    };

    // 总 Context
    struct gimbal_context_t
    {
        motor_ctx_t motor;
        pid_ctx_t pid;
        data_ctx_t data;
        direct_gimbal_cmd_t *cmd;
    };

    gimbal_context_t _ctx;

    // =====================================================
    // 状态定义 (HFSM)
    // =====================================================
    using owner = direct_gimbal_t;

    struct state_passive_t : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct state_active_t : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    state_passive_t _state_passive;
    state_active_t _state_active;
    fsm_t<owner> _main_fsm;

    // =====================================================
    // 静态配置 (编译器常量)
    // =====================================================
    static constexpr float PITCH_OFFSET_RAD = 0.0f;
    static constexpr float YAW_OFFSET_RAD   = 0.0f;

    static constexpr float PITCH_MAX_RAD    = 0.2f;  // 57.3 deg
    static constexpr float PITCH_MIN_RAD    = -0.6f; // -57.
};

} // namespace pyro

#endif