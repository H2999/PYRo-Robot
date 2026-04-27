#ifndef PYRO_GIMBAL_TASK_H
#define PYRO_GIMBAL_TASK_H

#include "pyro_algo_pid.h"
#include "pyro_core_fsm.h"
#include "pyro_ins.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"

namespace pyro
{

//命令定义
struct uav_gimbal_cmd_t final : cmd_base_t
{
    float yaw_delta_angle;      //yaw轴目标角度
    float pitch_delta_angle;    //pitch轴目标角度

    uint8_t auto_flag{};
    float pitch_target_angle{};
    float yaw_target_angle{};

    uav_gimbal_cmd_t()
    :yaw_delta_angle() , pitch_delta_angle(0)
    {
    }
};

//仅初始化一次的变量
struct uav_gimbal_cfg_t
{
    struct motor_ctx_t
    {
        motor_base_t *yaw_motor{nullptr};
        motor_base_t *pitch_motor{nullptr};
        motor_base_t *roll_motor{nullptr};
    };

    struct pid_ctx_t
    {
        //遥控器pid
        pid_t *yaw_position_pid{nullptr};
        pid_t *pitch_position_pid{nullptr};
        pid_t *yaw_speed_pid{nullptr};
        pid_t *pitch_speed_pid{nullptr};

        //自瞄pid
        pid_t *auto_yaw_position_pid{nullptr};
        pid_t *auto_pitch_position_pid{nullptr};
        pid_t *auto_yaw_speed_pid{nullptr};
        pid_t *auto_pitch_speed_pid{nullptr};
    };

    motor_ctx_t motor_ctx;
    pid_ctx_t pid_ctx;
};

//云台类定义
class uav_gimbal_t final : public module_base_t<uav_gimbal_t,uav_gimbal_cmd_t,uav_gimbal_cfg_t>
{
    friend class module_base_t;
    friend class jcom_drv_t;

    struct data_ctx_t;
    struct feedforward_data_ctx_t;
    struct gimbal_ctx_t;
    struct gimbal_auto_ctx_t;

public:
    uav_gimbal_t(const uav_gimbal_t &)            = delete;
    uav_gimbal_t &operator=(const uav_gimbal_t &) = delete;

    ins_drv_t *gimbal_ins;
private:
    uav_gimbal_t();
    ~uav_gimbal_t() override = default;

    //基类接口
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    //派生方法
    static void rc_gimbal_control(gimbal_ctx_t *ctx);
    static void auto_aim_gimbal_control(gimbal_ctx_t *ctx);
    static void send_motor_command(const gimbal_ctx_t *ctx);
    static void normalize_angle(float& angle);
    void feedforward_compensation();

    struct feedforward_data_ctx_t
    {
        float last_yaw_target_angle{};
        float now_yaw_target_angle{};

        float last_pitch_target_angle{};
        float now_pitch_target_angle{};

        float dt = 0.001f;
        float yaw_kff = 0.2f;
        float pitch_kff = 0.2f;

        float filtered_yaw_v;
        float filtered_pitch_v;
        float yaw_ff{};
        float pitch_ff{};
    };

    struct data_ctx_t
    {
        float pitch_real_min_limit_angle_filtered;
        float pitch_real_max_limit_angle_filtered;
        //目标角度
        float _target_yaw_angle{};
        float _target_pitch_angle{};
        //目标速度
        float _target_yaw_speed{};
        float _target_pitch_speed{};

        //当前角度
        float _current_imu_yaw_angle{};
        float _current_imu_pitch_angle{};
        float _current_imu_roll_angle{};
        // 当前速度
        float _current_imu_pitch_speed{};
        float _current_imu_yaw_speed{};
        float _current_imu_roll_speed{};

        // 输出扭矩
        float _output_yaw_torque{};
        float _output_pitch_torque{};

        float pitch_motor_angle{};
        float pitch_motor_speed{};
        float yaw_motor_angle{};

        float yaw_real_max_limit_angle{};
        float yaw_real_min_limit_angle{};

        float pitch_real_max_limit_angle{};
        float pitch_real_min_limit_angle{};

        float gravity_k{};
        float gravity_compensate{};
    };

    struct gimbal_auto_ctx_t
    {
        uint8_t auto_enable{0};
        float shoot_yaw_angle{};
        float shoot_pitch_angle{};
    };

    struct gimbal_ctx_t
    {
        uav_gimbal_cfg_t cfg;
        feedforward_data_ctx_t feedforward_data;
        data_ctx_t data{};
        uav_gimbal_cmd_t *cmd{};
        gimbal_auto_ctx_t auto_ctx{};
    };

    gimbal_ctx_t gimbal_ctx;

    struct state_passive_t final : state_t<uav_gimbal_t>
    {
        void enter(uav_gimbal_t *owner) override;
        void execute(uav_gimbal_t *owner) override;
        void exit(uav_gimbal_t *owner) override;
    };

    struct fsm_active_t final : fsm_t<uav_gimbal_t>
    {
        struct state_rc_t final : state_t<uav_gimbal_t>
        {
            void enter(uav_gimbal_t *owner) override;
            void execute(uav_gimbal_t *owner) override;
            void exit(uav_gimbal_t *owner) override;
        };

        struct state_auto_t final : state_t<uav_gimbal_t>
        {
            void enter(uav_gimbal_t *owner) override;
            void execute(uav_gimbal_t *owner) override;
            void exit(uav_gimbal_t *owner) override;
        private:
            float _last_filtered_yaw = 0.0f;
            float _last_filtered_pitch = 0.0f;
            const float _alpha = 0.3f;
        };

        void on_enter(uav_gimbal_t *owner) override;
        void on_execute(uav_gimbal_t *owner) override;
        void on_exit(uav_gimbal_t *owner) override;

    private:
        state_rc_t rc_state;
        state_auto_t auto_state;
    };

    // 状态实例
    state_passive_t state_passive;
    fsm_active_t state_active;
    fsm_t<uav_gimbal_t> main_fsm;

    static constexpr float yaw_motor_max_value = 1.5f;
    static constexpr float yaw_motor_min_value = -1.1f;

    // static constexpr float pitch_motor_max_value = 0.35f;
    // static constexpr float pitch_motor_min_value = -0.45f;
    static constexpr float pitch_max_value = 0.5f;
    static constexpr float pitch_min_value = -0.2f;

    static constexpr float YAW_OFFSET_RAD = 2.60700035f;
    static constexpr float PITCH_OFFSET_RAD = -0.2f;

    // typedef struct {
    //     float r;      // 快速因子：决定追踪的加速度（r 越大，起步越猛）
    //     float h;      // 滤波因子：决定平滑程度（h 越大，越不抖，通常设为 3~10 倍 dt）
    //     float dt;     // 运行周期：必须等于你调用此函数的真实频率（如 1ms = 0.001f）
    //
    //     float x1;     // 状态量1：跟踪出的平滑位置（我们要的影子指令）
    //     float x2;     // 状态量2：跟踪出的平滑速度（可用于前馈控制）
    // } TD_t;
    //
    // // 内部使用的符号函数
    // static float sgn(float x) {
    //     return (x > 0) - (x < 0);
    // }

    // void TD_Calculate(TD_t *td, float target) {
    //     // 1. 计算偏差
    //     float x1_err = td->x1 - target;
    //
    //     // 2. fhan 公式中间变量计算
    //     float d = td->r * td->h * td->h;
    //     float a0 = td->h * td->x2;
    //     float y = x1_err + a0;
    //
    //     float a1 = sqrtf(d * (d + 8.0f * fabsf(y)));
    //     float a2 = a0 + sgn(y) * (a1 - d) * 0.5f;
    //
    //     // 3. 线性区判定
    //     float sy = (sgn(y + d) - sgn(y - d)) * 0.5f;
    //     float a = (a0 + y - a2) * sy + a2;
    //
    //     float sa = (sgn(a + d) - sgn(a - d)) * 0.5f;
    //     float fh = -td->r * ((a / d - sgn(a)) * sa + sgn(a));
    //
    //     // 4. 状态更新 (积分)
    //     td->x1 += td->dt * td->x2;
    //     td->x2 += td->dt * fh;
    // }
};

}

#endif
