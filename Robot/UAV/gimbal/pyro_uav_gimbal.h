#ifndef PYRO_GIMBAL_TASK_H
#define PYRO_GIMBAL_TASK_H

#include "pyro_algo_pid.h"
#include "pyro_core_fsm.h"
#include "pyro_ins.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "gimbal_config.h"

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
    struct gimbal_ctx_t;
    struct gimbal_auto_ctx_t;
    struct TD_t;
    struct ui_ctx_t;
    struct pitch_ldob_t;
    // 添加 ESO 结构体
    struct eso_t;

public:
    uav_gimbal_t(const uav_gimbal_t &)            = delete;
    uav_gimbal_t &operator=(const uav_gimbal_t &) = delete;

    ins_drv_t *gimbal_ins;
    gimbal_ctx_t* get_data();
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
    static void td_calculate(TD_t *td, float target);

    static void eso_update(eso_t *eso, float y, float u);

    struct TD_t{
        float r;      // 快速因子
        float h;      // 滤波因子
        float dt;     // 周期
        float x1;     // 平滑位置输出
        float x2;     // 平滑速度输出
    };

    // --- 新增 ESO 实例 ---
    // struct eso_t {
    //     float r;          // 这里的 r 对应 ADRC 中的带宽 omega_o
    //     float b0;         // 控制增益
    //     float dt;         // 采样周期
    //     float z1;         // 估计角度
    //     float z2;         // 估计角速度
    //     float z3;         // 估计总扰动 (ADRC 核心)
    // };

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

        float yaw_kff = 0.8f;
        float pitch_kff = 0.7f;

        float gravity_compensate_k = 0.8f;
        float gravity_compensate{};
    };

    struct gimbal_auto_ctx_t
    {
        uint8_t auto_enable{0};
        float shoot_yaw_angle{};
        float shoot_pitch_angle{};

        float kalman_yaw_v{};
        float kalman_pitch_v{};
    };

    struct ui_ctx_t
    {
        bool is_aiming_locked{false};
    };

    struct gimbal_ctx_t
    {
        uav_gimbal_cfg_t cfg;
        TD_t yaw_td;
        TD_t pitch_td;
        data_ctx_t data{};
        ui_ctx_t ui_ctx{};
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
};

}

#endif
