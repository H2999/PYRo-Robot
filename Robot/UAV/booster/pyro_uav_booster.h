#ifndef PYRO_UAV_BOOSTER_H
#define PYRO_UAV_BOOSTER_H

#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"

namespace pyro
{

//命令模板
struct uav_booster_cmd_t final : public cmd_base_t
{
    bool trigger_enable;        // 拨弹开启
    float target_fric1_mps;   // 第一级摩擦轮目标转速
    float target_fric2_mps;   // 第二级摩擦轮目标转速

    float target_trigger_radps;
    bool single_mode;
    bool continue_mode;

    uint8_t booster_auto_flag;

    uav_booster_cmd_t()
        :trigger_enable(false), target_fric1_mps(0), target_fric2_mps(0),target_trigger_radps(0),
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

public:
    uav_booster_t(const uav_booster_t &) = delete;
    uav_booster_t & operator = (const uav_booster_t &) = delete;

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
    void send_trigger_command();
    static float normalize_angle(float angle);

    struct data_ctx_t
    {
        float last_rotor_rad{0};
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

    struct booster_ctx_t
    {
        uav_booster_cfg_t cfg;
        data_ctx_t data_ctx;
        uav_booster_cmd_t *cmd{};
        booster_auto_ctx_t auto_ctx{};
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
        struct state_interim_t final : public state_t<uav_booster_t>
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
        state_interim_t interim_state;
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
};

};

#endif // PYRO_UAV_BOOSTER_H
