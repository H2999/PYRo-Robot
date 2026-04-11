#include "pyro_uav_booster.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_referee.h"

namespace pyro
{
uav_booster_t::uav_booster_t() : module_base_t("quad_booster")
{
    booster_ctx = {};
    main_fsm.change_state(&passive_state);

    booster_ctx.auto_ctx.fire_enable = 0;
}

status_t uav_booster_t::_init()
{
    booster_ctx.referee_ctx.referee_drv = referee_drv_t::get_instance();

    //摩擦轮电机初始化
    booster_ctx.cfg.motor_cfg.fric_wheel[0] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,can_hub_t::can2);
    booster_ctx.cfg.motor_cfg.fric_wheel[1] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,can_hub_t::can2);
    //拨弹盘电机初始化
    booster_ctx.cfg.motor_cfg.trigger_wheel = new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_4,can_hub_t::can2);

    //摩擦轮pid初始化
    booster_ctx.cfg.pid_cfg.fric_pid[0] = new pid_t(7.40f, 0.02f, 0.02f,2.5f,
        20, 320, 80, 4);
    booster_ctx.cfg.pid_cfg.fric_pid[1] = new pid_t(7.40f, 0.02f, 0.02f,2.5f,
        20, 320, 80, 4);
    //拨弹盘pid初始化
    booster_ctx.cfg.pid_cfg.trigger_position_pid =
        new pid_t(8.4f, 0.05, 0, 1.0f, 10.0f, 100, 80, 4);
    booster_ctx.cfg.pid_cfg.trigger_speed_pid =
        new pid_t(7.8f, 0.05, 0, 1.0, 15.0f, 100, 80, 4);

    booster_ctx.cfg.pid_cfg.shoot_closed_pid = new pid_t(3.0f, 0.25, 0, 1.0f, 5.0f);
    return PYRO_OK;
}

float uav_booster_t::normalize_angle(float angle)
{
    while (angle > PI)
        angle -= 2.0f * PI;
    while (angle < -PI)
        angle += 2.0f * PI;
    return angle;
}

#if test_speed
/**
* @brief 二阶IIR低通滤波器封装
* @return 滤波后的平滑速度
*/
void uav_booster_t::speed_filter()
{
    booster_ctx.IIR_Filter_ctx[0].f[0] = booster_ctx.IIR_Filter_ctx[0].f[1];
    booster_ctx.IIR_Filter_ctx[0].f[1] = booster_ctx.IIR_Filter_ctx[0].f[2];

    booster_ctx.IIR_Filter_ctx[0].f[2] = booster_ctx.IIR_Filter_ctx[0].f[1] * filter_num[0] +
                 booster_ctx.IIR_Filter_ctx[0].f[0] * filter_num[1] +
                 booster_ctx.data_ctx.current_fric_mps[0] * filter_num[2];
    booster_ctx.data_ctx.current_fric_mps[0] = booster_ctx.IIR_Filter_ctx[0].f[2];


    booster_ctx.IIR_Filter_ctx[1].f[0] = booster_ctx.IIR_Filter_ctx[1].f[1];
    booster_ctx.IIR_Filter_ctx[1].f[1] = booster_ctx.IIR_Filter_ctx[1].f[2];

    booster_ctx.IIR_Filter_ctx[1].f[2] = booster_ctx.IIR_Filter_ctx[1].f[1] * filter_num[0] +
                 booster_ctx.IIR_Filter_ctx[1].f[0] * filter_num[1] +
                 booster_ctx.data_ctx.current_fric_mps[1] * filter_num[2];
    booster_ctx.data_ctx.current_fric_mps[1] = booster_ctx.IIR_Filter_ctx[1].f[2];

}
#endif

void uav_booster_t::_update_feedback()
{
    //检测是否进自瞄模式
    booster_ctx.auto_ctx.fire_enable = booster_ctx.cmd->booster_auto_flag;

    //通过裁判系统获取当前弹速
    booster_ctx.referee_ctx.referee_data = booster_ctx.referee_ctx.referee_drv->get_data();

    booster_ctx.shoot_data.last_bullet_speed_mps = booster_ctx.shoot_data.now_bullet_speed_mps;
    booster_ctx.shoot_data.now_bullet_speed_mps = booster_ctx.referee_ctx.referee_data.shoot.initial_speed;

    //更新反馈
    booster_ctx.cfg.motor_cfg.fric_wheel[0]->update_feedback();
    booster_ctx.cfg.motor_cfg.fric_wheel[1]->update_feedback();
    booster_ctx.cfg.motor_cfg.trigger_wheel->update_feedback();

    //获取摩擦轮转速
    booster_ctx.data_ctx.current_fric_mps[0] = booster_ctx.cfg.motor_cfg.fric_wheel[0]->get_current_rotate() * FRIC1_RADIUS;
    booster_ctx.data_ctx.current_fric_mps[1] = booster_ctx.cfg.motor_cfg.fric_wheel[1]->get_current_rotate() * FRIC1_RADIUS;

    //获取拨弹盘转速 位置和扭矩
    booster_ctx.data_ctx.current_trigger_radps =
            booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_rotate() * reciprocal_reduction_ratio;

    booster_ctx.data_ctx.current_trigger_torque =
       booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_torque();

    const float now_motor_rad = booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_position();
    float delta_rotor   = now_motor_rad - booster_ctx.data_ctx.last_motor_rad;
    if (delta_rotor > PI)
    {
        delta_rotor -= 2.0f * PI;
    }
    else if (delta_rotor < -PI)
    {
        delta_rotor += 2.0f * PI;
    }
    // 累积到输出轴总角度
    booster_ctx.data_ctx.total_trigger_rad += delta_rotor * reciprocal_reduction_ratio;

    booster_ctx.data_ctx.last_motor_rad  = now_motor_rad;

    booster_ctx.data_ctx.current_trigger_rad = normalize_angle(booster_ctx.data_ctx.total_trigger_rad);

    /*
    const float now_rotor_rad = booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_position();
    const float now_rotor_torque = booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_torque();

    float delta_rotor = now_rotor_rad - booster_ctx.data_ctx.last_rotor_rad;
    if (delta_rotor > PI)
    {
        delta_rotor -= 2.0f * PI;
    }
    else if (delta_rotor < -PI)
    {
        delta_rotor += 2.0f * PI;
    }

    booster_ctx.data_ctx.total_trigger_rad += delta_rotor * reciprocal_ratio;

    if (fabsf(booster_ctx.data_ctx.total_trigger_rad) > 2.0f * PI) {
        booster_ctx.data_ctx.total_trigger_rad = fmodf(booster_ctx.data_ctx.total_trigger_rad, 2.0f * PI);
    }

    booster_ctx.data_ctx.current_trigger_radps = now_rotor_rad * reciprocal_ratio;
    booster_ctx.data_ctx.current_trigger_rad = normalize_angle(booster_ctx.data_ctx.total_trigger_rad);
    booster_ctx.data_ctx.current_trigger_torque = now_rotor_torque;
    booster_ctx.data_ctx.last_rotor_rad = now_rotor_rad;
    */
}

void uav_booster_t::speed_control()
{
    if (booster_ctx.shoot_data.last_bullet_speed_mps != booster_ctx.shoot_data.now_bullet_speed_mps)
    {
        booster_ctx.shoot_data.ball_speed[2] = booster_ctx.shoot_data.ball_speed[1];
        booster_ctx.shoot_data.ball_speed[1] = booster_ctx.shoot_data.ball_speed[0];
        booster_ctx.shoot_data.ball_speed[0] = booster_ctx.shoot_data.now_bullet_speed_mps;

        for (int i = 0; i < 3; i++)
        {
            if (booster_ctx.shoot_data.ball_speed[i] == 0.0f)
            {
                booster_ctx.shoot_data.ball_speed[i] = booster_ctx.cmd->target_fric_mps;
            }
        }

        if (booster_ctx.cmd->target_fric_mps > 18.0f)
        {
            constexpr float w0 = 0.65f; // 最新一发
            constexpr float w1 = 0.20f; // 上一发
            constexpr float w2 = 0.15f; // 上上发

            const float e0 = booster_ctx.shoot_data.ball_speed[0] - booster_ctx.cmd->target_fric_mps;
            const float e1 = booster_ctx.shoot_data.ball_speed[1] - booster_ctx.cmd->target_fric_mps;
            const float e2 = booster_ctx.shoot_data.ball_speed[2] - booster_ctx.cmd->target_fric_mps;

            float signed_weighted_mse = (w0 * e0 * std::abs(e0)) + (w1 * e1 * std::abs(e1)) +
                                        (w2 * e2 * std::abs(e2));

            float speed_increment =
                booster_ctx.cfg.pid_cfg.shoot_closed_pid->calculate(0.0f, signed_weighted_mse);

            booster_ctx.shoot_data.fric_mps += speed_increment;

            constexpr float MAX_FRIC_MPS = 25.0f;
            constexpr float MIN_FRIC_MPS = 18.0f;

            if (booster_ctx.shoot_data.fric_mps > MAX_FRIC_MPS)
            {
                booster_ctx.shoot_data.fric_mps = MAX_FRIC_MPS;
            }
            else if (booster_ctx.shoot_data.fric_mps < MIN_FRIC_MPS)
            {
                booster_ctx.shoot_data.fric_mps = MIN_FRIC_MPS;
            }
        }
    }
}

void uav_booster_t::_fsm_execute()
{
    booster_ctx.cmd = &_current_cmd;

    if (booster_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
        main_fsm.change_state(&active_state);
    else
        main_fsm.change_state(&passive_state);

    main_fsm.execute(this);
}

void uav_booster_t::fric_control()
{
    booster_ctx.data_ctx.fric_output_torque[0] = booster_ctx.cfg.pid_cfg.fric_pid[0]->calculate
        (booster_ctx.data_ctx.target_fric_mps[0], booster_ctx.data_ctx.current_fric_mps[0]);

    booster_ctx.data_ctx.fric_output_torque[1] = booster_ctx.cfg.pid_cfg.fric_pid[1]->calculate
        (booster_ctx.data_ctx.target_fric_mps[1],booster_ctx.data_ctx.current_fric_mps[1]);
}

void uav_booster_t::trigger_position_control()
{
    //处理过零点问题
    const float error = booster_ctx.data_ctx.target_trigger_rad - booster_ctx.data_ctx.current_trigger_rad;
    if (error > PI)
    {
        booster_ctx.data_ctx.target_trigger_rad -= 2.0f * PI;
    }
    else if (error < -PI)
    {
        booster_ctx.data_ctx.target_trigger_rad += 2.0f * PI;
    }

    booster_ctx.data_ctx.target_trigger_radps = booster_ctx.cfg.pid_cfg.trigger_position_pid->calculate
        (booster_ctx.data_ctx.target_trigger_rad, booster_ctx.data_ctx.current_trigger_rad);

    booster_ctx.data_ctx.trigger_output_torque = booster_ctx.cfg.pid_cfg.trigger_speed_pid->calculate(
   booster_ctx.data_ctx.target_trigger_radps,booster_ctx.data_ctx.current_trigger_radps);
}

void uav_booster_t::trigger_speed_control()
{
    booster_ctx.data_ctx.trigger_output_torque = booster_ctx.cfg.pid_cfg.trigger_speed_pid->calculate(
   booster_ctx.data_ctx.target_trigger_radps,booster_ctx.data_ctx.current_trigger_radps);
}

void uav_booster_t::send_fric_command()
{
    booster_ctx.cfg.motor_cfg.fric_wheel[0]->send_torque(booster_ctx.data_ctx.fric_output_torque[0]);
    booster_ctx.cfg.motor_cfg.fric_wheel[1]->send_torque(booster_ctx.data_ctx.fric_output_torque[1]);
}

void uav_booster_t::send_trigger_command()
{
    booster_ctx.cfg.motor_cfg.trigger_wheel->send_torque(booster_ctx.data_ctx.trigger_output_torque);
}

} // namespace pyro