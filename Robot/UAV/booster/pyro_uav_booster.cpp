#include "pyro_uav_booster.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_uav_booster.h"

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
    //摩擦轮电机初始化
    booster_ctx.cfg.motor_cfg.fric_wheel[0] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,can_hub_t::can2);
    booster_ctx.cfg.motor_cfg.fric_wheel[1] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,can_hub_t::can2);
    //拨弹盘电机初始化
    booster_ctx.cfg.motor_cfg.trigger_wheel = new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_4,can_hub_t::can2);

    //摩擦轮pid初始化
    booster_ctx.cfg.pid_cfg.fric_pid[0] = new pid_t(6.40f, 0.02f, 0.02f,2.5f,
        20, 320, 80, 4);
    booster_ctx.cfg.pid_cfg.fric_pid[1] = new pid_t(6.40f, 0.02f, 0.02f,2.5f,
        20, 320, 80, 4);
    //拨弹盘pid初始化
    booster_ctx.cfg.pid_cfg.trigger_position_pid =
        new pid_t(8.4f, 0.05, 0, 1.0f, 10.0f, 100, 80, 4);
    booster_ctx.cfg.pid_cfg.trigger_speed_pid =
        new pid_t(6.8f, 0.05, 0, 1.0, 15.0f, 100, 80, 4);

    //弹速闭环
    // booster_ctx.cfg.pid_cfg.shoot_closed_pid = new pid_t(4.0f, 0.05, 0, 1.0f, 10.0f);
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

void uav_booster_t::_update_feedback()
{
    booster_ctx.auto_ctx.fire_enable = booster_ctx.cmd->booster_auto_flag;

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

    booster_ctx.data_ctx.current_trigger_rad = booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_position();

    booster_ctx.data_ctx.current_trigger_torque =
       booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_torque();

    const float now_rotor_rad = booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_position();
    float delta_rotor   = now_rotor_rad - booster_ctx.data_ctx.last_rotor_rad;
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

    booster_ctx.data_ctx.last_rotor_rad  = now_rotor_rad;

    booster_ctx.data_ctx.current_trigger_rad = normalize_angle(booster_ctx.data_ctx.total_trigger_rad);

    // const float now_rotor_rad = booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_position();
    // const float now_rotor_torque = booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_torque();
    //
    // float delta_rotor = now_rotor_rad - booster_ctx.data_ctx.last_rotor_rad;
    // if (delta_rotor > PI)
    // {
    //     delta_rotor -= 2.0f * PI;
    // }
    // else if (delta_rotor < -PI)
    // {
    //     delta_rotor += 2.0f * PI;
    // }
    //
    // booster_ctx.data_ctx.total_trigger_rad += delta_rotor * reciprocal_ratio;
    //
    // if (fabsf(booster_ctx.data_ctx.total_trigger_rad) > 2.0f * PI) {
    //     booster_ctx.data_ctx.total_trigger_rad = fmodf(booster_ctx.data_ctx.total_trigger_rad, 2.0f * PI);
    // }
    //
    // booster_ctx.data_ctx.current_trigger_radps = now_rotor_rad * reciprocal_ratio;
    // booster_ctx.data_ctx.current_trigger_rad = normalize_angle(booster_ctx.data_ctx.total_trigger_rad);
    // booster_ctx.data_ctx.current_trigger_torque = now_rotor_torque;
    // booster_ctx.data_ctx.last_rotor_rad = now_rotor_rad;
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
    // booster_ctx.cfg.motor_cfg.fric_wheel[1]->send_torque(booster_ctx.data_ctx.fric_output_torque[1]);

    // booster_ctx.cfg.motor_cfg.fric_wheel[0]->send_torque(0);
    // booster_ctx.cfg.motor_cfg.fric_wheel[1]->send_torque(0);
}

void uav_booster_t::send_trigger_command()
{
    // booster_ctx.cfg.motor_cfg.trigger_wheel->send_torque(booster_ctx.data_ctx.trigger_output_torque);
    booster_ctx.cfg.motor_cfg.trigger_wheel->send_torque(0);
}

} // namespace pyro