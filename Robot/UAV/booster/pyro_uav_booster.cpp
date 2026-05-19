#include "pyro_uav_booster.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dwt_drv.h"
#include "pyro_referee.h"

namespace pyro
{
uav_booster_t::uav_booster_t() : module_base_t("quad_booster")
{
    booster_ctx = {};
    main_fsm.change_state(&passive_state);
}

status_t uav_booster_t::_init()
{
    booster_ctx.referee_ctx.referee_drv = referee_drv_t::get_instance();

    //摩擦轮电机初始化
    booster_ctx.cfg.motor_cfg.fric_wheel[0] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,can_hub_t::can1);
    booster_ctx.cfg.motor_cfg.fric_wheel[1] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,can_hub_t::can1);
    //拨弹盘电机初始化
    booster_ctx.cfg.motor_cfg.trigger_wheel = new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_4,can_hub_t::can1);

    //摩擦轮pid初始化
    booster_ctx.cfg.pid_cfg.fric_pid[0] = new pid_t(8.0f, 0.0f, 0.007f,1.0f,
       20);
    booster_ctx.cfg.pid_cfg.fric_pid[1] = new pid_t(7.6f, 0.0f, 0.007f,1.0f,
        20);

    //拨弹盘pid初始化
    booster_ctx.cfg.pid_cfg.trigger_position_pid =
        new pid_t(15.0f, 0.0f, 0.005f, 0.0f, 16.0f,100,80,4);
    booster_ctx.cfg.pid_cfg.trigger_speed_pid =
        new pid_t(2.4f, 0.0f, 0.0005f, 0.0f, 10.0f,80,50,4);

    booster_ctx.cfg.pid_cfg.shoot_closed_pid = new pid_t(0.0182f, 0.0f, 0.00004f, 0.0f, 0.5f);
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

void uav_booster_t::_update_feedback()
{
    booster_ctx.referee_ctx.referee_data = booster_ctx.referee_ctx.referee_drv->get_data();

    booster_ctx.shoot_data.now_bullet_speed_mps = booster_ctx.referee_ctx.referee_data.shoot.initial_speed;
    booster_ctx.shoot_data.robot_id             = booster_ctx.referee_ctx.referee_data.robot_status.robot_id;
    booster_ctx.shoot_data.robot_level          = booster_ctx.referee_ctx.referee_data.robot_status.robot_level;
    booster_ctx.shoot_data.Q_max                = booster_ctx.referee_ctx.referee_data.robot_status.shooter_barrel_heat_limit;
    booster_ctx.shoot_data.Q_cd                 = booster_ctx.referee_ctx.referee_data.robot_status.shooter_barrel_cooling_value;
    booster_ctx.shoot_data.launching_frequency  = booster_ctx.referee_ctx.referee_data.shoot.launching_frequency;

    // 更新反馈
    booster_ctx.cfg.motor_cfg.fric_wheel[0]->update_feedback();
    booster_ctx.cfg.motor_cfg.fric_wheel[1]->update_feedback();
    booster_ctx.cfg.motor_cfg.trigger_wheel->update_feedback();

    // 获取摩擦轮转速
    booster_ctx.data_ctx.current_fric_mps[0] = booster_ctx.cfg.motor_cfg.fric_wheel[0]->get_current_rotate() * FRIC1_RADIUS;
    booster_ctx.data_ctx.current_fric_mps[1] = booster_ctx.cfg.motor_cfg.fric_wheel[1]->get_current_rotate() * FRIC1_RADIUS;
    booster_ctx.data_ctx.current_fric_torque = booster_ctx.cfg.motor_cfg.fric_wheel[0]->get_current_torque();

    // 获取拨弹盘转速 位置和扭矩
    booster_ctx.data_ctx.current_trigger_radps =
            booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_rotate() * reciprocal_reduction_ratio;

    booster_ctx.data_ctx.current_trigger_torque =
       booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_torque();

    const float now_motor_rad = booster_ctx.cfg.motor_cfg.trigger_wheel->get_current_position();
    float delta_rotor   = now_motor_rad - booster_ctx.data_ctx.last_motor_rad;
    if (delta_rotor > PI)          delta_rotor -= 2.0f * PI;
    else if (delta_rotor < -PI)    delta_rotor += 2.0f * PI;

    // 锁存上一帧绝对角度
    booster_ctx.data_ctx.last_trigger_rad = booster_ctx.data_ctx.total_trigger_rad;
    // 累积到输出轴总角度
    booster_ctx.data_ctx.total_trigger_rad += delta_rotor * reciprocal_reduction_ratio;

    float current_time_ms = dwt_drv_t::get_timeline_ms();

    heat_control(current_time_ms);

    booster_ctx.data_ctx.last_motor_rad = now_motor_rad;
    booster_ctx.data_ctx.current_trigger_rad = normalize_angle(booster_ctx.data_ctx.total_trigger_rad);
}

void uav_booster_t::speed_control()
{
    if (booster_ctx.shoot_data.last_bullet_speed_mps != booster_ctx.shoot_data.now_bullet_speed_mps)
    {
        float speed_error = booster_ctx.shoot_data.target_bullet_speed - booster_ctx.shoot_data.now_bullet_speed_mps;
        if (abs(speed_error) < 0.35f)
        {
            booster_ctx.shoot_data.speed_increment = 0.0f;
        }
        else
        {
            if (booster_ctx.shoot_data.now_bullet_speed_mps > 19.0f && booster_ctx.shoot_data.now_bullet_speed_mps < 25.0f)
            {
                booster_ctx.shoot_data.ball_speed[2] = booster_ctx.shoot_data.ball_speed[1];
                booster_ctx.shoot_data.ball_speed[1] = booster_ctx.shoot_data.ball_speed[0];
                booster_ctx.shoot_data.ball_speed[0] = booster_ctx.shoot_data.now_bullet_speed_mps;

                for (float & i : booster_ctx.shoot_data.ball_speed)
                {
                    if (std::abs(i) < 1e-6f)
                    {
                        i = booster_ctx.shoot_data.target_bullet_speed;
                    }
                }

                constexpr float w0 = 0.5f;
                constexpr float w1 = 0.3f;
                constexpr float w2 = 0.2f;

                float e0 = booster_ctx.shoot_data.target_bullet_speed - booster_ctx.shoot_data.ball_speed[0];
                float e1 = booster_ctx.shoot_data.target_bullet_speed - booster_ctx.shoot_data.ball_speed[1];
                float e2 = booster_ctx.shoot_data.target_bullet_speed - booster_ctx.shoot_data.ball_speed[2];

                float signed_weighted_mse = (w0 * e0 * std::abs(e0)) +
                                                (w1 * e1 * std::abs(e1)) +
                                                (w2 * e2 * std::abs(e2));

                booster_ctx.shoot_data.speed_increment = booster_ctx.cfg.pid_cfg.shoot_closed_pid->calculate(
                    signed_weighted_mse, 0);
            }
        }

        booster_ctx.shoot_data.fric_mps += booster_ctx.shoot_data.speed_increment;

        constexpr float MAX_SPEED = 23.5f;
        constexpr float MIN_SPEED = 17.5f;
        if (booster_ctx.shoot_data.fric_mps > MAX_SPEED){booster_ctx.shoot_data.fric_mps = MAX_SPEED;}
        if (booster_ctx.shoot_data.fric_mps < MIN_SPEED){booster_ctx.shoot_data.fric_mps = MIN_SPEED;}

        booster_ctx.shoot_data.last_bullet_speed_mps = booster_ctx.shoot_data.now_bullet_speed_mps;
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
    booster_ctx.data_ctx.trigger_output_torque = booster_ctx.cfg.pid_cfg.trigger_speed_pid->calculate
        (booster_ctx.data_ctx.target_trigger_radps,booster_ctx.data_ctx.current_trigger_radps);
}

void uav_booster_t::trigger_speed_control()
{
    booster_ctx.data_ctx.trigger_output_torque = booster_ctx.cfg.pid_cfg.trigger_speed_pid->calculate(
   booster_ctx.data_ctx.target_trigger_radps,booster_ctx.data_ctx.current_trigger_radps);
}

void uav_booster_t::send_fric_command() const
{
    booster_ctx.cfg.motor_cfg.fric_wheel[0]->send_torque(booster_ctx.data_ctx.fric_output_torque[0]);
    booster_ctx.cfg.motor_cfg.fric_wheel[1]->send_torque(booster_ctx.data_ctx.fric_output_torque[1]);
}

void uav_booster_t::send_trigger_command() const
{
    booster_ctx.cfg.motor_cfg.trigger_wheel->send_torque(booster_ctx.data_ctx.trigger_output_torque);
}

uint8_t uav_booster_t::get_robot_id() const
{
    return booster_ctx.referee_ctx.referee_data.robot_status.robot_id;
}

void uav_booster_t::heat_control(float current_time_ms)
{
    // 精准计算dt
    static float last_tick_time = current_time_ms;
    float dt_s = (current_time_ms - last_tick_time) / 1000.0f;
    last_tick_time = current_time_ms;

    // 减去冷却
    if (booster_ctx.shoot_data.Q_cd > 0.0f && booster_ctx.heat_control_ctx.local_heat > 0.0f)
    {
        booster_ctx.heat_control_ctx.local_heat -= booster_ctx.shoot_data.Q_cd * dt_s;
        if (booster_ctx.heat_control_ctx.local_heat < 0.0f)
        {
            booster_ctx.heat_control_ctx.local_heat = 0.0f;
        }
    }

    // 通过拨弹盘角度变化来计算热量
    float delta_rad = booster_ctx.data_ctx.total_trigger_rad - booster_ctx.data_ctx.last_trigger_rad;
    if (delta_rad > 0.01f) // 只累加正向转动的增量
    {
        booster_ctx.data_ctx.accumulated_rad += delta_rad;
    }

    // 触发物理发射判定
    if (booster_ctx.data_ctx.accumulated_rad >= RAD_PER_SHOT)
    {
        booster_ctx.data_ctx.accumulated_rad -= RAD_PER_SHOT;

        // 打出一发，本地热量直接 +10 点
        booster_ctx.heat_control_ctx.local_heat += 10.0f;

        // 只要在射击，就刷新最后物理开火时间戳
        booster_ctx.heat_control_ctx.last_shot_time_ms = current_time_ms;
    }

    // 200ms 时间窗状态观测同步 超过200ms没开火就通过裁判系统复位一下 防止出问题导致热量不准确
    booster_ctx.shoot_data.Q_now_referee = booster_ctx.referee_ctx.referee_data.power_heat.shooter_17mm_barrel_heat;
    float ref_heat = booster_ctx.shoot_data.Q_now_referee;

    float time_since_last_shot = current_time_ms - booster_ctx.heat_control_ctx.last_shot_time_ms;

    if (time_since_last_shot > 200)
    {
        // 超过 200ms，直接覆写纠正本地累积漂移误差
        booster_ctx.heat_control_ctx.local_heat = ref_heat;
    }
    else
    {
        // 正在高频交火连发，或者自瞄信号跳变停射未满 200ms，强行用本地高精度预测顶住
        booster_ctx.heat_control_ctx.local_heat = std::max(booster_ctx.heat_control_ctx.local_heat, ref_heat);
    }

    // 4. 将最终安全融合后的 local_heat 转换为状态机可用的可打弹数
    // 预留两发子弹，防止超频爆热量
    float safe_q_res = booster_ctx.shoot_data.Q_max - booster_ctx.heat_control_ctx.local_heat - 20.0f;

    if (safe_q_res <= 0.0f)
    {
        booster_ctx.heat_control_ctx.allow_bullet_count = 0;
    }
    else
    {
        booster_ctx.heat_control_ctx.allow_bullet_count = static_cast<int16_t>(safe_q_res / 10.0f);
    }

}

uav_booster_t::booster_ctx_t* uav_booster_t::get_data()
{
    return &booster_ctx;
}

} // namespace pyro