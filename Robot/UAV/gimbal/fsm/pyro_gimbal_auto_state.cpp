#include "pyro_uav_gimbal.h"
#include "forecast_v.h"
#include "pyro_autoaim_drv.h"

using namespace pyro;
extern autoaim_drv_t::rx_data_t rx_data;

void uav_gimbal_t::fsm_active_t::state_auto_t::enter(uav_gimbal_t *owner)
{
    //由于基于x2的存在 从遥控器模式切换到自瞄模式的瞬间一定要把所有前馈清零 目标角度等于当前角度 否则会抽动一下

    // 初始化目标角度为当前IMU角度，避免切换时抖动
    // owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data._current_imu_yaw_angle;
    // owner->gimbal_ctx.data._target_pitch_angle = owner->gimbal_ctx.data._current_imu_pitch_angle;

    // 必须同步初始化 TD，否则 TD 会基于旧的 x1 算出错误的速度前馈
    owner->gimbal_ctx.yaw_td.x1 = owner->gimbal_ctx.data._current_imu_yaw_angle;
    owner->gimbal_ctx.yaw_td.x2 = 0.0f;
    owner->gimbal_ctx.pitch_td.x1 = owner->gimbal_ctx.data._current_imu_pitch_angle;
    owner->gimbal_ctx.pitch_td.x2 = 0.0f;

    // 清除可能存在的卡尔曼速度残余
    owner->gimbal_ctx.auto_ctx.kalman_yaw_v = 0.0f;
    owner->gimbal_ctx.auto_ctx.kalman_pitch_v = 0.0f;

    //清除内部的积分项
    // owner->gimbal_ctx.cfg.pid_ctx.auto_yaw_position_pid->clear();
    // owner->gimbal_ctx.cfg.pid_ctx.auto_yaw_speed_pid->clear();
    // owner->gimbal_ctx.cfg.pid_ctx.auto_pitch_position_pid->clear();
    // owner->gimbal_ctx.cfg.pid_ctx.auto_pitch_speed_pid->clear();
}

void uav_gimbal_t::fsm_active_t::state_auto_t::execute(uav_gimbal_t *owner)
{
    //没识别到目标的时候用遥控器控制 识别到目标的时候由自瞄控制
    if (owner->gimbal_ctx.cmd->yaw_target_angle > 80.0f || owner->gimbal_ctx.cmd->yaw_target_angle == 0.0f)
    {
        owner->gimbal_ctx.data._target_yaw_angle += owner->gimbal_ctx.cmd->yaw_delta_angle;
    }
    else
    {
        owner->gimbal_ctx.ui_ctx.is_aiming_locked = true;
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.cmd->yaw_target_angle;

        owner->gimbal_ctx.auto_ctx.kalman_yaw_v = rx_data.yaw_omega;
        owner->gimbal_ctx.auto_ctx.kalman_yaw_v = std::clamp(owner->gimbal_ctx.auto_ctx.kalman_yaw_v,-5.0f,5.0f);
    }
    //限位
    if (owner->gimbal_ctx.data._target_yaw_angle > owner->gimbal_ctx.data.yaw_real_max_limit_angle)
    {
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data.yaw_real_max_limit_angle;
    }
    if (owner->gimbal_ctx.data._target_yaw_angle < owner->gimbal_ctx.data.yaw_real_min_limit_angle)
    {
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data.yaw_real_min_limit_angle;
    }

    if (owner->gimbal_ctx.cmd->pitch_target_angle > 80.0f || owner->gimbal_ctx.cmd->pitch_target_angle == 0.0f)
    {
        owner->gimbal_ctx.data._target_pitch_angle += owner->gimbal_ctx.cmd->pitch_delta_angle;
    }
    else
    {
        //坐标系不同 所以加个负号
        owner->gimbal_ctx.data._target_pitch_angle = - owner->gimbal_ctx.cmd->pitch_target_angle;

        owner->gimbal_ctx.auto_ctx.kalman_pitch_v = rx_data.pitch_omega;
        owner->gimbal_ctx.auto_ctx.kalman_pitch_v = std::clamp(owner->gimbal_ctx.auto_ctx.kalman_pitch_v,-4.5f,4.5f);
    }

    if (owner->gimbal_ctx.data._target_pitch_angle > pitch_max_value)
    {
        owner->gimbal_ctx.data._target_pitch_angle = pitch_max_value;
    }
    else if (owner->gimbal_ctx.data._target_pitch_angle < pitch_min_value)
    {
        owner->gimbal_ctx.data._target_pitch_angle = pitch_min_value;
    }

    // auto_aim_gimbal_control(&owner->gimbal_ctx);
    auto_aim_gimbal_control_leso(&owner->gimbal_ctx);
    send_motor_command(&owner->gimbal_ctx);
}

void uav_gimbal_t::fsm_active_t::state_auto_t::exit(uav_gimbal_t *owner)
{

}