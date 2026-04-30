#include "pyro_uav_gimbal.h"
#include "forecast_v.h"

using namespace pyro;

void uav_gimbal_t::fsm_active_t::state_auto_t::enter(uav_gimbal_t *owner)
{
    // 初始化目标角度为当前IMU角度，避免切换时抖动
    owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data._current_imu_yaw_angle;
    owner->gimbal_ctx.data._target_pitch_angle = owner->gimbal_ctx.data._current_imu_pitch_angle;

    _last_filtered_yaw = owner->gimbal_ctx.data._current_imu_yaw_angle;
    _last_filtered_pitch = owner->gimbal_ctx.data._current_imu_pitch_angle;
}

void uav_gimbal_t::fsm_active_t::state_auto_t::execute(uav_gimbal_t *owner)
{
//     //用卡尔曼滤波拟合自瞄发来的角度来预测角速度  用于前馈来提高跟踪目标的速度
//     auto *kf = new feedforward_kf_t();
//     kf->kf_predict();
//     kf->kf_update(owner->gimbal_ctx.cmd->yaw_target_angle);
//     const float ff_angle = kf->get_v();
//     owner->gimbal_ctx.data._target_yaw_speed += ff_angle * 0.5f;

    owner->feedforward_compensation();

    float raw_yaw = owner->gimbal_ctx.cmd->yaw_target_angle;
    float filtered_yaw = _alpha * raw_yaw + (1.0f - _alpha) * _last_filtered_yaw;

    float raw_pitch = owner->gimbal_ctx.cmd->pitch_target_angle;
    float filtered_pitch = _alpha * raw_pitch + (1.0f - _alpha) * _last_filtered_pitch;


    //加个延迟看看能不能跟上目标
    float predict_time = 0.01f;
    //没识别到目标的时候用遥控器控制 识别到目标的时候由自瞄控制
    if (owner->gimbal_ctx.cmd->yaw_target_angle > 80.0f || owner->gimbal_ctx.cmd->yaw_target_angle == 0.0f)
    {
        owner->gimbal_ctx.data._target_yaw_angle += owner->gimbal_ctx.cmd->yaw_delta_angle;
    }
    else
    {
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.cmd->yaw_target_angle;// + owner->gimbal_ctx.feedforward_data.filtered_yaw_v * predict_time;
        _last_filtered_yaw = filtered_yaw;
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
        owner->gimbal_ctx.data._target_pitch_angle = - owner->gimbal_ctx.cmd->pitch_target_angle;// + owner->gimbal_ctx.feedforward_data.filtered_pitch_v * predict_time;
        _last_filtered_pitch = filtered_pitch;
    }

    // if (owner->gimbal_ctx.data._target_pitch_angle > pitch_max_value)
    // {
    //     owner->gimbal_ctx.data._target_pitch_angle = pitch_max_value;
    // }
    // else if (owner->gimbal_ctx.data._target_pitch_angle < pitch_min_value)
    // {
    //     owner->gimbal_ctx.data._target_pitch_angle = pitch_min_value;
    // }

    auto_aim_gimbal_control(&owner->gimbal_ctx);
    send_motor_command(&owner->gimbal_ctx);
}

void uav_gimbal_t::fsm_active_t::state_auto_t::exit(uav_gimbal_t *owner)
{

}