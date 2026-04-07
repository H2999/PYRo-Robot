#include "pyro_uav_gimbal.h"
#include "forecast_v.h"

using namespace pyro;

void uav_gimbal_t::fsm_active_t::state_auto_t::enter(uav_gimbal_t *owner)
{

}

void uav_gimbal_t::fsm_active_t::state_auto_t::execute(uav_gimbal_t *owner)
{
    //用卡尔曼滤波拟合自瞄发来的角度来预测角速度 用于前馈来提高跟踪目标的速度
    auto *kf = new feedforward_kf_t();
    kf->kf_predict();
    kf->kf_update(owner->gimbal_ctx.cmd->yaw_target_angle);
    const float ff_angle = kf->get_v();

    //没识别到目标的时候用遥控器控制 识别到目标的时候由自瞄控制
    if (owner->gimbal_ctx.cmd->yaw_target_angle > 10.0f)
    {
        owner->gimbal_ctx.data._target_yaw_angle += owner->gimbal_ctx.cmd->yaw_delta_angle;
    }
    else
    {
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.cmd->yaw_target_angle;
        owner->gimbal_ctx.data._target_yaw_speed += ff_angle * 0.5f;
    }

    if (owner->gimbal_ctx.data._target_yaw_angle > owner->gimbal_ctx.data.yaw_real_max_limit_angle)
    {
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data.yaw_real_max_limit_angle;
    }
    if (owner->gimbal_ctx.data._target_yaw_angle < owner->gimbal_ctx.data.yaw_real_min_limit_angle)
    {
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.data.yaw_real_min_limit_angle;
    }


    //计算垂直速度 然后加入速度环前馈
    last_pitch_target_angle = now_pitch_target_angle;
    now_pitch_target_angle = owner->gimbal_ctx.cmd->pitch_target_angle;
    constexpr float dt = 0.001f;
    constexpr float Kff = 0.6f;
    float feedforward = (now_pitch_target_angle - last_pitch_target_angle) / dt;

    if (owner->gimbal_ctx.cmd->pitch_target_angle > 10.0f)
    {
        owner->gimbal_ctx.data._target_pitch_angle += owner->gimbal_ctx.cmd->pitch_delta_angle;
    }
    else
    {
        owner->gimbal_ctx.data._target_pitch_angle = - owner->gimbal_ctx.cmd->pitch_target_angle;
        owner->gimbal_ctx.data._target_pitch_speed += feedforward * Kff;
    }

    gimbal_control(&owner->gimbal_ctx);
    send_motor_command(&owner->gimbal_ctx);
}

void uav_gimbal_t::fsm_active_t::state_auto_t::exit(uav_gimbal_t *owner)
{

}