#include "pyro_uav_gimbal.h"
#include "forecast_v.h"

using namespace pyro;

void uav_gimbal_t::fsm_active_t::state_auto_t::enter(uav_gimbal_t *owner)
{

}

void uav_gimbal_t::fsm_active_t::state_auto_t::execute(uav_gimbal_t *owner)
{
// #if test_gimbal
//     //比较一下两种方法哪种效果更好
//
//     //用卡尔曼滤波拟合自瞄发来的角度来预测角速度  用于前馈来提高跟踪目标的速度
//     auto *kf = new feedforward_kf_t();
//     kf->kf_predict();
//     kf->kf_update(owner->gimbal_ctx.cmd->yaw_target_angle);
//     const float ff_angle = kf->get_v();
// #endif
    owner->feedforward_compensation(&yaw_ff,&pitch_ff);

    //没识别到目标的时候用遥控器控制 识别到目标的时候由自瞄控制
    if (owner->gimbal_ctx.cmd->yaw_target_angle > 80.0f || owner->gimbal_ctx.cmd->yaw_target_angle == 0.0f)
    {
        owner->gimbal_ctx.data._target_yaw_angle += owner->gimbal_ctx.cmd->yaw_delta_angle;
    }
    else
    {
        owner->gimbal_ctx.data._target_yaw_angle = owner->gimbal_ctx.cmd->yaw_target_angle;
        owner->gimbal_ctx.data._target_yaw_speed += yaw_ff;
// #if test_gimbal
//         owner->gimbal_ctx.data._target_yaw_speed += ff_angle * 0.5f;
// #endif
    }

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
        owner->gimbal_ctx.data._target_pitch_angle = - owner->gimbal_ctx.cmd->pitch_target_angle;
        owner->gimbal_ctx.data._target_pitch_speed += pitch_ff;
    }

    if (owner->gimbal_ctx.data._target_pitch_angle > owner->gimbal_ctx.data.pitch_real_max_limit_angle)
    {
        owner->gimbal_ctx.data._target_pitch_angle = owner->gimbal_ctx.data.pitch_real_max_limit_angle;
    }
    if (owner->gimbal_ctx.data._target_pitch_angle < owner->gimbal_ctx.data.pitch_real_min_limit_angle)
    {
        owner->gimbal_ctx.data._target_pitch_angle = owner->gimbal_ctx.data.pitch_real_min_limit_angle;
    }

    auto_aim_gimbal_control(&owner->gimbal_ctx);
    send_motor_command(&owner->gimbal_ctx);
}

void uav_gimbal_t::fsm_active_t::state_auto_t::exit(uav_gimbal_t *owner)
{

}