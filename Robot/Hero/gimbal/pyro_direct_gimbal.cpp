#include "pyro_direct_gimbal.h"
#include "pyro_ins.h"

namespace pyro
{

// =========================================================
// 构造与初始化
// =========================================================

direct_gimbal_t::direct_gimbal_t() : module_base_t("direct_gimbal")
{
    _ctx = {};
}

status_t direct_gimbal_t::_init()
{
    // 1. 初始化电机

    // Pitch: 使用 DM 电机 (示例 ID: Master 0x11, Slave 0x21, CAN1)
    // 根据 hybrid 中的用法进行配置
    _ctx.motor.pitch = new dm_motor_drv_t(0x33, 0x43, can_hub_t::can2);

    // Yaw: 使用 DJI GM6020 (ID 2, CAN1)
    _ctx.motor.yaw   = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                                   can_hub_t::can3);

    // 2. 配置 DM 电机范围 (DJI 电机无需配置)
    // NOLINTBEGIN(cppcoreguidelines-pro-type-static-cast-downcast)
    static_cast<dm_motor_drv_t *>(_ctx.motor.pitch)
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(_ctx.motor.pitch)
        ->set_rotate_range(-2.72, 2.72); // rad/s
    static_cast<dm_motor_drv_t *>(_ctx.motor.pitch)
        ->set_torque_range(-27, 27); // Nm (DM单位通常为Nm)
    // NOLINTEND(cppcoreguidelines-pro-type-static-cast-downcast)

    // 3. 初始化串级 PID
    _ctx.pid.pitch_pos =
        new pid_t(40.0f, 0.1f, 3.4f, 2.0f, 25.0f, 20, 10,
                  4); // 位置环输出为 rad/s，限制在电机可接受范围内
    _ctx.pid.pitch_spd = new pid_t(1.4f, 0.02f, 0.025f, 1.5f, 22.0f, 40, 20,
                                   4); // 输出限制匹配 DM 电机 Nm 级

    // Yaw 轴 (DJI GM6020，输出为电流值/电压值，通常量级较大，如 +/- 30000)
    _ctx.pid.yaw_pos   = new pid_t(5.2f, 0.01f, 0.22f, 0.8f, 5.0f);
    _ctx.pid.yaw_spd   = new pid_t(3.0f, 0.0003f, 0.0001f, 0.2f, 3.0f);


    return PYRO_OK;
}

// =========================================================
// 核心循环回调
// =========================================================

void direct_gimbal_t::_update_feedback()
{
    // 1. 刷新电机数据
    _ctx.motor.pitch->update_feedback();
    _ctx.motor.yaw->update_feedback();


    // 读取 IMU 数据作为云台姿态反馈
    ins_drv_t::get_instance()->get_rads_n(&_ctx.data.current_yaw_rad,
                                          &_ctx.data.current_pitch_rad,
                                          &_ctx.data.current_roll_rad);

    ins_drv_t::get_instance()->get_gyro_b(&_ctx.data.current_yaw_radps,
                                          &_ctx.data.current_pitch_radps,
                                          &_ctx.data.current_roll_radps);

    ins_drv_t::get_instance()->get_accel_b(&_ctx.data.current_x_accel,
                                           &_ctx.data.current_y_accel,
                                           &_ctx.data.current_z_accel);

    // 通常不用读取电机位置作为姿态反馈，改为使用 IMU 数据
    // // 读取 电机 数据作为反馈 (含 Offset)
    // _ctx.data.current_pitch_rad =
    //     _ctx.motor.pitch->get_current_position() - PITCH_OFFSET_RAD;
    // _ctx.data.current_pitch_radps = _ctx.motor.pitch->get_current_rotate();
    //
    // _ctx.data.current_yaw_rad =
    //     _ctx.motor.yaw->get_current_position() - YAW_OFFSET_RAD;
    // _ctx.data.current_yaw_radps = _ctx.motor.yaw->get_current_rotate();
}

const direct_gimbal_t::gimbal_context_t& direct_gimbal_t::get_ctx() const
{
    return _ctx;
}


void direct_gimbal_t::_gimbal_control(gimbal_context_t *ctx)
{

    ctx->data.out_gravity_torque =
        -ctx->data.current_z_accel * 12.5f * 1.05f * 0.0325f;

    // --- Pitch 串级控制 ---
    // 1. 位置环
    ctx->data.target_pitch_radps = ctx->pid.pitch_pos->calculate(
        ctx->data.target_pitch_rad, ctx->data.current_pitch_rad);

    // 2. 速度环
    ctx->data.out_pitch_torque = ctx->pid.pitch_spd->calculate(
        ctx->data.target_pitch_radps, ctx->data.current_pitch_radps);

    // --- Yaw 串级控制 ---
    // 1. 位置环
    ctx->data.target_yaw_radps = ctx->pid.yaw_pos->calculate(
        ctx->data.target_yaw_rad, ctx->data.current_yaw_rad);

    // 2. 速度环
    ctx->data.out_yaw_torque = ctx->pid.yaw_spd->calculate(
        ctx->data.target_yaw_radps, ctx->data.current_yaw_radps);
}

void direct_gimbal_t::_send_motor_command(gimbal_context_t *ctx)
{
    // ctx->motor.pitch->send_torque(ctx->data.out_pitch_torque);
    // ctx->motor.pitch->send_torque(ctx->data.out_gravity_torque);
    ctx->motor.pitch->send_torque(ctx->data.out_gravity_torque +
                                  ctx->data.out_pitch_torque);
    ctx->motor.yaw->send_torque(ctx->data.out_yaw_torque);

    // ctx->motor.pitch->send_torque(0);
    // ctx->motor.yaw->send_torque(0);
}


// =========================================================
// 状态机逻辑
// =========================================================

void direct_gimbal_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);
    else if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}

} // namespace pyro