#ifndef PYRO_ROBOT_FORECAST_V_H
#define PYRO_ROBOT_FORECAST_V_H

#include "arm_math.h"
#include "dsp/matrix_functions.h"

class feedforward_kf_t
{
public:
    explicit feedforward_kf_t();
    ~feedforward_kf_t() = default;
    //卡尔曼预测
    void kf_predict();
    //更新卡尔曼
    void kf_update(float vision_angle);
    //获取用于前馈的角速度
    [[nodiscard]] float get_v() const;

    [[nodiscard]] float predict_future_angle(float delay_time) const;
private:
    //初始化矩阵和噪声参数
    void kf_init();

    // 更新频率
    float delta_t = 0.001f;

    // 状态向量 xk = [角度, 速度, 加速度]^T
    float xk[3]{};
    arm_matrix_instance_f32 xk_vec{};

    // 协方差矩阵 P (3x3)
    float P[9]{};
    arm_matrix_instance_f32 P_vec{};

    // 观测噪声 R
    float R{};

    // 过程噪声 Q_val
    float Q_val{};
};

// // 1. 获取目标距离和弹速
// float distance = get_target_distance();
// float bullet_speed = 25.0f;
//
// // 2. 计算总延迟 = 系统固有延迟 + 子弹飞行延迟
// float total_delay = 0.001f + (distance / bullet_speed);
//
// // 3. 获得预测的角度，并让云台指向这个预测点
// float target_pitch = pitch_kf.predict_future_angle(total_delay);
// set_gimbal_target(target_pitch);

#endif //PYRO_ROBOT_FORECAST_V_H