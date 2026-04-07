#include "forecast_v.h"

feedforward_kf_t::feedforward_kf_t()
{
    xk[0] = 0.0f; // 初始角度
    xk[1] = 0.0f;  // 初始速度
    xk[2] = 0.0f;  // 初始加速度
    kf_init();
}

void feedforward_kf_t::kf_init()
{
    // 初始化 P 矩阵(协方差)
    for(float & i : P)
    {
        i = 0.0f;
    }
    P[0] = 1.0f;
    P[4] = 1.0f;
    P[8] = 1.0f;

    // 观测噪声 R
    R = 0.01f;
    Q_val = 0.001f;

    arm_mat_init_f32(&xk_vec, 3, 1, xk);
    arm_mat_init_f32(&P_vec, 3, 3, P);
}

// 预测步：xk = F * xk
void feedforward_kf_t::kf_predict()
{
    // 1. 状态预测（手动展开 3x1 矩阵乘法）
    xk[0] = xk[0] + xk[1] * delta_t + 0.5f * delta_t * delta_t * xk[2];
    xk[1] = xk[1] + xk[2] * delta_t;
    xk[2] = xk[2];

    // 2. 协方差预测 P = F*P*F' + Q
    // Q 代表模型预测的不确定性，值越大，自瞄跟随越灵敏但越抖
    P[0] += Q_val; // 角度不确定度
    P[4] += Q_val; // 速度不确定度
    P[8] += Q_val; // 加速度不确定度
}

// 更新步：根据视觉测量值修正状态
void feedforward_kf_t::kf_update(const float vision_angle)
{
    // 1. 计算残差 (Innovation)
    // 因为 H = [1, 0, 0]，所以 H*xk 就是 xk[0]
    float y = vision_angle - xk[0];

    // 2. 计算卡尔曼增益 K (3x1)
    // S = H*P*H' + R = P[0] + R
    const float S = P[0] + R;
    float K[3];
    K[0] = P[0] / S;
    K[1] = P[3] / S;
    K[2] = P[6] / S;

    // 3. 更新状态量 xk = xk + K*y
    xk[0] += K[0] * y;
    xk[1] += K[1] * y;
    xk[2] += K[2] * y;

    // 4. 更新协方差 P = (I - K*H) * P
    const float m0 = P[0];
    const float m1 = P[1];
    const float m2 = P[2];
    P[0] -= K[0] * m0;
    P[1] -= K[0] * m1;
    P[2] -= K[0] * m2;
    P[3] -= K[1] * m0;
    P[4] -= K[1] * m1;
    P[5] -= K[1] * m2;
    P[6] -= K[2] * m0;
    P[7] -= K[2] * m1;
    P[8] -= K[2] * m2;
}

float feedforward_kf_t::predict_future_angle(const float delay_time) const
{
    float prediction = xk[0] + xk[1] * delay_time + 0.5f * xk[2] * delay_time * delay_time;
    return prediction;
}

float feedforward_kf_t::get_v() const
{
    return xk[1];
}
