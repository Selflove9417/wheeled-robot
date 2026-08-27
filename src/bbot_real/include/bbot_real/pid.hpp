#ifndef BBOT_REAL_PID_HPP
#define BBOT_REAL_PID_HPP

namespace bbot_real
{

/**
 * @brief PID控制器，带积分抗饱和、输出斜率限制和微分低通滤波。
 *
 * 移植自: bbot_balance_controller/include/bbot_balance_controller/pid.h
 */
class PIDController
{
public:
    float P, I, D;
    float output_ramp;   // 输出变化率限制（0=不限制）
    float limit;         // 输出幅值限制 [-limit, limit]

    float error_prev = 0.0f;
    float output_prev = 0.0f;
    float integral_prev = 0.0f;
    float derivative_prev = 0.0f;

    PIDController(float p = 0.0f, float i = 0.0f, float d = 0.0f,
                  float ramp = 0.0f, float lim = 1.0f);

    /// 计算PID输出
    float operator()(float error, float dt);

    /// 复位所有内部状态
    void reset();
};

}  // namespace bbot_real

#endif  // BBOT_REAL_PID_HPP
