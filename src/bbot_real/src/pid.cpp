#include "bbot_real/pid.hpp"
#include <algorithm>

namespace bbot_real
{

PIDController::PIDController(float p, float i, float d, float ramp, float lim)
    : P(p), I(i), D(d), output_ramp(ramp), limit(lim) {}

float PIDController::operator()(float error, float dt)
{
    if (dt <= 0.0f) dt = 0.005f;

    // 比例
    float proportional = P * error;

    // 积分（梯形积分 + 限幅防饱和）
    float integral = integral_prev + I * dt * 0.5f * (error + error_prev);
    integral = std::clamp(integral, -limit / 3.0f, limit / 3.0f);
    integral_prev = integral;

    // 微分（低通滤波）
    float raw_deriv = (error - error_prev) / dt;
    derivative_prev = 0.15f * raw_deriv + 0.85f * derivative_prev;

    float output = std::clamp(proportional + integral + D * derivative_prev, -limit, limit);

    // 输出斜率限制
    if (output_ramp > 0.0f)
    {
        float max_step = output_ramp * dt;
        float delta = output - output_prev;
        output = (delta > max_step) ? output_prev + max_step :
                 (delta < -max_step) ? output_prev - max_step : output;
    }

    error_prev = error;
    output_prev = output;
    return output;
}

void PIDController::reset()
{
    error_prev = output_prev = integral_prev = derivative_prev = 0.0f;
}

}  // namespace bbot_real
