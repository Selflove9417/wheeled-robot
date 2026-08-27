#ifndef BBOT_REAL_KINEMATICS_HPP
#define BBOT_REAL_KINEMATICS_HPP

#include "bbot_real/robot_params.hpp"

namespace bbot_real
{

/// 逆运动学解
struct IKSolution
{
    double theta_shank;   // 小腿绝对角 (rad)
    double theta_knee;    // 膝关节角 (rad)
    double theta_hip;     // 髋关节角 (rad)
};

/// 重力补偿扭矩（单腿）
struct JointTorques
{
    double hip_torque;    // Nm
    double knee_torque;   // Nm
    double wheel_torque;  // Nm
};

/// 2×3 质心雅可比: [vx, vz]^T = J · [dθ_hip, dθ_knee, dθ_body]^T
struct Jacobian2D
{
    double Jx_hip, Jx_knee, Jx_body;
    double Jz_hip, Jz_knee, Jz_body;
};

/// 正运动学结果
struct FKResult
{
    double x_knee, z_knee;
    double x_hip, z_hip;
    double x_body, z_body;
    double x_com, z_com;
};

/**
 * @brief 双轮足机器人运动学与动力学。
 *
 * 移植自: bbot_kinematics/include/bbot_kinematics/kinematics.hpp
 */
class Kinematics
{
public:
    explicit Kinematics(const RobotParams & params = RobotParams());

    // ---- 角度工具 ----
    void compute_absolute_angles(double body_pitch, double hip_angle, double knee_angle,
                                 double & out_theta_body, double & out_theta_thigh,
                                 double & out_theta_shank) const;

    // ---- 正运动学 ----
    double calculate_com_height(double body_pitch, double hip_angle, double knee_angle) const;
    FKResult forward_kinematics(double body_pitch, double hip_angle, double knee_angle) const;

    // ---- 逆运动学（2连杆余弦定理） ----
    IKSolution inverse_kinematics(double target_z, double body_pitch, double target_x) const;

    // ---- 雅可比 ----
    Jacobian2D compute_jacobian(double body_pitch, double hip_angle, double knee_angle) const;

    // ---- 重力补偿（虚功原理） ----
    JointTorques compute_gravity_torques(double body_pitch, double hip_angle, double knee_angle) const;
    JointTorques compute_gravity_torques_at_height(double target_z, double body_pitch,
                                                    double target_x = 0.0) const;

    // ---- 轮部平衡扭矩（倒立摆模型） ----
    double estimate_wheel_balance_torque(double com_height, double body_pitch) const;

    const RobotParams & params() const { return params_; }
    void set_params(const RobotParams & p) { params_ = p; }

private:
    RobotParams params_;
};

}  // namespace bbot_real

#endif  // BBOT_REAL_KINEMATICS_HPP
