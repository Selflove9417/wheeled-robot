#include "bbot_real/kinematics.hpp"
#include <algorithm>
#include <cmath>

namespace bbot_real
{

Kinematics::Kinematics(const RobotParams & params) : params_(params) {}

void Kinematics::compute_absolute_angles(double body_pitch, double hip_angle, double knee_angle,
                                          double & out_theta_body, double & out_theta_thigh,
                                          double & out_theta_shank) const
{
    out_theta_body  = body_pitch;
    out_theta_thigh = body_pitch - hip_angle;
    out_theta_shank = out_theta_thigh - knee_angle;
}

double Kinematics::calculate_com_height(double body_pitch, double hip_angle, double knee_angle) const
{
    double theta_body, theta_thigh, theta_shank;
    compute_absolute_angles(body_pitch, hip_angle, knee_angle,
                            theta_body, theta_thigh, theta_shank);

    double z_knee = params_.l1 * std::cos(theta_shank);
    double z_hip  = z_knee + params_.l2 * std::cos(theta_thigh);
    double z_body = z_hip  + params_.l3 * std::cos(theta_body);

    // 各杆件质心高度
    double z_c1 = params_.x1c * std::cos(theta_shank);               // 小腿
    double z_c2 = z_knee + params_.x2c * std::cos(theta_thigh);      // 大腿
    double z_c3 = z_body;                                             // 上身

    double M = params_.M_total();
    return (params_.m1 * z_c1 + params_.m2 * z_c2 + params_.m3 * z_c3) / M;
}

FKResult Kinematics::forward_kinematics(double body_pitch, double hip_angle, double knee_angle) const
{
    double theta_body, theta_thigh, theta_shank;
    compute_absolute_angles(body_pitch, hip_angle, knee_angle,
                            theta_body, theta_thigh, theta_shank);

    FKResult fk;
    fk.x_knee = params_.l1 * std::sin(theta_shank);
    fk.z_knee = params_.l1 * std::cos(theta_shank);
    fk.x_hip  = fk.x_knee + params_.l2 * std::sin(theta_thigh);
    fk.z_hip  = fk.z_knee + params_.l2 * std::cos(theta_thigh);
    fk.x_body = fk.x_hip  + params_.l3 * std::sin(theta_body);
    fk.z_body = fk.z_hip  + params_.l3 * std::cos(theta_body);

    double x_c1 = params_.x1c * std::sin(theta_shank);
    double x_c2 = fk.x_knee + params_.x2c * std::sin(theta_thigh);
    double z_c1 = params_.x1c * std::cos(theta_shank);
    double z_c2 = fk.z_knee + params_.x2c * std::cos(theta_thigh);

    double M = params_.M_total();
    fk.x_com = (params_.m1 * x_c1 + params_.m2 * x_c2 + params_.m3 * fk.x_body) / M;
    fk.z_com = (params_.m1 * z_c1 + params_.m2 * z_c2 + params_.m3 * fk.z_body) / M;

    return fk;
}

IKSolution Kinematics::inverse_kinematics(double target_z, double body_pitch, double target_x) const
{
    double L = std::sqrt(target_x * target_x + target_z * target_z);
    L = std::clamp(L, params_.L_MIN, params_.L_MAX);

    double gamma = std::atan2(target_x, target_z);

    // 2连杆IK（余弦定理）：l1, l2, L
    double cos_knee = (L * L - params_.l1 * params_.l1 - params_.l2 * params_.l2) /
                      (2.0 * params_.l1 * params_.l2);
    cos_knee = std::clamp(cos_knee, -1.0, 1.0);
    double theta_knee = std::acos(cos_knee);

    double cos_beta = (params_.l1 * params_.l1 + L * L - params_.l2 * params_.l2) /
                      (2.0 * params_.l1 * L);
    cos_beta = std::clamp(cos_beta, -1.0, 1.0);
    double beta = std::acos(cos_beta);

    IKSolution ik;
    ik.theta_shank = gamma - beta;
    ik.theta_knee  = theta_knee;
    ik.theta_hip   = body_pitch - (ik.theta_shank + ik.theta_knee);

    return ik;
}

Jacobian2D Kinematics::compute_jacobian(double body_pitch, double hip_angle, double knee_angle) const
{
    double theta_body, theta_thigh, theta_shank;
    compute_absolute_angles(body_pitch, hip_angle, knee_angle,
                            theta_body, theta_thigh, theta_shank);

    double M = params_.M_total();
    double a1 = params_.m1 * params_.x1c + (params_.m2 + params_.m3) * params_.l1;
    double b1 = params_.m2 * params_.x2c + params_.m3 * params_.l2;
    double c1 = params_.m3 * params_.l3;

    Jacobian2D J;
    J.Jx_hip  = (-a1 * std::cos(theta_shank) - b1 * std::cos(theta_thigh)) / M;
    J.Jx_knee = (-a1 * std::cos(theta_shank)) / M;
    J.Jx_body = ( a1 * std::cos(theta_shank) + b1 * std::cos(theta_thigh) + c1 * std::cos(theta_body)) / M;

    J.Jz_hip  = ( a1 * std::sin(theta_shank) + b1 * std::sin(theta_thigh)) / M;
    J.Jz_knee = ( a1 * std::sin(theta_shank)) / M;
    J.Jz_body = (-a1 * std::sin(theta_shank) - b1 * std::sin(theta_thigh) - c1 * std::sin(theta_body)) / M;

    return J;
}

JointTorques Kinematics::compute_gravity_torques(double body_pitch, double hip_angle,
                                                   double knee_angle) const
{
    Jacobian2D J = compute_jacobian(body_pitch, hip_angle, knee_angle);
    double M = params_.M_total();

    // 虚功原理: τ = J^T · [0, -M·g]^T
    JointTorques t;
    t.hip_torque   = -M * params_.g * J.Jz_hip;
    t.knee_torque  = -M * params_.g * J.Jz_knee;
    t.wheel_torque = -M * params_.g * J.Jz_body;
    return t;
}

JointTorques Kinematics::compute_gravity_torques_at_height(double target_z, double body_pitch,
                                                             double target_x) const
{
    IKSolution ik = inverse_kinematics(target_z, body_pitch, target_x);
    return compute_gravity_torques(body_pitch, ik.theta_hip, ik.theta_knee);
}

double Kinematics::estimate_wheel_balance_torque(double com_height, double body_pitch) const
{
    return params_.M_total() * params_.g * com_height * std::sin(body_pitch);
}

}  // namespace bbot_real
