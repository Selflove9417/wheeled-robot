#ifndef BBOT_REAL_ROBOT_PARAMS_HPP
#define BBOT_REAL_ROBOT_PARAMS_HPP

namespace bbot_real
{

    /**
     * @brief 双轮足机器人物理参数。
     *
     * 模型：3连杆平面链（小腿→大腿→上身）+ 双轮。
     * 所有质量为双腿合计。
     *
     * 移植自: bbot_kinematics/include/bbot_kinematics/robot_params.hpp
     */
    struct RobotParams
    {
        // ---- 几何 ----
        double wheel_radius = 0.070;   // 轮子半径 (m)
        double l1 = 0.30;              // 小腿长（膝→轮轴）
        double l2 = 0.30;              // 大腿长（髋→膝）
        double l3 = 0.10;              // 髋到上身质心距离
        double x1c = 0.15;             // 小腿质心（距膝）
        double x2c = 0.15;             // 大腿质心（距髋）
        double wheel_separation = 0.4; // 轮距

        // ---- 质量 ----
        double m0 = 4.00; // 轮子（双侧）
        double m1 = 1.60; // 小腿（双侧）
        double m2 = 2.40; // 大腿（双侧）
        double m3 = 9.5;  // 上身

        double M_total() const { return m0 + m1 + m2 + m3; }

        // ---- 限位 ----
        double L_MIN = 0.30; // 虚拟腿长最小值 (m)
        double L_MAX = 0.45; // 虚拟腿长最大值 (m)
        double g = 9.81;     // 重力加速度

        double hip_torque_max = 75.0;  // 髋关节峰值扭矩 (Nm)
        double knee_torque_max = 60.0; // 膝关节峰值扭矩 (Nm)
        double wheel_torque_max = 7.0; // 轮毂电机峰值扭矩 (Nm)
    };

} // namespace bbot_real

#endif // BBOT_REAL_ROBOT_PARAMS_HPP
