#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "bbot_real/can_interface.hpp"
#include "bbot_real/joint_motor_driver.hpp"

int main(int argc, char ** argv)
{
    std::string can_if = "can0";
    if (argc > 1) can_if = argv[1];

    std::cout << "========================================" << std::endl;
    std::cout << "  BBot 关节电机零点标定工具" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "CAN接口: " << can_if << std::endl;
    std::cout << std::endl;

    // 打开CAN
    auto can = std::make_shared<bbot_real::CanInterface>();
    try
    {
        can->open(can_if);
        std::cout << "[OK] CAN接口打开成功" << std::endl;
    }
    catch (const std::exception & e)
    {
        std::cerr << "[FAIL] CAN打开失败: " << e.what() << std::endl;
        return 1;
    }

    // 初始化4个关节电机
    bbot_real::JointMotorDriver motors[4];
    const char * names[4] = {"左髋", "左膝", "右髋", "右膝"};
    int ids[4] = {1, 2, 3, 4};

    for (int i = 0; i < 4; ++i)
    {
        if (!motors[i].init(can, ids[i]))
        {
            std::cerr << "[FAIL] 电机初始化失败: " << names[i] << " (ID=" << ids[i] << ")" << std::endl;
            return 1;
        }
    }
    std::cout << "[OK] 4个关节电机已连接" << std::endl;

    // ---- 步骤1: 关闭电机，使关节可自由转动 ----
    std::cout << std::endl;
    std::cout << "步骤1: 关闭电机使能..." << std::endl;
    for (int i = 0; i < 4; ++i)
    {
        motors[i].disable();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::cout << "[OK] 电机已关闭，现在可以手动转动关节" << std::endl;

    // ---- 步骤2: 等待用户摆好伸直姿态 ----
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  请手动将机器人双腿摆成完全伸直姿态" << std::endl;
    std::cout << "  （小腿和大腿成一条直线，垂直于地面）" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "摆好后按回车键继续...";
    std::cin.get();

    // ---- 步骤3: 设置零点 ----
    std::cout << std::endl;
    std::cout << "步骤2: 设置当前位置为零点..." << std::endl;
    for (int i = 0; i < 4; ++i)
    {
        if (motors[i].set_zero_position())
            std::cout << "  [OK] " << names[i] << " (ID=" << ids[i] << ") 零点已设置" << std::endl;
        else
            std::cout << "  [WARN] " << names[i] << " (ID=" << ids[i] << ") 设置失败" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // ---- 步骤4: 重新使能 ----
    std::cout << std::endl;
    std::cout << "步骤3: 使能电机..." << std::endl;
    for (int i = 0; i < 4; ++i)
    {
        motors[i].enable();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::cout << "[OK] 电机已使能，当前位置为零点" << std::endl;

    // ---- 完成 ----
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  标定完成！" << std::endl;
    std::cout << "  伸直姿态 = 髋0° 膝0°（IK的零点参考）" << std::endl;
    std::cout << "========================================" << std::endl;

    can->close();
    return 0;
}
