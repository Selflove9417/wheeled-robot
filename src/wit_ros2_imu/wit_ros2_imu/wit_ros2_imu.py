import math
import serial
import struct
import numpy as np
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# 存储传感器原始解析数据的全局变量
key = 0
buff = {}
angularVelocity = [0.0, 0.0, 0.0]
acceleration = [0.0, 0.0, 0.0]
magnetometer = [0.0, 0.0, 0.0]
angle_degree = [0.0, 0.0, 0.0]

# 线程锁，用于保护多线程间全局变量的读写安全
data_lock = threading.Lock()

def hex_to_short(raw_data):
    """将串口接收到的2字节16进制原始数据组合并解包为有符号短整型(short)"""
    return list(struct.unpack("hhhh", bytearray(raw_data)))


def check_sum(list_data, check_data):
    """对前10个字节进行校验和计算，并与第11个字节的校验码进行比对"""
    return sum(list_data) & 0xff == check_data


def handle_serial_data(raw_data):
    """解析输入的单个字节，根据维特智能协议帧格式分类解算加速度、角速度和角度"""
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity
    angle_flag = False
    
    buff[key] = raw_data
    key += 1

    # 帧头检查，若首字节不是0x55则清空缓存重新对齐
    if 0 in buff and buff[0] != 0x55:
        buff = {}
        key = 0
        return False
        
    # 确保单帧11个字节的数据收集完整
    if key < 11:
        return False
    else:
        data_buff = list(buff.values())
        with data_lock:
            # 解析加速度数据 (输出单位: m/s²)
            if buff[1] == 0x51:
                if check_sum(data_buff[0:10], data_buff[10]):
                    acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16.0 * 9.8 for i in range(0, 3)]
                else:
                    print('0x51 Check failure')
            # 解析角速度数据 (输出单位: rad/s)
            elif buff[1] == 0x52:
                if check_sum(data_buff[0:10], data_buff[10]):
                    angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000.0 * math.pi / 180.0 for i in range(0, 3)]
                else:
                    print('0x52 Check failure')
            # 解析欧拉角数据 (输出单位: 角度度数)
            elif buff[1] == 0x53:
                if check_sum(data_buff[0:10], data_buff[10]):
                    angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180.0 for i in range(0, 3)]
                    angle_flag = True
                else:
                    print('0x53 Check failure')
            # 解析磁力计数据
            elif buff[1] == 0x54:
                if check_sum(data_buff[0:10], data_buff[10]):
                    magnetometer = hex_to_short(data_buff[2:10])
                else:
                    print('0x54 Check failure')
            else:
                buff = {}
                key = 0
                return False

        buff = {}
        key = 0
        return angle_flag


def get_quaternion_from_euler(roll, pitch, yaw):
    """将弧度制的欧拉角(航向角、俯仰角、翻滚角)数学转换给ROS使用的四元数格式"""
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]


class IMUDriverNode(Node):
    """ROS 2 驱动节点类，负责初始化发布器、管理参数以及开辟串口读取线程"""
    def __init__(self, port_name):
        super().__init__('imu_driver_node')

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'

        # 根据ROS规范，将未使用的协方差矩阵首元素设为-1
        self.imu_msg.orientation_covariance[0] = -1.0
        self.imu_msg.angular_velocity_covariance[0] = -1.0
        self.imu_msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # 开启独立的后台线程，用于高频监听并读取串口硬件数据
        self.driver_thread = threading.Thread(target=self.driver_loop, args=(port_name,), daemon=True)
        self.driver_thread.start()

    def driver_loop(self, port_name):
        """串口管理与接收的主循环，持续监听串口数据流并分发给解析函数"""
        try:
            wt_imu = serial.Serial(port=port_name, baudrate=230400, timeout=0.5)
            if wt_imu.isOpen():
                self.get_logger().info(f"\033[32mSerial port {port_name} opened successfully...\033[0m")
            else:
                wt_imu.open()
                self.get_logger().info(f"\033[32mSerial port {port_name} opened successfully...\033[0m")
        except Exception as e:
            self.get_logger().error(f"Serial port opening failure: {e}")
            return

        while rclpy.ok():
            try:
                buff_count = wt_imu.inWaiting()
                if buff_count > 0:
                    buff_data = wt_imu.read(buff_count)
                    for i in range(0, buff_count):
                        tag = handle_serial_data(buff_data[i])
                        if tag:
                            self.publish_imu_data()
                else:
                    # 串口无数据时短暂休眠，防止单核CPU被空转死循环占满
                    time.sleep(0.001)
            except Exception as e:
                self.get_logger().error(f"IMU loop exception: {e}")
                break

    def publish_imu_data(self):
        """将解析出的最新运动学参数打包进规范的ROS消息，并发布到对应的Topic中"""
        with data_lock:
            accel_x, accel_y, accel_z = acceleration[0], acceleration[1], acceleration[2]
            gyro_x, gyro_y, gyro_z = angularVelocity[0], angularVelocity[1], angularVelocity[2]
            local_angles = list(angle_degree)

        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.imu_msg.linear_acceleration.x = accel_x
        self.imu_msg.linear_acceleration.y = accel_y
        self.imu_msg.linear_acceleration.z = accel_z
        
        self.imu_msg.angular_velocity.x = gyro_x
        self.imu_msg.angular_velocity.y = gyro_y
        self.imu_msg.angular_velocity.z = gz = gyro_z
        
        # 转换角度单位并生成四元数后填充消息体
        angle_radian = [local_angles[i] * math.pi / 180.0 for i in range(3)]
        qua = get_quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])
        
        self.imu_msg.orientation.x = qua[0]
        self.imu_msg.orientation.y = qua[1]
        self.imu_msg.orientation.z = qua[2]
        self.imu_msg.orientation.w = qua[3]

        self.imu_pub.publish(self.imu_msg)


def main():
    """节点入口函数，执行环境初始化、节点实例化以及自旋阻塞"""
    rclpy.init()
    node = IMUDriverNode('/dev/ttyCH341USB0')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()