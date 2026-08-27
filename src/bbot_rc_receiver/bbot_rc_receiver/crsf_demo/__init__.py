"""CRSF Demo —— 一个用 Python 实现的 Crossfire (CRSF) 协议演示项目。

定位：作为串口端点，**被动解析**进来的 CRSF 控制帧（RC_CHANNELS_PACKED），
并按配置的周期**回传遥测帧**。不模拟发射机/对端，串口对面接什么由用户决定
（例如：真实飞控、真实接收机、自定义 CRSF 主控、串口调试助手等）。

包含：
- 协议层（CRC8 / 帧打包解包 / 各遥测帧编解码）
- 串口 I/O 抽象（跨 Windows / Linux / macOS）
- 控制帧解析 + 遥测回传（``CrsfReceiver``）
- 遥测数据源（固定值 / 随时间变化）

参考资料：
    Betaflight CRSF 协议（src/main/rx/crsf_protocol.h）
    ExpressLRS 文档：https://www.expresslrs.org/
    TBS Crossfire 用户手册
"""

__version__ = "0.1.0"
