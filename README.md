RoboMaster 视觉培训 - 第三次培训：ROS 2 海康工业相机驱动

一、 项目简介

本项目为 RoboMaster 算法组第三次培训的最终交付物。
基于 Ubuntu 24.04 和 ROS 2 Jazzy，使用 C++ 封装了海康机器人（HIKROBOT）MVS SDK，实现了一个高性能、高稳定性的 ROS 2 工业相机驱动节点。

该节点具备自动发现相机、硬件取图、多线程图像处理与发布、动态参数配置、断线重连以及性能诊断等工业级特性。

二、 环境与依赖

· 操作系统：Ubuntu 24.04 LTS
· ROS 版本：ROS 2 Jazzy (注：培训教程指定为 Ubuntu 22.04 + ROS 2 Humble，经测试 Jazzy 与该驱动代码完全兼容，且能够正常运行)
· 核心依赖包：
  · rclcpp, sensor_msgs, std_msgs, std_srvs, rcl_interfaces
  · OpenCV (用于图像色彩转换和去马赛克)
  · 海康 MVS SDK (必须提前从海康官网下载并安装到 /opt/MVS，rosdep 无法自动安装该 SDK)

三、 目录结构

```text
ros2_ws/src/hk_camera_driver/
├── CMakeLists.txt              # 编译配置
├── package.xml                 # 包依赖清单
├── README.md                   # 本文件
├── ACCEPTANCE.md               # 验收记录表
├── src/
│   ├── hk_camera_node.cpp      # 相机驱动核心节点（多线程架构）
│   └── image_rate_probe.cpp    # 独立帧率测量工具
├── include/hk_camera_driver/
│   └── core.hpp                # 核心工具与验证逻辑
├── launch/
│   └── camera.launch.py        # 一键启动脚本
├── config/
│   └── camera.yaml             # 相机参数配置文件
├── rviz/
│   └── camera.rviz             # RViz2 可视化配置
└── tests/
    └── core_test.cpp           # 核心逻辑单元测试
```

四、 编译步骤

在终端依次执行以下命令：

```bash
# 1. 加载 ROS 2 环境
source /opt/ros/jazzy/setup.bash

# 2. 进入工作空间
cd ~/ros2_ws

# 3. 编译功能包（如果 MVS 不在默认的 /opt/MVS，请用 -DMVS_ROOT 指定）
colcon build --packages-select hk_camera_driver \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DMVS_ROOT=/opt/MVS

# 4. 激活工作空间
source install/setup.bash
```

五、 运行方式

1. 直接运行节点

```bash
ros2 run hk_camera_driver hk_camera_node
```

2. 使用 Launch 文件启动（推荐）

```bash
# 仅启动相机节点
ros2 launch hk_camera_driver camera.launch.py

# 启动相机节点并同时打开 RViz2 预览画面
ros2 launch hk_camera_driver camera.launch.py rviz:=true
```

(注意：若修改了源目录里的 camera.yaml，启动时建议使用绝对路径 params_file:=$HOME/ros2_ws/src/hk_camera_driver/config/camera.yaml 避免读取到旧配置)

六、 核心功能与特性说明

1. 多线程采集与发布架构：将 SDK 阻塞取图、图像格式转换和 ROS 2 发布解耦为独立的采集线程和处理线程。通过“最新帧队列（LatestSlot）”保证发布延迟最低，即使处理速度变慢，也会智能丢弃旧帧，而非造成画面滞后。
2. 像素格式自适应：支持 Mono8、RGB8、BGR8 及多种 Bayer 格式（RG/GR/GB/BG）的自动识别和色彩转换。
3. 动态参数实时生效：运行时修改曝光、增益、帧率，节点会暂停采集、校验 SDK 返回值、应用新参数，并支持失败回滚。
4. 断线重连保护：当相机断连或取流超时，节点会自动清理旧句柄并定期尝试枚举重连，恢复最后成功接受的参数，杜绝“僵尸节点”。
5. 性能诊断与独立验证：
   · 节点每 2 秒打印一次 capture/process/publish 三段真实频率统计。
   · 提供 image_rate_probe 工具，独立运行以测量最准确的订阅端接收频率。

七、 参数配置与话题

· 发布话题：/image_raw (sensor_msgs/msg/Image)，默认 QoS 为 Best Effort。
· 可配置参数：
  · camera_ip：相机 IP（USB 相机保持为空）
  · serial_number：相机序列号（多相机场景建议填写）
  · image_topic：发布话题名，默认 /image_raw
  · exposure_time：曝光时间（微秒）
  · gain：增益
  · frame_rate：帧率（0.0 表示解除硬件帧率限制）
  · pixel_format：像素格式，默认 BayerRG8

八、 动态参数操作与回读

在节点运行时，另开终端执行：

```bash
# 修改曝光时间为 10000 微秒
ros2 param set /hk_camera_node exposure_time 10000.0

# 修改增益
ros2 param set /hk_camera_node gain 15.0

# 查看相机硬件寄存器里真实写入的值（包括实际帧率、实际曝光）
ros2 service call /hk_camera_node/readback std_srvs/srv/Trigger '{}'
```

九、 性能实测记录与说明

经实测，相机在 1280x720 分辨率、BayerRG8 格式下：

· SDK 物理采集频率：稳定在 24 Hz（受 USB 传输链路实际带宽影响）。
· 图像转换耗时：极低（约 1.5 毫秒/帧），证明 CPU 逻辑处理没有瓶颈。
· 丢帧策略表现：latest_queue_drops 数值存在，表明发布端偶尔阻塞，但采集端仍然稳定工作，这是实时视觉系统预期的行为（宁丢帧，不延迟）。

详细的可复现实验数据和回读日志，请查阅项目根目录下的 ACCEPTANCE.md 文件。

十、 作者

· 姓名：  王子凡
· 学号：2264215100
· 日期：2026.10.01
