# mocap_bridge  
**ROS 2 VRPN → MAVROS 动捕位姿桥接节点**

---

## 1. 项目简介

`mocap_bridge` 是一个基于 **ROS 2 Humble** 的 Python 功能包，用于将动捕系统通过  
`/vrpn/<object>/pose` 发布的位姿信息，转换并转发至 **MAVROS** 的：

```

/mavros/mocap/pose

````

从而为 **PX4 飞控**提供可靠的 **无 GPS 位置参考（Motion Capture / Vision Pose）**。

该工程适用于：

- 室内动捕环境（OptiTrack / Vicon / 凌云光等）
- PX4 + MAVROS2
- 单机或多机 UAV

---

## 2. 系统架构

```text
[动捕机PC(Windows)]
          ↓ (VRPN)
[动捕机PC(Linux)]
  vrpn_client_ros2
          ↓
 /vrpn/UAV0(动捕软件可改)/pose
          ↓   (DDS 网络)
──────────────────────────────(同一局域网内)
          ↓
[板载电脑]
  mocap_bridge
          ↓
 /mavros/mocap/pose
          ↓
[ MAVROS2 ]
          ↓
[ PX4 EKF2 ]
````

---

## 3. 工程目录结构

```text
mocap_bridge/
├── launch/
│   └── mocap_bridge.launch.py     # 启动文件
├── mocap_bridge/
│   ├── __init__.py
│   └── vrpn_to_mavros.py          # 核心节点
├── resource/
│   └── mocap_bridge
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## 4. 环境依赖

### 必需软件

* Ubuntu 22.04
* ROS 2 Humble
* MAVROS2
* PX4（1.13+ 推荐）

### ROS 2 依赖包

* `rclpy`
* `geometry_msgs`
* `launch`
* `launch_ros`

---

## 5. 前置条件（非常重要）

在运行本工程前，请确认：

1. 板载电脑 **已经能收到 VRPN 位姿话题**，例如：

```bash
ros2 topic echo /vrpn/UAV0/pose
```

2. MAVROS2 已正常启动，并与 PX4 通信：

```bash
ros2 topic list | grep mavros
```

3. PX4 已开启视觉 / 动捕融合参数（见第 8 节）

---

## 6. 编译与安装

在工作空间根目录执行：

```bash
cd ~/ws_mocap
source /opt/ros/humble/setup.bash
colcon build --packages-select mocap_bridge
source install/setup.bash
```

建议加入 `~/.bashrc`：

```bash
echo "source ~/ws_mocap/install/setup.bash" >> ~/.bashrc
```

---

## 7. 启动方式

### 7.1 使用 launch 文件（推荐）

```bash
ros2 launch mocap_bridge mocap_bridge.launch.py
```

### 7.2 参数覆盖示例

```bash
ros2 launch mocap_bridge mocap_bridge.launch.py \
  vrpn_topic:=/vrpn/UAV1/pose \
  mavros_topic:=/mavros/vision_pose/pose
```

---

## 8. PX4 侧参数配置（关键）

在 QGroundControl 或 MAVLink Console 中设置：

```text
EKF2_AID_MASK = 24     # vision + mocap
EKF2_EV_CTRL = 3
EKF2_HGT_MODE = Vision
EKF2_EV_DELAY = 0
EKF2_EV_POS_X = 0
EKF2_EV_POS_Y = 0
EKF2_EV_POS_Z = 0
```

修改后 **重启飞控**。

---

## 9. 坐标系说明

### 输入（VRPN）

* 默认：ENU

  * x → East
  * y → North
  * z → Up

### 输出（PX4 via MAVROS）

* PX4 使用 NED

  * x → North
  * y → East
  * z → Down

本节点已在代码中完成如下映射：

```text
x_out = x_in
y_out = z_in
z_out = -y_in
```

---

## 10. 验证方法

### 方法一：ROS 话题验证

```bash

ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyACM0:115200
```
波特率和接口根据实际情况调整。
另开窗口
```bash

ros2 topic echo /mavros/local_position/pose
```

移动飞机，位置应随动捕变化。

---

### 方法二：QGroundControl

* Estimator Status 正常
* Vision Position Innovation 接近 0
* Local Position 非 NaN

---

## 11. 常见问题

### Q1：`ros2 launch mocap_bridge` 报 `file 'None' not found`

**原因**：未指定 launch 文件名
**正确命令**：

```bash
ros2 launch mocap_bridge mocap_bridge.launch.py
```

---

### Q2：能看到 `/vrpn/.../pose`，但 PX4 位置不动

请重点检查：

* PX4 EKF2 参数是否正确
* MAVROS 是否订阅 `/mavros/mocap/pose`
* 频率是否 ≥ 30 Hz（推荐 100 Hz）

---

## 12. 后续可扩展方向

* 多机支持（UAV0 / UAV1 / UAV2）
* 加入滤波器（低通 / EKF / UKF）
* systemd 上电自启动
* Docker 化部署
* TF 广播与 RViz 可视化

---

## 13. 许可证

本项目采用 Apache-2.0 License。