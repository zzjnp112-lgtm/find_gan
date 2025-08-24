🚀 Pole Detection with LiDAR (ROS 2 + Open3D)
📌 简介 | Introduction

本项目基于 ROS 2 + Open3D，实现了对环境中 细长杆状物体（如路灯杆、电线杆） 的检测。
The project uses ROS 2 + Open3D to detect thin vertical poles (e.g., lamp posts, utility poles) from LiDAR point clouds.

主要功能 / Features:

点云清洗 (NaN/Inf/异常值移除)

多帧点云累积 (frame accumulation)

多种检测方法 (DBSCAN, 投影圆拟合, 垂直分层, ThinPole 专用检测)

输出杆子位置与检测评分 (publish results as /gan)

⚙️ 依赖环境 | Dependencies

ROS 2 (Humble / Foxy 推荐 Recommended)

Python ≥ 3.8

依赖库 | Python libs:

pip install open3d numpy


ROS 依赖 | ROS packages:

sudo apt install ros-${ROS_DISTRO}-rclpy ros-${ROS_DISTRO}-sensor-msgs \
                 ros-${ROS_DISTRO}-std-msgs ros-${ROS_DISTRO}-geometry-msgs

🚀 使用方法 | Usage

克隆到工作空间 | Clone into your ROS 2 workspace:

cd ~/ros2_ws/src
git clone https://github.com/zzjnp112-lgtm/find_gan.git


编译 | Build:

cd ~/ros2_ws
colcon build
source install/setup.bash


运行节点 | Run node:

ros2 run find_gan find_gan


查看结果 | Check results:

ros2 topic echo /gan

📊 输出示例 | Example Output

终端日志 | Console log:

[ThinPole] 检测到杆子! 位置: 前方2.34m, 右方0.56m, 直径: 0.030m, 高度: 1.85m, 评分: 0.83


话题消息 /gan | Topic message /gan:

r: 234.0     # 前方 2.34m | Forward 2.34 m
g: 56.0      # 右方 0.56m | Right 0.56 m
b: 0.83      # 评分 | Score
a: 1.0       # 检测到 | Detected

📝 注意事项 | Notes

确保话题 /cloud_registered 提供有效点云 | Make sure /cloud_registered provides valid point cloud.

可调整参数以适应不同杆子尺寸 | Adjust min_diameter / max_diameter / min_height to match pole size.

增加 max_frames 可提升稳定性 | Increase max_frames for more stable detection.
