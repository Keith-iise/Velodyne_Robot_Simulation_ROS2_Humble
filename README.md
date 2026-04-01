# ROS2 Humble Vehicle Simulator

本仓库是一个基于 ROS2 Humble 的车辆模拟器工作区，包含车辆仿真、机器人工具箱和 Velodyne 激光雷达模拟相关包。

## 主要内容

- `src/vehicle_simulator`
  - 车辆仿真包
  - 包含 Gazebo 场景、模型、URDF、launch 文件
- `src/robot_toolbox`
  - 机器人相关工具包
  - 包含控制、bringup 和 joystick 相关启动配置
- `src/velodyne_simulator`
  - Velodyne 仿真元包
  - 包含 `velodyne_description`、`velodyne_gazebo_plugins`、`velodyne_simulator`

## 依赖环境

- ROS 2 Humble
- Gazebo ROS
- `colcon` 构建工具
- 其他 ROS 依赖由各个包的 `package.xml` 指定

## 快速开始

### 1. 设置 ROS 2 环境

```bash
source /opt/ros/humble/setup.bash
```

### 2. 构建工作区

推荐使用仓库根目录中的脚本：

```bash
./build.sh
```

如果你希望使用符号链接安装，可执行：

```bash
./build.sh symlink
```

构建指定包：

```bash
./build.sh <package_name>
```

### 3. 载入构建结果

```bash
source install/setup.bash
```

## 启动示例

- 启动车辆模拟器：

```bash
ros2 launch vehicle_simulator vehicle_simulator.launch.py
```

- 启动机器人工具箱 bringup：

```bash
ros2 launch robot_toolbox bringup.launch.py
```

- 启动摇杆控制器：

```bash
ros2 launch robot_toolbox joy_controller.launch.py
```

## 目录结构

- `build/`：构建输出目录
- `install/`：安装输出目录
- `log/`：构建日志目录
- `src/`：源代码目录
  - `robot_toolbox/`
  - `vehicle_simulator/`
  - `velodyne_simulator/`

## 开发提示

- 如果新增或修改包，建议使用 `colcon build --packages-select <package_name>` 进行快速构建。
- 运行 Gazebo 仿真时，请确保 Gazebo 插件路径和模型资源路径已正确配置。

## 注意事项

- 当前仓库中的部分包元信息仍为 TODO，请根据实际项目补充 `package.xml` 中的 `description`、`license` 等字段。
- 若需要加载全局 ROS 2 环境，请先执行 `source /opt/ros/humble/setup.bash`。
