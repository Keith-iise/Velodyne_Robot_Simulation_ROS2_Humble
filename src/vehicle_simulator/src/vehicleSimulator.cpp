#include <math.h>
#include <chrono>
#include <iostream>

// 核心ROS2基础头文件
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

// 核心消息/服务头文件
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"

// TF2相关头文件（发布TF变换必需）
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

const double PI = 3.1415926;

class VehicleSimulatorNode : public rclcpp::Node
{ 
private:
    //主循环
    rclcpp::TimerBase::SharedPtr timer_;

    // 里程计发布
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubVehicleOdom_;

    // 订阅cmdvel导航
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subSpeed_;

    // 客户端调用gazebo服务
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;

    // 存储客户端请求
    gazebo_msgs::srv::SetEntityState::Request::SharedPtr request_;

    // 机器人tf
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

    // ========== 车辆核心状态变量 ==========

    //用于计算里程计
    float vehicleX = 0.0f;          // 机器人X坐标  m
    float vehicleY = 0.0f;          // 机器人Y坐标  m
    float vehicleZ = 0.0f;          // 机器人Z坐标  m
    float vehicleRoll = 0.0f;       // 机器人Roll  rad
    float vehiclePitch = 0.0f;      // 机器人Pitch rad
    float vehicleYaw = 0.0f;        // 机器人Yaw   rad

    // 用于控制机器人移动
    float vehicleYawRate = 0.0f;    // 机器人角速度 rad/s
    float vehicleSpeed = 0.0f;      // 机器人线速度 m/s
    float vehicleSpeedY = 0.0f;

    // 用于改变gazebo中机器人姿态
    float terrainZ = 0.0f;          // 地形高度  m
    float terrainRoll = 0.0f;       // 地形沿世界坐标系 X 轴的倾斜角度
    float terrainPitch = 0.0f;      // 地形沿世界坐标系 Y 轴的倾斜角度

    // ========== 核心配置参数 ==========
    float rclcpp_rate = 200.0f;     // 发布频率

    // 用于计算每帧里程数据
    float dt = (1.0f / rclcpp_rate);// 时间间隔

    // ========== 传感器偏移参数 ==========
    // 这些参数不给也可以,但是里程会有所偏差,给了会更好
    double sensorOffsetX = 0.0;     // 传感器X轴偏移  m
    double sensorOffsetY = 0.0;     // 传感器Y轴偏移  m
    double cameraOffsetZ = 0.0;     // 相机 Z轴偏移  m
    double vehicleHeight = 0.75;    // 机器人高度  m

    // ========== 消息/TF缓存变量 ==========
    rclcpp::Time odomTime;
    nav_msgs::msg::Odometry odomData;
    geometry_msgs::msg::TransformStamped tf_msg; 
    geometry_msgs::msg::TransformStamped odom_tf_msg; 
    gazebo_msgs::msg::EntityState cameraState;
    gazebo_msgs::msg::EntityState lidarState;
    gazebo_msgs::msg::EntityState robotState;

    // ========== 回调函数 ==========
    void vehicle_info();
    void vehicle_speed_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr cmd_vel);

public:
    VehicleSimulatorNode() : Node("vehicleSimulator")
    {
        // 里程计发布者
        pubVehicleOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 5);
        
        // 机器人状态客户端
        client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
        
        // 构建请求消息
        request_ = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        
        // TF发布者
        tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 订阅cmdvel 用于控制机器人移动
        subSpeed_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 5,
            std::bind(&VehicleSimulatorNode::vehicle_speed_callback, this, std::placeholders::_1));
        
        // 创建主线程定时器
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt),
            std::bind(&VehicleSimulatorNode::vehicle_info, this));

        RCLCPP_INFO(this->get_logger(), "环境模拟节点启动!,频率:%fHZ", rclcpp_rate);
    }
};

// 速度回调函数
void VehicleSimulatorNode::vehicle_speed_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr cmd_vel)
{
    vehicleSpeed = cmd_vel->twist.linear.x;
    vehicleSpeedY = cmd_vel->twist.linear.y; 
    vehicleYawRate = cmd_vel->twist.angular.z;
}

void VehicleSimulatorNode::vehicle_info()
{
    float vehicleReceRoll = vehicleRoll;
    float vehicleRecePitch = vehiclePitch;
    float vehicleReceZ = vehicleZ;

    // 姿态更新
    vehicleRoll = terrainRoll * cos(vehicleYaw) + terrainPitch * sin(vehicleYaw);
    vehiclePitch = -terrainRoll * sin(vehicleYaw) + terrainPitch * cos(vehicleYaw);
    vehicleYaw += dt * vehicleYawRate;

    // 角度归一化
    if (vehicleYaw > PI) vehicleYaw -= 2 * PI;
    if (vehicleYaw < -PI) vehicleYaw += 2 * PI;

    // 车辆位置更新
    vehicleX += (dt * cos(vehicleYaw) * vehicleSpeed - dt * sin(vehicleYaw) * vehicleSpeedY) +
                dt * vehicleYawRate * (-sin(vehicleYaw)*sensorOffsetX - cos(vehicleYaw)*sensorOffsetY);
                
    vehicleY += (dt * sin(vehicleYaw) * vehicleSpeed + dt * cos(vehicleYaw) * vehicleSpeedY) +
                dt * vehicleYawRate * (cos(vehicleYaw)*sensorOffsetX - sin(vehicleYaw)*sensorOffsetY);
    vehicleZ = terrainZ + vehicleHeight;

    odomTime = this->now();

    // 1. 生成当前机器人的四元数
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(vehicleRoll, vehiclePitch, vehicleYaw);
    geometry_msgs::msg::Quaternion geoQuat = tf2::toMsg(quat_tf);
    
    // 2. 发布里程计消息 (odom -> base_link)
    odomData.header.stamp = odomTime;
    odomData.header.frame_id = "odom";
    odomData.child_frame_id = "base_link";
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = 0.0; // 通常 base_link 在地面
    
    // 角速度（绕X/Y/Z轴旋转的速度）
    odomData.twist.twist.angular.x = rclcpp_rate * (vehicleRoll - vehicleReceRoll);
    odomData.twist.twist.angular.y = rclcpp_rate * (vehiclePitch - vehicleRecePitch);
    odomData.twist.twist.angular.z = vehicleYawRate;

    // 线速度（沿X/Y/Z轴移动的速度）
    odomData.twist.twist.linear.x = vehicleSpeed;
    odomData.twist.twist.linear.z = rclcpp_rate * (vehicleZ - vehicleReceZ);
    
    pubVehicleOdom_->publish(odomData);

    // 3. 发布坐标变换 (odom -> base_link)
    // 这是机器人整体在地图上的移动
    tf_msg.header.stamp = odomTime;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.rotation = geoQuat;
    tf_msg.transform.translation.x = vehicleX;
    tf_msg.transform.translation.y = vehicleY;
    tf_msg.transform.translation.z = 0.0; 
    tfBroadcaster->sendTransform(tf_msg);


    // Gazebo 相机位姿设置
    cameraState.name = "camera";               //指定要控制的 Gazebo 实体名：相机（必须和 SDF/URDF 中相机名一致）
    cameraState.pose.orientation = geoQuat;    //相机姿态：复用车辆的四元数（和里程计/TF 姿态同步）
    cameraState.pose.position.x = vehicleX;    //相机位置：车辆位置 + 相机Z轴安装偏移（相机通常在车顶，所以Z轴抬高）
    cameraState.pose.position.y = vehicleY;
    cameraState.pose.position.z = vehicleZ + cameraOffsetZ;
    request_->state = cameraState;             //将相机状态赋值到服务请求中
    client_->async_send_request(request_);     //异步发送请求给 Gazebo

    // Gazebo 雷达位姿设置
    lidarState.name = "lidar";
    lidarState.pose.orientation = geoQuat;
    lidarState.pose.position.x = vehicleX;
    lidarState.pose.position.y = vehicleY;
    lidarState.pose.position.z = vehicleZ;
    request_->state = lidarState;
    client_->async_send_request(request_);

    // Gazebo 机器人位姿设置
    robotState.name = "robot";
    robotState.pose.orientation = geoQuat;
    robotState.pose.position.x = vehicleX;
    robotState.pose.position.y = vehicleY;
    robotState.pose.position.z = vehicleZ; 
    request_->state = robotState;
    client_->async_send_request(request_);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleSimulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}