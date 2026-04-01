#include <math.h>
#include <chrono>
#include <iostream>

 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

 
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"

 
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

const double PI = 3.1415926;

class VehicleSimulatorNode : public rclcpp::Node
{ 
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubVehicleOdom_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subSpeed_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
    gazebo_msgs::srv::SetEntityState::Request::SharedPtr request_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

     
    float vehicleX = 0.0f;           
    float vehicleY = 0.0f;           
    float vehicleZ = 0.0f;           
    float vehicleRoll = 0.0f;        
    float vehiclePitch = 0.0f;       
    float vehicleYaw = 0.0f;         

     
    float vehicleYawRate = 0.0f;     
    float vehicleSpeed = 0.0f;       
    float vehicleSpeedY = 0.0f;     

     
    float node_rate = 200.0f;      
    float dt = (1.0f / node_rate); 


    double sensorOffsetX = 0.0;      
    double sensorOffsetY = 0.0;      
    double cameraOffsetZ = 0.0;      
    double vehicleHeight = 0.75;     

     
    rclcpp::Time odomTime;
    nav_msgs::msg::Odometry odomData;
    geometry_msgs::msg::TransformStamped tf_msg; 
    gazebo_msgs::msg::EntityState cameraState;
    gazebo_msgs::msg::EntityState lidarState;
    gazebo_msgs::msg::EntityState robotState;

     
    void vehicle_info();
    void initdeclare_parameters();
    void vehicle_speed_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr cmd_vel);

public:
    VehicleSimulatorNode() : Node("vehicleSimulator")
    {
        initdeclare_parameters();
        pubVehicleOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 5);
        client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
        request_ = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        subSpeed_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 5,
            std::bind(&VehicleSimulatorNode::vehicle_speed_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt),
            std::bind(&VehicleSimulatorNode::vehicle_info, this));
        RCLCPP_INFO(this->get_logger(), "Velodyne Initialized!, rate: %f HZ", node_rate);
    }
};

 