#include "vehicle_simulator/vehicleSimulator.hpp"
void VehicleSimulatorNode::initdeclare_parameters()
{ 
    vehicleX = this->declare_parameter<float>("vehicleX", vehicleX);
    vehicleY = this->declare_parameter<float>("vehicleY", vehicleY);
    vehicleZ = this->declare_parameter<float>("vehicleZ", vehicleZ);
    vehicleRoll = this->declare_parameter<float>("vehicleRoll", vehicleRoll);
    vehiclePitch = this->declare_parameter<float>("vehiclePitch", vehiclePitch);
    vehicleYaw = this->declare_parameter<float>("vehicleYaw", vehicleYaw);
    node_rate = this->declare_parameter<float>("node_rate", node_rate);
    sensorOffsetX = this->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
    sensorOffsetY = this->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
    cameraOffsetZ = this->declare_parameter<double>("cameraOffsetZ", cameraOffsetZ);
    vehicleHeight = this->declare_parameter<double>("vehicleHeight", vehicleHeight);
}



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
    vehicleRoll = 0.0;
    vehiclePitch = 0.0;
    vehicleYaw += dt * vehicleYawRate;
    if (vehicleYaw > PI) vehicleYaw -= 2 * PI;
    if (vehicleYaw < -PI) vehicleYaw += 2 * PI;
    vehicleX += (dt * cos(vehicleYaw) * vehicleSpeed - dt * sin(vehicleYaw) * vehicleSpeedY) +
                dt * vehicleYawRate * (-sin(vehicleYaw)*sensorOffsetX - cos(vehicleYaw)*sensorOffsetY); 
    vehicleY += (dt * sin(vehicleYaw) * vehicleSpeed + dt * cos(vehicleYaw) * vehicleSpeedY) +
                dt * vehicleYawRate * (cos(vehicleYaw)*sensorOffsetX - sin(vehicleYaw)*sensorOffsetY);
    vehicleZ = vehicleHeight;
    odomTime = this->now();

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(vehicleRoll, vehiclePitch, vehicleYaw);
    geometry_msgs::msg::Quaternion geoQuat = tf2::toMsg(quat_tf);
    
    odomData.header.stamp = odomTime;
    odomData.header.frame_id = "odom";
    odomData.child_frame_id = "base_link";
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = 0.0;  
    odomData.twist.twist.angular.x = node_rate * (vehicleRoll - vehicleReceRoll);
    odomData.twist.twist.angular.y = node_rate * (vehiclePitch - vehicleRecePitch);
    odomData.twist.twist.angular.z = vehicleYawRate;
    odomData.twist.twist.linear.x = vehicleSpeed;
    odomData.twist.twist.linear.z = node_rate * (vehicleZ - vehicleReceZ);
    pubVehicleOdom_->publish(odomData);

    
    tf_msg.header.stamp = odomTime;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.rotation = geoQuat;
    tf_msg.transform.translation.x = vehicleX;
    tf_msg.transform.translation.y = vehicleY;
    tf_msg.transform.translation.z = 0.0; 
    tfBroadcaster->sendTransform(tf_msg);

    cameraState.name = "camera";                
    cameraState.pose.orientation = geoQuat;     
    cameraState.pose.position.x = vehicleX;     
    cameraState.pose.position.y = vehicleY;
    cameraState.pose.position.z = vehicleZ + cameraOffsetZ;
    request_->state = cameraState;              
    client_->async_send_request(request_);      

    lidarState.name = "lidar";
    lidarState.pose.orientation = geoQuat;
    lidarState.pose.position.x = vehicleX;
    lidarState.pose.position.y = vehicleY;
    lidarState.pose.position.z = vehicleZ;
    request_->state = lidarState;
    client_->async_send_request(request_);

    robotState.name = "robot";
    robotState.pose.orientation = geoQuat;
    robotState.pose.position.x = vehicleX;
    robotState.pose.position.y = vehicleY;
    robotState.pose.position.z = vehicleZ; 
    request_->state = robotState;
    client_->async_send_request(request_);
}

