#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyController : public rclcpp::Node
{
public:
    JoyController() : Node("joy_controller")
    {
        // 声明参数（可以在 launch 文件或命令行中覆盖）
        this->declare_parameter("axis_linear_x", 1);   // 左摇杆上下
        this->declare_parameter("axis_linear_y", 0);   // 左摇杆左右 (平移)
        this->declare_parameter("axis_angular_z", 3);  // 右摇杆左右 (旋转)
        this->declare_parameter("scale_linear", 0.5);
        this->declare_parameter("scale_angular", 1.0);

        // 获取参数
        this->get_parameter("axis_linear_x", linear_x_axis_);
        this->get_parameter("axis_linear_y", linear_y_axis_);
        this->get_parameter("axis_angular_z", angular_z_axis_);
        this->get_parameter("scale_linear", l_scale_);
        this->get_parameter("scale_angular", a_scale_);

        // 创建发布者：发布到 /cmd_vel
        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

        // 创建订阅者：订阅 /joy 话题
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyController::joy_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Joy Controller Node Started. Ready to move!");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
    {
        auto msg = geometry_msgs::msg::TwistStamped();

        // 核心逻辑：将手柄数据映射到 Twist
        // linear.x: 前后
        msg.twist.linear.x = l_scale_ * joy->axes[linear_x_axis_];
        // linear.y: 左右平移 (全向轮/类人机器人横移)
        msg.twist.linear.y = l_scale_ * joy->axes[linear_y_axis_];
        // angular.z: 自旋
        msg.twist.angular.z = a_scale_ * joy->axes[angular_z_axis_];

        vel_pub_->publish(msg);
    }

    // 内部变量
    int linear_x_axis_, linear_y_axis_, angular_z_axis_;
    double l_scale_, a_scale_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyController>());
    rclcpp::shutdown();
    return 0;
}