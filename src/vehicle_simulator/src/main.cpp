#include "vehicle_simulator/vehicleSimulator.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleSimulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}