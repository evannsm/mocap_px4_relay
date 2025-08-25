#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <mocap4r2_msgs/msg/full_state.hpp>

#include <chrono>     // std::chrono_literals
#include <functional> // std::bind, std::placeholders::_1

#include <algorithm> // std::find_if
#include <iostream>  // std::cout
#include <cstdio>    // setvbuf

using namespace px4_msgs::msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

class FullStateRelay : public rclcpp::Node
{
public:
    FullStateRelay() : Node("full_state_relay_node")
    {
        auto px4_qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();
        auto px4_qos_sub = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

        vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", px4_qos_sub, std::bind(&FullStateRelay::vehicleOdometryCallback, this, _1));

        local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", px4_qos_sub, std::bind(&FullStateRelay::localPositionCallback, this, _1));

        full_state_pub_ = create_publisher<mocap4r2_msgs::msg::FullState>(
            "/merge_odom_localpos/full_state_relay", px4_qos_pub);

        timer_pub_full_state_ = this->create_wall_timer(
            10ms, std::bind(&FullStateRelay::publishFullState, this));
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    void vehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    px4_msgs::msg::VehicleOdometry vehicle_odometry_msg{};

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    px4_msgs::msg::VehicleLocalPosition local_position_msg{};

    rclcpp::Publisher<mocap4r2_msgs::msg::FullState>::SharedPtr full_state_pub_;
    void publishFullState();
    mocap4r2_msgs::msg::FullState full_state_msg{};
    rclcpp::TimerBase::SharedPtr timer_pub_full_state_;
};

void FullStateRelay::publishFullState()
{
    // std::cout << "Publishing FullState message." << std::endl;
    // // print full_state_msg contents below
    // std::cout << "Position: [" << full_state_msg.position[0] << ", "
    //           << full_state_msg.position[1] << ", "
    //           << full_state_msg.position[2] << "]" << std::endl;
    // std::cout << "Velocity: [" << full_state_msg.velocity[0] << ", "
    //           << full_state_msg.velocity[1] << ", "
    //           << full_state_msg.velocity[2] << "]" << std::endl;
    // std::cout << "Acceleration: [" << full_state_msg.acceleration[0] << ", "
    //           << full_state_msg.acceleration[1] << ", "
    //           << full_state_msg.acceleration[2] << "]" << std::endl;
    // std::cout << "Orientation (quaternion): [" << full_state_msg.quaternion[0] << ", "
    //           << full_state_msg.quaternion[1] << ", "
    //           << full_state_msg.quaternion[2] << ", "
    //           << full_state_msg.quaternion[3] << "]" << std::endl;
    // std::cout << "Angular Velocity: [" << full_state_msg.angular_velocity[0] << ", "
    //           << full_state_msg.angular_velocity[1] << ", "
    //           << full_state_msg.angular_velocity[2] << "]" << std::endl;

    full_state_pub_->publish(full_state_msg);
}

void FullStateRelay::localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    // std::cout << "Received VehicleLocalPosition message." << std::endl;

    full_state_msg.acceleration[0] = msg->ax;
    full_state_msg.acceleration[1] = msg->ay;
    full_state_msg.acceleration[2] = msg->az;
}

void FullStateRelay::vehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    // std::cout << "Received VehicleOdometry message." << std::endl;
    full_state_msg.position[0] = msg->position[0];
    full_state_msg.position[1] = msg->position[1];
    full_state_msg.position[2] = msg->position[2];

    full_state_msg.velocity[0] = msg->velocity[0];
    full_state_msg.velocity[1] = msg->velocity[1];
    full_state_msg.velocity[2] = msg->velocity[2];

    full_state_msg.quaternion[0] = msg->q[0];
    full_state_msg.quaternion[1] = msg->q[1];
    full_state_msg.quaternion[2] = msg->q[2];
    full_state_msg.quaternion[3] = msg->q[3];

    full_state_msg.angular_velocity[0] = msg->angular_velocity[0];
    full_state_msg.angular_velocity[1] = msg->angular_velocity[1];
    full_state_msg.angular_velocity[2] = msg->angular_velocity[2];
}

int main(int argc, char *argv[])
{
    std::cout << "Starting Full State Relay..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FullStateRelay>());
    rclcpp::shutdown();
    return 0;
}
