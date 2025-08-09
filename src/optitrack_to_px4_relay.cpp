#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>

#include <algorithm> // std::find_if
#include <iostream>  // std::cout
#include <cstdio>    // setvbuf

using namespace px4_msgs::msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

class OptitrackPX4Relay : public rclcpp::Node
{
public:
    OptitrackPX4Relay() : Node("optitrack2_px4_relay_node")
    {
        declare_parameter<std::string>("rigid_body_name", "");
        try
        {
            get_parameter("rigid_body_name", rigid_body_name_);
        }
        catch (rclcpp::ParameterTypeException &excp)
        {
            RCLCPP_ERROR(get_logger(), "Parameter type exception caught");
            rclcpp::shutdown(nullptr, "Parameter type exception caught on initialization");
        }

        auto px4_qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();

        optitrack_rigid_body_array_sub_ = create_subscription<mocap4r2_msgs::msg::RigidBodies>(
            "/rigid_bodies", 10, std::bind(&OptitrackPX4Relay::callback_rigidbodies_sub, this, _1));

        px4_visual_odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", px4_qos_pub);

        timer_pub_rigid_body_ = this->create_wall_timer(
            10ms, std::bind(&OptitrackPX4Relay::callback_rigidbodies_pub, this));
    }

private:
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_visual_odom_pub_;
    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr optitrack_rigid_body_array_sub_;

    px4_msgs::msg::VehicleOdometry mocap_odometry_msg{};
    rclcpp::TimerBase::SharedPtr timer_pub_rigid_body_;

    void callback_rigidbodies_sub(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg);
    void callback_rigidbodies_pub();

    std::string rigid_body_name_;
};

void OptitrackPX4Relay::callback_rigidbodies_pub()
{
    px4_visual_odom_pub_->publish(mocap_odometry_msg);
}

void OptitrackPX4Relay::callback_rigidbodies_sub(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->rigidbodies[0].rigid_body_name.c_str());

    // find the rigid body by name
    const auto find_rigid_body_ptr = std::find_if(
        msg->rigidbodies.begin(), msg->rigidbodies.end(),
        [this](const auto &rb)
        { return rb.rigid_body_name == rigid_body_name_; });
    // RCLCPP_INFO(this->get_logger(), "Rigid bodies count: %zu", msg->rigidbodies.size());
    // RCLCPP_INFO(this->get_logger(), "Rigid body '%s' has index %zu", rigid_body_name_.c_str(), rigid_body_idx);
    // if (!msg->rigidbodies.empty())
    // {
    //     RCLCPP_INFO(this->get_logger(),
    //                 "rigidbodies.begin() name = %s",
    //                 msg->rigidbodies.begin()->rigid_body_name.c_str());
    // }
    // if (find_rigid_body_ptr == msg->rigidbodies.end())
    // {
    //     RCLCPP_WARN(this->get_logger(), "Rigid body '%s' not found", rigid_body_name_.c_str());
    //     // Not found; nothing to publish this cycle
    //     return;
    // }
    // else
    // {
    //     RCLCPP_INFO(this->get_logger(), "Rigid body '%s' found", rigid_body_name_.c_str());
    // }
    auto rigid_body_idx = find_rigid_body_ptr - msg->rigidbodies.begin();

    // Frame & pose
    mocap_odometry_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    const auto &p = msg->rigidbodies[rigid_body_idx].pose.position;
    const auto &q = msg->rigidbodies[rigid_body_idx].pose.orientation;
    mocap_odometry_msg.position[0] = static_cast<float>(p.x);
    mocap_odometry_msg.position[1] = static_cast<float>(p.y);
    mocap_odometry_msg.position[2] = static_cast<float>(p.z);
    // PX4 quaternion order is [w, x, y, z]
    mocap_odometry_msg.q[0] = static_cast<float>(q.w);
    mocap_odometry_msg.q[1] = static_cast<float>(q.x);
    mocap_odometry_msg.q[2] = static_cast<float>(q.y);
    mocap_odometry_msg.q[3] = static_cast<float>(q.z);

    mocap_odometry_msg.timestamp = int(get_clock()->now().nanoseconds() / 1000);
    mocap_odometry_msg.timestamp_sample = mocap_odometry_msg.timestamp;

    // px4_visual_odom_pub_->publish(mocap_odometry_msg);
}

int main(int argc, char *argv[])
{
    std::cout << "Starting OptiTrack PX4 odometry relay..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptitrackPX4Relay>());
    rclcpp::shutdown();
    return 0;
}
