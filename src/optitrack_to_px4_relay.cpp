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
    // find the rigid body by name
    const auto it = std::find_if(
        msg->rigidbodies.begin(), msg->rigidbodies.end(),
        [this](const auto &rb)
        { return rb.rigid_body_name == rigid_body_name_; });

    if (it == msg->rigidbodies.end())
    {
        // Not found; nothing to publish this cycle
        return;
    }

    const auto &p = it->pose.position;
    const auto &q = it->pose.orientation;

    // Frame & pose
    mocap_odometry_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    mocap_odometry_msg.position[0] = static_cast<float>(p.x);
    mocap_odometry_msg.position[1] = static_cast<float>(p.y);
    mocap_odometry_msg.position[2] = static_cast<float>(p.z);
    // PX4 quaternion order is [w, x, y, z]
    mocap_odometry_msg.q[0] = static_cast<float>(q.w);
    mocap_odometry_msg.q[1] = static_cast<float>(q.x);
    mocap_odometry_msg.q[2] = static_cast<float>(q.y);
    mocap_odometry_msg.q[3] = static_cast<float>(q.z);

    // Use message header time if available
    const uint64_t ts_us =
        static_cast<uint64_t>(rclcpp::Time(msg->header.stamp).nanoseconds() / 1000ULL);
    mocap_odometry_msg.timestamp = ts_us;
    mocap_odometry_msg.timestamp_sample = ts_us;

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
