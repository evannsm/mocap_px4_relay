#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <vicon_receiver/msg/position.hpp>
#include "px4_ros_com/frame_transforms.h"

#include <rclcpp/rclcpp.hpp>

using namespace px4_msgs::msg;

class ViconPX4Relay : public rclcpp::Node {
public:
    ViconPX4Relay() : Node("vicon_px4_relay_node") {
        auto px4_qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();

        px4_visual_odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
                "/fmu/in/vehicle_visual_odometry", px4_qos_pub);

        vicon_sub_ = create_subscription<vicon_receiver::msg::Position>("", 10,
                        [this](const vicon_receiver::msg::Position::UniquePtr msg){
                            px4_msgs::msg::VehicleOdometry px4_mocap_odom_msg;
                            auto q_ned = px4_ros_com::frame_transforms::enu_to_ned_orientation(
                                    Eigen::Quaterniond(
                                               msg->w,
                                            msg->x_rot,
                                            msg->y_rot,
                                            msg->z_rot));
                            px4_mocap_odom_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
                            px4_mocap_odom_msg.position[0] =  static_cast<float>(msg->y_trans / 1000.0);
                            px4_mocap_odom_msg.position[1] =  static_cast<float>(msg->x_trans / 1000.0);
                            px4_mocap_odom_msg.position[2] =  static_cast<float>(-msg->z_trans / 1000.0);
                            px4_mocap_odom_msg.q[0] = static_cast<float>(q_ned.w());
                            px4_mocap_odom_msg.q[1] = static_cast<float>(q_ned.x());
                            px4_mocap_odom_msg.q[2] = static_cast<float>(q_ned.y());
                            px4_mocap_odom_msg.q[3] = static_cast<float>(q_ned.z());
                            px4_mocap_odom_msg.timestamp = int(get_clock()->now().nanoseconds() / 1000);
                            px4_mocap_odom_msg.timestamp_sample = px4_mocap_odom_msg.timestamp;

                            px4_visual_odom_pub_->publish(px4_mocap_odom_msg);
                        });
    }

private:
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_visual_odom_pub_;
    rclcpp::Subscription<vicon_receiver::msg::Position>::SharedPtr vicon_sub_;
};

int main(int argc, char *argv[]) {
    std::cout << "Starting OptiTrack PX4 odometry relay..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ViconPX4Relay>());
    rclcpp::shutdown();
    return 0;
}