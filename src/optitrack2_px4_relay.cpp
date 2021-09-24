#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <mocap_msgs/msg/rigid_body_array.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace px4_msgs::msg;

class OptitrackPX4Relay : public rclcpp::Node {
public:
    OptitrackPX4Relay() : Node("optitrack2_px4_relay_node") {
        declare_parameter<std::string>("rigid_body_name", "unknown");
        try {
            get_parameter("rigid_body_name", rigid_body_name_);
        } catch (rclcpp::ParameterTypeException &excp) {
            RCLCPP_ERROR(get_logger(), "Parameter type exception caught");
            rclcpp::shutdown(nullptr, "Parameter type exception caught on initialization");
        }

        px4_visual_odom_pub_ = create_publisher<px4_msgs::msg::VehicleVisualOdometry>(
                "/" + rigid_body_name_ + "/fmu/vehicle_visual_odometry/in", 10);

        px4_timesync_sub_ =
                this->create_subscription<px4_msgs::msg::Timesync>("/" + rigid_body_name_ + "/fmu/timesync/out", 10,
                                                                   [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
                                                                       timestamp_.store(msg->timestamp);
                                                                   });
        optitrack_rigid_body_array_sub_ =
                this->create_subscription<mocap_msgs::msg::RigidBodyArray>("/optitrack2_driver/rigid_body_array", 10,
                                                                   [this](const mocap_msgs::msg::RigidBodyArray::UniquePtr msg) {
                                                                       auto find_rigid_body_ptr = std::find(
                                                                               msg->rigid_body_names.begin(),
                                                                               msg->rigid_body_names.end(),
                                                                               rigid_body_name_);
                                                                       if (find_rigid_body_ptr !=
                                                                           msg->rigid_body_names.end()) {
                                                                           px4_msgs::msg::VehicleVisualOdometry visual_odometry_msg;
                                                                           auto rigid_body_idx = find_rigid_body_ptr -
                                                                                                 msg->rigid_body_names.begin();
                                                                           visual_odometry_msg.x = msg->poses[rigid_body_idx].position.x;
                                                                           visual_odometry_msg.y = msg->poses[rigid_body_idx].position.y;
                                                                           visual_odometry_msg.z = msg->poses[rigid_body_idx].position.z;
                                                                           visual_odometry_msg.q[0] = msg->poses[rigid_body_idx].orientation.w;
                                                                           visual_odometry_msg.q[1] = msg->poses[rigid_body_idx].orientation.x;
                                                                           visual_odometry_msg.q[2] = msg->poses[rigid_body_idx].orientation.y;
                                                                           visual_odometry_msg.q[3] = msg->poses[rigid_body_idx].orientation.z;
                                                                           visual_odometry_msg.timestamp =
                                                                                   timestamp_.load() - (this->now() -
                                                                                                        msg->header.stamp).nanoseconds() / 1000;
                                                                           px4_visual_odom_pub_->publish(visual_odometry_msg);
                                                                       }
                                                                   });
    }

private:
    rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr px4_visual_odom_pub_;
    rclcpp::Subscription<mocap_msgs::msg::RigidBodyArray>::SharedPtr optitrack_rigid_body_array_sub_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr px4_timesync_sub_;

    std::string rigid_body_name_;
    std::atomic<uint64_t> timestamp_;
};


int main(int argc, char *argv[]) {
    std::cout << "Starting OptiTrack PX4 odometry relay..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptitrackPX4Relay>());
    rclcpp::shutdown();
    return 0;
}