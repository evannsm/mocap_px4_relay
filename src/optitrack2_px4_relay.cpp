#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <mocap_msgs/msg/rigid_body_array.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace px4_msgs::msg;

class OptitrackPX4Relay : public rclcpp::Node {
public:
    OptitrackPX4Relay() : Node("optitrack2_px4_relay_node") {
        declare_parameter<std::string>("rigid_body_name", "");
        try {
            get_parameter("rigid_body_name", rigid_body_name_);
        } catch (rclcpp::ParameterTypeException &excp) {
            RCLCPP_ERROR(get_logger(), "Parameter type exception caught");
            rclcpp::shutdown(nullptr, "Parameter type exception caught on initialization");
        }

        auto px4_qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();

        px4_visual_odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
                "/fmu/in/vehicle_visual_odometry", px4_qos_pub);

        optitrack_rigid_body_array_sub_ =
                create_subscription<mocap_msgs::msg::RigidBodyArray>("/optitrack2_driver/rigid_body_array", 10,
                 [this](const mocap_msgs::msg::RigidBodyArray::UniquePtr msg) {
                     auto find_rigid_body_ptr = std::find(
                             msg->rigid_body_names.begin(),
                             msg->rigid_body_names.end(),
                             rigid_body_name_);
                     if (find_rigid_body_ptr != msg->rigid_body_names.end()) {
                         px4_msgs::msg::VehicleOdometry mocap_odometry_msg;
                         auto rigid_body_idx = find_rigid_body_ptr - msg->rigid_body_names.begin();

                         mocap_odometry_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
                         mocap_odometry_msg.position[0] = static_cast<float>(msg->poses[rigid_body_idx].position.x);
                         mocap_odometry_msg.position[1] = static_cast<float>(msg->poses[rigid_body_idx].position.y);
                         mocap_odometry_msg.position[2] = static_cast<float>(msg->poses[rigid_body_idx].position.z);
                         mocap_odometry_msg.q[0] = static_cast<float>(msg->poses[rigid_body_idx].orientation.w);
                         mocap_odometry_msg.q[1] = static_cast<float>(msg->poses[rigid_body_idx].orientation.x);
                         mocap_odometry_msg.q[2] = static_cast<float>(msg->poses[rigid_body_idx].orientation.y);
                         mocap_odometry_msg.q[3] = static_cast<float>(msg->poses[rigid_body_idx].orientation.z);
                         mocap_odometry_msg.timestamp = int(get_clock()->now().nanoseconds() / 1000);
                         mocap_odometry_msg.timestamp_sample = mocap_odometry_msg.timestamp;
                         px4_visual_odom_pub_->publish(mocap_odometry_msg);
                     }
                 });
    }

private:
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_visual_odom_pub_;
    rclcpp::Subscription<mocap_msgs::msg::RigidBodyArray>::SharedPtr optitrack_rigid_body_array_sub_;

    std::string rigid_body_name_;
};


int main(int argc, char *argv[]) {
    std::cout << "Starting OptiTrack PX4 odometry relay..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OptitrackPX4Relay>());
    rclcpp::shutdown();
    return 0;
}