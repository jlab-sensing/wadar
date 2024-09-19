#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "inertial-sense-sdk/msg/gps.hpp" // Replace with the actual path to your GPS message

#define COORDINATE_TOLERANCE 0.0001

class GPSSubscriber : public rclcpp::Node
{
public:
    GPSSubscriber()
        : Node("gps_subscriber_node")
    {
        subscription_ = this->create_subscription<inertial-sense-sdk::msg::GPS>(
            "GPS_rel/", 10, std::bind(&GPSSubscriber::gps_callback, this, std::placeholders::_1));
    }

private:
    void gps_callback(const your_custom_msg_package::msg::GPS::SharedPtr msg)
    {
        double target_latitude = 36.956999;
        double target_longitude = -122.058549;

        if (abs(msg->latitude - target_latitude) < COORDINATE_TOLERANCE &&
            abs(msg->longitude - target_longitude) < COORDINATE_TOLERANCE)
        {
            RCLCPP_INFO(this->get_logger(), "Target location reached: Latitude: %f, Longitude: %f",
                        msg->latitude, msg->longitude);
        }
    }

    rclcpp::Subscription<your_custom_msg_package::msg::GPS>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSSubscriber>());
    rclcpp::shutdown();
    return 0;
}
