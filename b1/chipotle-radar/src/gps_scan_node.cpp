#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <inertial_sense_ros2/msg/gps.hpp>
#include <iostream>
#include <fstream>
#include <vector>

#define COORDINATE_TOLERANCE 0.0001

class GPSSubscriber : public rclcpp::Node
{
public:
    GPSSubscriber()
        : Node("gps_subscriber_node")
    {
        subscription_ = this->create_subscription<inertial_sense_ros2::msg::GPS>(
            "/gps1/pos_vel", 10, std::bind(&GPSSubscriber::gps_callback, this, std::placeholders::_1));

        target_coordinates_ = {
            {36.956991, -122.058694},
            {36.956938, -122.058536},
            {34.052235, -118.243683}};
    }

private:
    void gps_callback(const inertial_sense_ros2::msg::GPS::SharedPtr msg)
    {
        static std::vector<bool> reached_targets(target_coordinates_.size(), false);

        for (size_t i = 0; i < target_coordinates_.size(); ++i)
        {
            if (reached_targets[i])
                continue;

            double target_latitude = target_coordinates_[i].first;
            double target_longitude = target_coordinates_[i].second;

            if (abs(msg->latitude - target_latitude) < COORDINATE_TOLERANCE &&
                abs(msg->longitude - target_longitude) < COORDINATE_TOLERANCE)
            {
                RCLCPP_INFO(this->get_logger(), "Target location reached: Latitude: %f, Longitude: %f",
                            msg->latitude, msg->longitude);

                std::string fullDataPath = "/data"; // Update the path to the data folder
                std::string airFramesName = "temp";
                std::string trialName = "test";
                double tagHz = 80;
                int frameCount = 2000;
                int captureCount = 1;

                // std::string command = "./wadar wadarTagTest -s " + fullDataPath +
                //               " -b " + airFramesName +
                //               " -t " + trialName +
                //               " -f " + std::to_string(tagHz) +
                //               " -c " + std::to_string(frameCount) +
                //               " -n " + std::to_string(captureCount);

                // system(command.c_str());

                reached_targets[i] = true;
            }
        }
    }

    rclcpp::Subscription<inertial_sense_ros2::msg::GPS>::SharedPtr subscription_;
    std::vector<std::pair<double, double>> target_coordinates_;
};

int main(int argc, char *argv[])
{
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<GPSSubscriber>());
    // rclcpp::shutdown();

    std::string fullDataPath = "/data"; // Update the path to the data folder
    std::string airFramesName = "temp";
    std::string trialName = "test";
    double tagHz = 80;
    int frameCount = 2000;
    int captureCount = 1;

    std::string command = "chmod +x ./wadar && ./wadar wadarTagTest -s " + fullDataPath +
                          " -b " + airFramesName +
                          " -t " + trialName +
                          " -f " + std::to_string(tagHz) +
                          " -c " + std::to_string(frameCount) +
                          " -n " + std::to_string(captureCount);

    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != NULL) {
        std::cout << "Current working directory: " << cwd << std::endl;
    } else {
        perror("getcwd() error");
        return 1;
    }

    std::cout << command << std::endl;

    system(command.c_str());

    return 0;
}
