#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <inertial_sense_ros2/msg/gps.hpp>
#include <inertial_sense_ros2/msg/didins1.hpp>
#include <iostream>
#include <fstream>
#include <vector>

#define COORDINATE_TOLERANCE 0.0001
#define GPS_RADAR_DIST 0.25

class GPSSubscriber : public rclcpp::Node
{
public:
    GPSSubscriber()
        : Node("gps_subscriber_node")
    {
        gps_subscription_ = this->create_subscription<inertial_sense_ros2::msg::GPS>(
            "/gps1/pos_vel", 10, std::bind(&GPSSubscriber::gps_callback, this, std::placeholders::_1));

        ins_subscription_ = this->create_subscription<inertial_sense_ros2::msg::DIDINS1>(
            "/did_ins1", 10, std::bind(&GPSSubscriber::ins_callback, this, std::placeholders::_1));

        tag_coordinates_ = {
            {36.956991, -122.058694},
            {36.956938, -122.058536},
            {34.052235, -118.243683}};

        tag_names_ = {
            "Target 1",
            "Target 2",
            "Target 3"};
    }

private:
    void gps_callback(const inertial_sense_ros2::msg::GPS::SharedPtr msg)
    {
        gps_data_ = msg;
        check_tags();
    }

    void ins_callback(const inertial_sense_ros2::msg::DIDINS1::SharedPtr msg)
    {
        ins_data_ = msg;
        check_tags();
    }

    void check_tags()
    {
        if (!gps_data_ || !ins_data_)
            return;

        static std::vector<bool> reached_targets(tag_coordinates_.size(), false);

        for (size_t i = 0; i < tag_coordinates_.size(); ++i)
        {
            if (reached_targets[i])
                continue;
            double yaw = ins_data_->theta[2];
            double tag_latitude = tag_coordinates_[i].first * sin(yaw) + GPS_RADAR_DIST * sin(yaw);
            double tag_longitude = tag_coordinates_[i].second * cos(yaw) + GPS_RADAR_DIST * cos(yaw);

            if (abs(gps_data_->latitude - tag_latitude) < COORDINATE_TOLERANCE &&
                abs(gps_data_->longitude - tag_longitude) < COORDINATE_TOLERANCE)
            {
                RCLCPP_INFO(this->get_logger(), "Target location reached: Latitude: %f, Longitude: %f",
                            gps_data_->latitude, gps_data_->longitude);

                // std::string fullDataPath = "/data"; // Update the path to the data folder
                // std::string airFramesName = "temp";
                // double tagHz = 80;
                // int frameCount = 2000;
                // int captureCount = 1;

                // std::string command = "./wadar wadarTagTest -s " + fullDataPath +
                //                       " -b " + airFramesName +
                //                       " -t " + tag_names_[i] +
                //                       " -f " + std::to_string(tagHz) +
                //                       " -c " + std::to_string(frameCount) +
                //                       " -n " + std::to_string(captureCount);

                // system(command.c_str());

                reached_targets[i] = true;
            }
        }
    }

    rclcpp::Subscription<inertial_sense_ros2::msg::GPS>::SharedPtr gps_subscription_;
    rclcpp::Subscription<inertial_sense_ros2::msg::DIDINS1>::SharedPtr ins_subscription_;
    inertial_sense_ros2::msg::GPS::SharedPtr gps_data_;
    inertial_sense_ros2::msg::DIDINS1::SharedPtr ins_data_;
    std::vector<std::pair<double, double>> tag_coordinates_;
    std::vector<std::string> tag_names_;
};

int main(int argc, char *argv[])
{
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<GPSSubscriber>());
    // rclcpp::shutdown();

    std::string fullDataPath = "ericdvet@192.168.7.1:/home/ericdvet/hare-lab/dev_ws/src/wadar/signal_processing/data"; // Update the path to the data folder
    std::string airFramesName = "2024-10-16__test_C1.frames";
    std::string trialName = "test2";
    double tagHz = 80;
    int frameCount = 200;
    int captureCount = 1;
    double captureDepth = 0.25;

    std::string command = "cd wadar/signal_processing/ && ./wadar wadar -s " + fullDataPath +
                          " -b " + airFramesName +
                          " -t " + trialName +
                          " -f " + std::to_string(tagHz) +
                          " -c " + std::to_string(frameCount) +
                          " -n " + std::to_string(captureCount) +
                          " -d " + std::to_string(captureDepth);
    // std::string command = "ls";

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
