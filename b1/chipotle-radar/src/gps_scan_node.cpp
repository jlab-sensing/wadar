#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "inertial-sense-sdk/msg/gps.hpp" 
#include <iostream>


#define COORDINATE_TOLERANCE 0.0001

class GPSSubscriber : public rclcpp::Node
{
public:
    GPSSubscriber()
        : Node("gps_subscriber_node")
    {
        subscription_ = this->create_subscription<inertial-sense-sdk::msg::GPS>(
            "GPS_rel/", 10, std::bind(&GPSSubscriber::gps_callback, this, std::placeholders::_1));
        
        target_coordinates_ = {
            {36.956999, -122.058549},
            {37.774929, -122.419416},
            {34.052235, -118.243683}
        };
    }

private:
    void gps_callback(const inertial-sense-sdk::msg::GPS::SharedPtr msg)
    {
        for (const auto& target : target_coordinates_)
        {
            double target_latitude = target.first;
            double target_longitude = target.second;

            if (abs(msg->latitude - target_latitude) < COORDINATE_TOLERANCE &&
                abs(msg->longitude - target_longitude) < COORDINATE_TOLERANCE)
            {
                RCLCPP_INFO(this->get_logger(), "Target location reached: Latitude: %f, Longitude: %f",
                            msg->latitude, msg->longitude);

                char localDataPath[] = "../data"; // TODO: Update the path to the data folder
                char airFramesName[] = "temp";
                char trialName[] = "test";
                double tagHz = 80;
                int frameCount = 100;
                int captureCount = 10;

                double result = wadarTagTest(localDataPath, airFramesName, trialName, tagHz, frameCount, captureCount);

                RCLCPP_INFO(this->get_logger(), "wadarTagTest result: %f", result);
                std::cout << "wadarTagTest result: " << result << std::endl;

                // Ensure the file is created if it doesn't exist
                std::ofstream file_check("gps_log.csv", std::ios_base::app);
                if (!file_check)
                {
                    std::ofstream create_file("gps_log.csv");
                    if (!create_file)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to create log file");
                        return;
                    }
                    create_file.close();
                }
                file_check.close();

                std::ofstream log_file("gps_log.csv", std::ios_base::app);
                if (log_file.is_open())
                {
                    log_file << msg->latitude << "," << msg->longitude << "," << result << "\n";
                    log_file.close();
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open log file");
                }

                // return;
            }
        }
    }

    rclcpp::Subscription<inertial-sense-sdk::msg::GPS>::SharedPtr subscription_;
    std::vector<std::pair<double, double>> target_coordinates_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSSubscriber>());
    rclcpp::shutdown();
    return 0;
}
