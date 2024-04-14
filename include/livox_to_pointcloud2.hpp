#ifndef LIVOX_TO_POINTCLOUD2_HPP
#define LIVOX_TO_POINTCLOUD2_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_field.hpp"

class LivoxToPointCloud2 : public rclcpp::Node
{
public:
    LivoxToPointCloud2();

private:
    void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

#endif  // LIVOX_TO_POINTCLOUD2_HPP
