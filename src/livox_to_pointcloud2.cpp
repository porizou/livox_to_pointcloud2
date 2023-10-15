#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

using namespace std::chrono_literals;

class LivoxToPointCloud2 : public rclcpp::Node
{
public:
    LivoxToPointCloud2() : Node("livox_converter")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("converted_pointcloud2", 10);
        subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "livox_pointcloud", 10, std::bind(&LivoxToPointCloud2::callback, this, std::placeholders::_1));
    }

private:
    void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud2 output;
        output.header = msg->header;

        // Set the PointCloud2 fields
        output.fields.resize(6);
        // x field
        output.fields[0].name = "x";
        output.fields[0].offset = 0;
        output.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output.fields[0].count = 1;

        // y field
        output.fields[1].name = "y";
        output.fields[1].offset = 4;
        output.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output.fields[1].count = 1;

        // z field
        output.fields[2].name = "z";
        output.fields[2].offset = 8;
        output.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output.fields[2].count = 1;

        // intensity field
        output.fields[3].name = "intensity";
        output.fields[3].offset = 12;
        output.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output.fields[3].count = 1;

        // tag field
        output.fields[4].name = "tag";
        output.fields[4].offset = 16;
        output.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
        output.fields[4].count = 1;

        // line field
        output.fields[5].name = "line";
        output.fields[5].offset = 17;
        output.fields[5].datatype = sensor_msgs::msg::PointField::UINT8;
        output.fields[5].count = 1;

        output.point_step = 18; // Size of a point in bytes
        output.row_step = output.point_step * msg->point_num;
        output.data.resize(output.row_step);

        uint8_t* raw_data_ptr = output.data.data();

        for (const auto& point : msg->points)
        {
            *(reinterpret_cast<float*>(raw_data_ptr + 0)) = point.x;
            *(reinterpret_cast<float*>(raw_data_ptr + 4)) = point.y;
            *(reinterpret_cast<float*>(raw_data_ptr + 8)) = point.z;
            *(reinterpret_cast<float*>(raw_data_ptr + 12)) = static_cast<float>(point.reflectivity);
            *(raw_data_ptr + 16) = point.tag;
            *(raw_data_ptr + 17) = point.line;

            raw_data_ptr += output.point_step;
        }

        output.width = msg->point_num;
        output.height = 1; // Unordered point cloud
        output.is_bigendian = false;
        output.is_dense = true;

        publisher_->publish(output);
    }

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxToPointCloud2>());
    rclcpp::shutdown();
    return 0;
}

