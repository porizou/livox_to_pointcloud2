#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <livox_ros_driver2/CustomMsg.h>

class LivoxToPCL2
{
public:
    LivoxToPCL2()
    {
        pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output_topic", 1);
        livox_sub_ = nh_.subscribe<livox_ros_driver2::CustomMsg>("input_topic", 1, &LivoxToPCL2::callback, this);
    }

    void callback(const livox_ros_driver2::CustomMsg::ConstPtr& msg)
    {
        sensor_msgs::PointCloud2 pcl_msg = livoxToPCL2(*msg);
        pcl_pub_.publish(pcl_msg);
    }

    sensor_msgs::PointCloud2 livoxToPCL2(const livox_ros_driver2::CustomMsg& input)
    {
        sensor_msgs::PointCloud2 output;

        output.header = input.header;
        output.height = 1;
        output.width = input.point_num;
        output.is_bigendian = false;
        output.is_dense = false;

        sensor_msgs::PointCloud2Modifier pcd_modifier(output);

        pcd_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                             "y", 1, sensor_msgs::PointField::FLOAT32,
                                             "z", 1, sensor_msgs::PointField::FLOAT32,
                                             "reflectivity", 1, sensor_msgs::PointField::UINT8);

        sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_reflectivity(output, "reflectivity");

        for (const auto& point : input.points)
        {
            *iter_x = point.x;
            *iter_y = point.y;
            *iter_z = point.z;
            *iter_reflectivity = point.reflectivity;
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_reflectivity;
        }

        return output;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pcl_pub_;
    ros::Subscriber livox_sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "livox_to_pcl2");

    LivoxToPCL2 converter;

    ros::spin();

    return 0;
}

