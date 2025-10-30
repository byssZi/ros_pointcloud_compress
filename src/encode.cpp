#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt8MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <msg_all/CompressLidar.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <thread>  
#include <chrono>  
#include "rcpcc/modules/encoder_module.h"
#include "rcpcc/modules/decoder_module.h"
#include "rcpcc/utils/struct.h"
#include "rcpcc/utils/utils.h"
class LidarEncoder
{
public:
    LidarEncoder(ros::NodeHandle& nh) : encoder_() // 初始化 EncoderModule
    {
        // 从参数服务器读取话题名称
        nh.param<std::string>("lidar_topic", lidar_topic_, "/lidar_points");
        nh.param<std::string>("encoded_topic", encoded_topic_, "/encoded_data");
        nh.param<int>("q_level", q_level_, 0);

        // 订阅与发布
        sub_lidar_ = nh.subscribe(lidar_topic_, 1, &LidarEncoder::lidarCallback, this);
        pub_encoded_ = nh.advertise<msg_all::CompressLidar>(encoded_topic_, 1);

        ROS_INFO("Subscribed to: %s", lidar_topic_.c_str());
        ROS_INFO("Publishing encoded data on: %s", encoded_topic_.c_str());

    }

private:
    ros::Subscriber sub_lidar_;
    ros::Publisher pub_encoded_;
    std::string lidar_topic_;
    std::string encoded_topic_;
    int q_level_;
    EncoderModule encoder_;

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // 转换为 PCL 点云格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        ROS_INFO("Received PointCloud with %zu points", cloud->points.size());

        // ================================
        // 示例：将点云简单编码为字符串
        // 实际项目中可改为特征提取或压缩
        // ================================
        // Load point cloud using project's loading functions
        std::vector<point_cloud> pcloud_data;
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            point_cloud pc;
            pc.x = cloud->points[i].x;
            pc.y = cloud->points[i].y;
            pc.z = cloud->points[i].z;
            pcloud_data.emplace_back(pc);
        }
        encoder_ = EncoderModule(4, q_level_);
        // Encode point cloud
        std::vector<char> encoded_data = encoder_.encodeToData(pcloud_data, true);

        // 打包为 ROS 消息
        msg_all::CompressLidar encoded_msg;
        encoded_msg.header = msg->header;
        encoded_msg.point_data.data.resize(encoded_data.size());
        for (size_t i = 0; i < encoded_data.size(); ++i)
            encoded_msg.point_data.data[i] = static_cast<uint8_t>(encoded_data[i]);

        pub_encoded_.publish(encoded_msg);

        // Calculate compression ratio
        double compression_ratio = static_cast<double>(cloud->points.size()) / encoded_data.size();

        ROS_INFO("Compression results:");
        ROS_INFO("Original size: (%zu bytes)", cloud->points.size());
        ROS_INFO("Compressed size: (%zu bytes)", encoded_data.size());
        ROS_INFO_STREAM("  Compression ratio: " << std::fixed << std::setprecision(2) << compression_ratio << ":1");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_encode_node");
    ros::NodeHandle nh("~");

    LidarEncoder encoder(nh);

    ros::spin();
    return 0;
}
