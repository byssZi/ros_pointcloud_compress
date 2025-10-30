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
class LidarDecoder
{
public:
    LidarDecoder(ros::NodeHandle& nh) : decoder_() // 初始化 DecoderModule
    {
        // 从参数服务器读取话题名称
        nh.param<std::string>("lidar_restored_topic", lidar_topic_, "/lidar_points");
        nh.param<std::string>("encoded_topic", encoded_topic_, "/encoded_data");
        nh.param<int>("q_level", q_level_, 0);

        // 订阅与发布
        pub_lidar_ = nh.advertise<sensor_msgs::PointCloud2>(lidar_topic_, 1);
        sub_encoded_ = nh.subscribe(encoded_topic_, 1, &LidarDecoder::decodeCallback, this);

        ROS_INFO("Subscribed to: %s", encoded_topic_.c_str());
        ROS_INFO("Publishing decode data on: %s", lidar_topic_.c_str());

    }

private:
    ros::Publisher pub_lidar_;
    ros::Subscriber sub_encoded_;
    std::string lidar_topic_;
    std::string encoded_topic_;
    int q_level_;
    DecoderModule decoder_;

    void decodeCallback(const msg_all::CompressLidarConstPtr& msg)
    {
        // 转换为 PCL 点云格式
        std::vector<char> decoded_data(msg->point_data.data.begin(), msg->point_data.data.end());
        ROS_INFO("Received %zu bytes", decoded_data.size());

        // Decode point cloud
        decoder_ = DecoderModule(decoded_data, 4, true, q_level_);
        auto restored_pcloud = decoder_.restored_pcloud;

        // Convert restored data to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr restored_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        restored_pcl_cloud->width = restored_pcloud.size();
        restored_pcl_cloud->height = 1;
        restored_pcl_cloud->is_dense = true;
        restored_pcl_cloud->points.resize(restored_pcloud.size());

        for (size_t i = 0; i < restored_pcloud.size(); ++i)
        {
            restored_pcl_cloud->points[i].x = restored_pcloud[i].x;
            restored_pcl_cloud->points[i].y = restored_pcloud[i].y;
            restored_pcl_cloud->points[i].z = restored_pcloud[i].z;
        }

          // 创建 sensor_msgs::PointCloud2 消息
        sensor_msgs::PointCloud2 cloud_msg;

        // 将 PCL 点云转换为 ROS 消息
        pcl::toROSMsg(*restored_pcl_cloud, cloud_msg);

        // 设置消息的帧 ID 和时间戳
        cloud_msg.header = msg->header;

        // 发布消息
        pub_lidar_.publish(cloud_msg);
  
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_decode_node");
    ros::NodeHandle nh("~");

    LidarDecoder decoder(nh);

    ros::spin();
    return 0;
}
