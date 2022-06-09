#ifndef __POINTCLOUD2DEPTHIMAGE_H
#define __POINTCLOUD2DEPTHIMAGE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class PointCloud2DepthImage
{
    public:
        PointCloud2DepthImage(void);

            void process(void);

            void pc_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

            void pointcloud2depthimage(const sensor_msgs::PointCloud2& pc); 

    private:

        double HZ;
        bool pc_callback_flag;

        sensor_msgs::PointCloud2 input_pc;

        ros::NodeHandle nh;
        ros::Subscriber pointcloud_subscriber;



};

#endif // __POINTCLOUD2DEPTHIMAGE