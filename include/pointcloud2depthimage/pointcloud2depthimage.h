#ifndef __POINTCLOUD2DEPTHIMAGE_H
#define __POINTCLOUD2DEPTHIMAGE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>

class PointCloud2DepthImage
{
    public:
        PointCloud2DepthImage(void);

            typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
            typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

            void process(void);

            void pc_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);

            void pointcloud2depthimage(void); 

            std::vector<float> tranrate2polarcoodinates(const float&, const float&, const float&);


    private:

        double HZ;

        int HEGHT;
        int WIDTH;

        double VERTICAL_FOV;
        double HORIZONTAL_FOV;

        bool pc_callback_flag;

        PointCloudPtr input_pc_ptr {new PointCloud};

        ros::NodeHandle nh;
        ros::Subscriber pointcloud_sub;
        ros::Publisher depthimage_pub;


};

#endif // __POINTCLOUD2DEPTHIMAGE
