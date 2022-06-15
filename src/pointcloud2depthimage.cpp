#include "pointcloud2depthimage/pointcloud2depthimage.h"

PointCloud2DepthImage::PointCloud2DepthImage(void)
:nh("~")
{
    // Initialize ROS node
    nh.param("HZ", HZ, 10.0);
    nh.param("HEGHT", HEGHT, 512);
    nh.param("WIDTH", WIDTH, 512);

    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "HEGHT: " << HEGHT << std::endl;
    std::cout << "WIDTH: " << WIDTH << std::endl;

    pointcloud_sub = nh.subscribe("/velodyne_points", 10 , &PointCloud2DepthImage::pc_callback, this);
    depthimage_pub = nh.advertise<sensor_msgs::Image>("/depthimage", 1);

}

void PointCloud2DepthImage::pc_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, *input_pc_ptr);
    pc_callback_flag = true;
}

void PointCloud2DepthImage::pointcloud2depthimage(void)
{
    cv::Mat depth_image(HEGHT, WIDTH, CV_32FC1, cv::Scalar(0));

    // for(int i=0; i<pc.size(); i++)
    // {
    //     float x = pc.points[i].x;
    //     float y = pc.points[i].y;
    //     float z = pc.points[i].z;

    // }


    // Convert PointCloud2 to PointCloud
}

// void PointCloud2DepthImage::pointcloud2depthimage(const sensor_msgs::PointCloud2& pc)
// {
    // Convert the point cloud to pcl format
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromROSMsg(pc, *cloud);

    // Create a depth image
    // cv::Mat depth_image(HEGHT, WIDTH, CV_32FC1);

    // Iterate through the point cloud and fill in the depth image
    // for (int v = 0; v < cloud->height; v++)
    // {
    //     for (int u = 0; u < cloud->width; u++)
    //     {
    //         pcl::PointXYZ pt = cloud->at(u, v);
    //         depth_image.at<float>(v, u) = pt.z;
    //     }
    // }

    // Convert the depth image to a ROS image
    // cv_bridge::CvImage cv_image;
    // cv_image.header.stamp = ros::Time::now();
    // cv_image.header.frame_id = "velodyne";
    // cv_image.encoding = "32FC1";
    // cv_image.image = depth_image;

    // Publish the depth image
    // depthimage_pub.publish(cv_image.toImageMsg());
// }

void PointCloud2DepthImage::process(void)
{
    ros::Rate loop_rate(HZ);
    while(ros::ok())
    {
        if(pc_callback_flag)
        {
            // pointcloud2depthimage();
            pc_callback_flag = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    } 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud2depthimage");
    PointCloud2DepthImage pointcloud2depthimage;
    pointcloud2depthimage.process();
    return 0;
}
