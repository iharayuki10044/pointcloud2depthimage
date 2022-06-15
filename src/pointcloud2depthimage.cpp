#include "pointcloud2depthimage/pointcloud2depthimage.h"

PointCloud2DepthImage::PointCloud2DepthImage(void)
:nh("~")
{
    // Initialize ROS node
    nh.param("HZ", HZ, 10.0);
    nh.param("GRID_HEGHT", GRID_HEGHT, 512);
    nh.param("GRID_WIDTH", GRID_WIDTH, 512);
    nh.param("VERTICAL_FOV", VERTICAL_FOV, M_PI/2.0);
    nh.param("HORIZONTAL_FOV", HORIZONTAL_FOV, M_PI);
    nh.param("MAX_DEPTH", MAX_DEPTH, 10.0);

    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "GRID_HEGHT: " << GRID_HEGHT << std::endl;
    std::cout << "GRID_WIDTH: " << GRID_WIDTH << std::endl;
    std::cout << "VERTICAL_FOV: " << VERTICAL_FOV << std::endl;
    std::cout << "HORIZONTAL_FOV: " << HORIZONTAL_FOV << std::endl;
    std::cout << "MAX_DEPTH: " << MAX_DEPTH << std::endl;

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
    cv::Mat depth_image(GRID_HEGHT, GRID_WIDTH, CV_32FC1, cv::Scalar(0));

    std::vector<float> polar_coordinates(3, 0);
    // r, theta, phi

    for(auto& point : input_pc_ptr->points){
        polar_coordinates[0] = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
        polar_coordinates[1] = acos(point.z/polar_coordinates[0]);
        polar_coordinates[2] = acos(point.x/sqrt(point.x*point.x + point.y*point.y));

        if((abs(polar_coordinates[1]) < VERTICAL_FOV)&&(abs(polar_coordinates[2]) < HORIZONTAL_FOV)){
            int row = GRID_HEGHT/2 + polar_coordinates[1]*GRID_HEGHT/VERTICAL_FOV;
            int col = GRID_WIDTH/2 + polar_coordinates[2]*GRID_WIDTH/HORIZONTAL_FOV;
            depth_image.at<float>(row, col) = polar_coordinates[0] /MAX_DEPTH; 
        
        }

    }

    depthimage_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_image).toImageMsg());
}

std::vector<float> transrate2polarcoodinates(const float& x, const float& y, const float& z)
{
    std::vector<float> polar_coordinates(3, 0);

    polar_coordinates[0] = sqrt(x*x + y*y + z*z);
    polar_coordinates[1] = acos(z/polar_coordinates[0]);
    polar_coordinates[2] = acos(x/sqrt(x*x + y*y));

    return polar_coordinates;
}


void PointCloud2DepthImage::process(void)
{
    ros::Rate loop_rate(HZ);
    pc_callback_flag = false;
    std::cout << "PointCloud2DepthImage::process()" << std::endl;
    while(ros::ok())
    {
        std::cout << "pc_callback_flag: " << pc_callback_flag << std::endl;
 
        if(pc_callback_flag)
        {
            pointcloud2depthimage();
            std::cout << "--published depth image--" << std::endl;
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
