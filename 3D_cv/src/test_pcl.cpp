#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

bool has_saved = false;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if (has_saved) return;

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);

    std::string filename = "/home/adrian/catkin_ws/src/3D_cv/src/inputs/test1_single.pcd";
    if (pcl::io::savePCDFileASCII(filename, cloud) == 0)
    {
        ROS_INFO("Saved single point cloud to %s", filename.c_str());
        has_saved = true;  // Prevent future saves
    }
    else
    {
        ROS_ERROR("Failed to save point cloud.");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "single_pointcloud_saver");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/rgbd_camera_depth/depth/points", 1, cloudCallback);

    ROS_INFO("Waiting for first point cloud... will save only one.");
    ros::spin();
    return 0;
}
