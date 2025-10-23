#include <ros/ros.h>
#include <iostream>
#include <math.h>


#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/common/centroid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <geometry_msgs/PointStamped.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/transforms.h>



bool has_saved = false;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if (has_saved) return;

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);

    std::string filename = "/home/adrian/catkin_ws/src/3D_cv/src/inputs/test2.pcd";
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
void process_hole(const std::string& input_pcd, const std::string& output_pcd, tf2_ros::Buffer& tf_buffer)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader cloud_reader;
    cloud_reader.read(input_pcd, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 1. Voxel downsampling
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.001f, 0.001f, 0.001f);
    voxel_filter.filter(*cloud_filtered);

    // 2. Remove the dominant plane
    pcl::SACSegmentation<pcl::PointXYZ> seg_plane;
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);

    seg_plane.setOptimizeCoefficients(true);
    seg_plane.setModelType(pcl::SACMODEL_PLANE);
    seg_plane.setMethodType(pcl::SAC_RANSAC);
    seg_plane.setMaxIterations(200);
    seg_plane.setDistanceThreshold(0.01);
    seg_plane.setInputCloud(cloud_filtered);
    seg_plane.segment(*inliers_plane, *coefficients_plane);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    // 3. Pass-through filter on Z
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.65);
    pass.filter(*cloud_filtered);

    // 4. Outlier removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);


    // 1️⃣ Transform to world frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZ>);
    geometry_msgs::TransformStamped transform_stamped;

    try {
        transform_stamped = tf_buffer.lookupTransform(
            "world",  // target frame
            cloud_filtered->header.frame_id="rgbd_camera_depth_optical_frame", // source frame
            ros::Time(0),
            ros::Duration(1.0)
        );

        Eigen::Matrix4f transform;
        pcl_ros::transformAsMatrix(transform_stamped.transform, transform);
        pcl::transformPointCloud(*cloud_filtered, *cloud_world, transform);

    } catch (tf2::TransformException &ex) {
        ROS_WARN("Transform failed: %s", ex.what());
        return;
    }

    // 5. Euclidean clustering to identify objects
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.06);  // 2 cm tolerance between points in the same object
    ec.setMinClusterSize(10);      // min points in a cluster
    ec.setMaxClusterSize(50000);   // max points in a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_world);
    ec.extract(cluster_indices);

pcl::PointCloud<pcl::PointXYZ>::Ptr top_surfaces(new pcl::PointCloud<pcl::PointXYZ>);

for (const auto& indices : cluster_indices) {
    // Find max Z in this cluster
    float max_z = -std::numeric_limits<float>::max();
    for (int idx : indices.indices) {
        if (cloud_world->points[idx].z > max_z)
            max_z = cloud_world->points[idx].z;
    }

    // Keep only points within 1 cm of max Z
    for (int idx : indices.indices) {
        if (std::abs(cloud_world->points[idx].z - max_z) <= 0.04f) {
            top_surfaces->points.push_back(cloud_world->points[idx]);
        }
    }
}

    top_surfaces->width = top_surfaces->points.size();
    top_surfaces->height = 1;
    top_surfaces->is_dense = true;


   

    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>);
    *remaining = *top_surfaces; // All top surface points

// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr all_circles(new pcl::PointCloud<pcl::PointXYZ>);

while (!remaining->empty()) {
    // Compute normals
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(remaining);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*cloud_normals);

    // Circle segmentation
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::PointNormal> circle_segmentation;
    circle_segmentation.setOptimizeCoefficients(true);
    circle_segmentation.setModelType(pcl::SACMODEL_CIRCLE2D);
    circle_segmentation.setMethodType(pcl::SAC_RANSAC);
    circle_segmentation.setNormalDistanceWeight(0.05);
    circle_segmentation.setMaxIterations(5000);
    circle_segmentation.setDistanceThreshold(0.005);
    circle_segmentation.setRadiusLimits(0.018, 0.027);

    pcl::PointIndices::Ptr inliers_circle(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_circle(new pcl::ModelCoefficients);

    circle_segmentation.setInputCloud(remaining);
    circle_segmentation.setInputNormals(cloud_normals);
    circle_segmentation.segment(*inliers_circle, *coefficients_circle);

    if (inliers_circle->indices.empty()) {
        break; // No more circles found
    }

    // Extract this circle's points
    pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(remaining);
    extractor.setIndices(inliers_circle);
    extractor.setNegative(false);
    extractor.filter(*circle_cloud);

    // Add to all_circles
    *all_circles += *circle_cloud;

    // Remove these points from remaining for next iteration
    extractor.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    extractor.filter(*tmp);
    remaining.swap(tmp);
}

std::cout << "Total circles detected: " << all_circles->size() << " points" << std::endl;

// Save detected circles
pcl::PCDWriter writer;
writer.write<pcl::PointXYZ>("detected_circles.pcd", *all_circles, false);

}









int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_process_node");
    ros::NodeHandle nh;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);  // start the TF listener


    ros::Subscriber sub = nh.subscribe("/rgbd_camera_depth/depth/points", 1, cloudCallback);

    
    ros::Duration(5.0).sleep();  // wait a bit for the message



    while (ros::ok())
    {
        if(has_saved)
        {
            ROS_INFO("Point cloud saved successfully, proceeding with processing.");
            break;
        }
        else
        {
            ROS_INFO("Waiting for point cloud to be saved...");
            ros::spinOnce();  // allow first callback
        }
    }



    std::string path_input="/home/adrian/catkin_ws/src/3D_cv/src/inputs/";
    std::string path_output="/home/adrian/catkin_ws/src/3D_cv/src/outputs/";
    std::string input_pcd = path_input + std::string("test2.pcd");
    std::string output_pcd = path_output + std::string("filtered_points.pcd");

    process_hole(input_pcd, output_pcd,tf_buffer);


    return 0;
}