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

bool has_saved = false;
std::array<std::array<double, 3>,4> centroid_cylinder;

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


void process_pcl_data(const std::string& input_pcd, const std::string& output_pcd, tf2_ros::Buffer& tf_buffer)
{
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader cloud_reader;
    cloud_reader.read(input_pcd, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);


    //voxel filter (downsampling)
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.001f, 0.001f, 0.001f); // Set the voxel size //play
    voxel_filter.filter(*cloud_filtered);
    
    // plane segmentation
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

    seg.setOptimizeCoefficients(true); //image in 3d except for the plane
    seg.setModelType(pcl::SAC_RANSAC);
    seg.setMethodType(pcl::SACMODEL_PLANE);
    
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(0.01); 
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers_plane, *coefficients_plane);

    pcl::ExtractIndices<pcl::PointXYZ> neg_plane_extracted;
    neg_plane_extracted.setInputCloud(cloud_filtered);
    neg_plane_extracted.setIndices(inliers_plane);
    neg_plane_extracted.setNegative(true); // Extract everything except the plane
    neg_plane_extracted.filter(*cloud_filtered);

    // pass through filter
    // crop the point cloud to a specific region

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.65); // Set the z limits //play
    pass.filter(*cloud_filtered);

    // pass.setFilterFieldName("x");
    // pass.setFilterLimits(-0.15, 0.15); // Set the x limits //play
    // pass.filter(*cloud_filtered);

    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(-10.0, 0.2); // Set the y limits //play
    // pass.filter(*cloud_filtered);

    // remove outliers using statistical outlier removal (remove noise)
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(30); // Number of nearest neighbors to use for mean distance estimation
    sor.setStddevMulThresh(1.0); // Standard deviation multiplier threshold
    sor.filter(*cloud_filtered);

    // segment a cylinder using RANSAC
    // get the normals of the point cloud (how the points are oriented in space)
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimator;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud_filtered);
    normal_estimator.setKSearch(30); // Evaluate number of neighbors to use for normal estimation //play
    normal_estimator.compute(*cloud_normals);

    // segmentation of the cylinder from the normals
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::PointNormal> cylinder_segmentation;
    cylinder_segmentation.setOptimizeCoefficients(true);
    cylinder_segmentation.setModelType(pcl::SACMODEL_CYLINDER);
    cylinder_segmentation.setMethodType(pcl::SAC_RANSAC);
    cylinder_segmentation.setNormalDistanceWeight(0.05); // Set the distance threshold for cylinder
    cylinder_segmentation.setMaxIterations(10000); // Set the maximum number of iterations for RANSAC
    cylinder_segmentation.setDistanceThreshold(0.005); // Set the distance threshold for cylinder //play
    cylinder_segmentation.setRadiusLimits(0.015, 0.021); // Set the radius limits for cylinder  //play


    
    // get the inliers and coefficients of the cylinder
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    

    // extract the cylinder from the point cloud
    pcl::ExtractIndices<pcl::PointXYZ> cylinder_extracted;
    pcl::ExtractIndices<pcl::PointNormal>   cylinder_indices_extractor_temp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_cylinders(new pcl::PointCloud<pcl::PointXYZ>);

    int l=0;
    int row_cy=0;
    while(true)
    {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);


    cylinder_segmentation.setInputCloud(cloud_filtered);
    cylinder_segmentation.setInputNormals(cloud_normals);
    cylinder_segmentation.segment(*inliers_cylinder, *coefficients_cylinder);

    cylinder_extracted.setInputCloud(cloud_filtered);
    cylinder_extracted.setIndices(inliers_cylinder);
    cylinder_extracted.setNegative(false); // Extract the cylinder
    cylinder_extracted.filter(*cylinder_cloud);

    

    if(!cylinder_cloud->points.empty()){
        // compute the centroid of the cylinder
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cylinder_cloud, centroid);

        if (centroid[2] < 1.2 || centroid[2] > 1.4) {
            ROS_WARN("Skipping this circle due to height.");
            // Remove the high circle from input clouds
            cylinder_extracted.setNegative(true);
            cylinder_extracted.filter(*cloud_filtered);

            // processing normals
            cylinder_indices_extractor_temp.setInputCloud(cloud_normals);
            cylinder_indices_extractor_temp.setIndices(inliers_cylinder);
            cylinder_indices_extractor_temp.setNegative(true);
            cylinder_indices_extractor_temp.filter(*cloud_normals);

            continue;
        
        }
        
        else if(cylinder_cloud->points.size() > 90){
            *all_cylinders += *cylinder_cloud;
            l++;
        }

        geometry_msgs::PointStamped point_cam;
        point_cam.header.stamp = ros::Time(0);  // Use latest TF
        point_cam.header.frame_id = "rgbd_camera_depth_optical_frame";  // <-- Replace with your actual camera frame
        point_cam.point.x = centroid[0];
        point_cam.point.y = centroid[1];
        point_cam.point.z = centroid[2];

        // print the centroid of the cylinder
        // std::cout << "Centroid of the cylinder in the camera frame: ("
        //         << centroid[0] << ", " 
        //         << centroid[1] << "," 
        //         << centroid[2] << ")" << std::endl;

        // Transform the point to the world frame
        try {
            geometry_msgs::PointStamped point_world;
            point_world = tf_buffer.transform(point_cam, "world", ros::Duration(1.0));

            

            centroid_cylinder[row_cy][0]=point_world.point.x;
            centroid_cylinder[row_cy][1]=point_world.point.y;
            centroid_cylinder[row_cy][2]=point_world.point.z;
            
            std::cout << "Centroid cylinders in world frame: ("
                    << centroid_cylinder[row_cy][0] << ", "
                    << centroid_cylinder[row_cy][1] << ", "
                    << centroid_cylinder[row_cy][2] << ")" << std::endl;


        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF transform failed: %s", ex.what());
        }
        if (centroid_cylinder[row_cy][2] < 1.1) {
            ROS_WARN("Skipping this circle due to height.");
            // Remove the high circle from input clouds
            cylinder_extracted.setNegative(true);
            cylinder_extracted.filter(*cloud_filtered);

            // processing normals
            cylinder_indices_extractor_temp.setInputCloud(cloud_normals);
            cylinder_indices_extractor_temp.setIndices(inliers_cylinder);
            cylinder_indices_extractor_temp.setNegative(true);
            cylinder_indices_extractor_temp.filter(*cloud_normals);

            continue;
        
        }



        cylinder_extracted.setNegative(true);
        cylinder_extracted.filter(*cloud_filtered);

        // processing normals
        cylinder_indices_extractor_temp.setInputCloud(cloud_normals);
        cylinder_indices_extractor_temp.setIndices(inliers_cylinder);
        cylinder_indices_extractor_temp.setNegative(true);
        cylinder_indices_extractor_temp.filter(*cloud_normals);
        row_cy++;
    }
    else{
        std::cout << "No more cylinders. Terminating segmentation." << std::endl;
        break;
    }

    }


    pcl::PCDWriter cloud_writer;
    cloud_writer.write<pcl::PointXYZ>(output_pcd, *cloud, false);
    // cloud_writer.write<pcl::PointXYZ>(output_pcd, *cloud_filtered, false);


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

    process_pcl_data(input_pcd, output_pcd,tf_buffer);


    return 0;
}