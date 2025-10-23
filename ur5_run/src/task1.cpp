#include <ros/ros.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// Gazebo Link Attacher
#include <gazebo_ros_link_attacher/Attach.h>
// Other necessary includes
#include <trajectory_msgs/JointTrajectory.h>
#include <shape_msgs/SolidPrimitive.h>
#include <std_msgs/String.h>

#include <array>

//laser scan
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>
#include <numeric>

//point cloud libraries
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


using namespace tf2;

const double tau = 2 * M_PI;

class PickAndPlace
{
public:
    PickAndPlace(ros::NodeHandle& nh)
        : planning_scene_interface(),
          move_group("ur5_manipulator"),
          gripper_group("robotiq_gripper"),
          tf_buffer(),
          tf_listener(tf_buffer)
    {
        move_group.setPlanningTime(30.0);
        move_group.setMaxVelocityScalingFactor(1.0);
        move_group.setMaxAccelerationScalingFactor(1.0);
        move_group.setGoalTolerance(0.03);
        move_group.setSupportSurfaceName("table1");
        move_group.allowReplanning(true);
        move_group.setNumPlanningAttempts(10); // Added retries
        attach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
        detach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
        // Wait for services
        if (!attach_client.waitForExistence(ros::Duration(20.0)))
            ROS_ERROR("Attach service not available!");
        if (!detach_client.waitForExistence(ros::Duration(20.0)))
            ROS_ERROR("Detach service not available!");

        // Subscribe to laser scan
        sub = nh.subscribe<sensor_msgs::LaserScan>(
            "/scan",
            10,
            &PickAndPlace::scanCallback,
            this
        );

        // Subscribe to depth camera point cloud
        sub1 = nh.subscribe(
            "/rgbd_camera_depth/depth/points",
            10,
            &PickAndPlace::cloudCallback,
            this
        );

        
        // input_pcd = path_input + std::string("test2.pcd");
        // output_pcd = path_output + std::string("filtered_points.pcd");

    }
    

    //lidar scan callback
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
    double angle_min = scan->angle_min;
    double angle_increment = scan->angle_increment;

    std::vector<float> right_ranges;
    std::vector<float> left_ranges;

    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        double angle_rad = angle_min + i * angle_increment;
        double angle_deg = angle_rad * 180.0 / M_PI;

        float range = scan->ranges[i];
        if (std::isinf(range)) range = 0.0f;

        if (angle_deg > -20.0 && angle_deg < -1.0)
        {
        right_ranges.push_back(range);
        }

        if (angle_deg > 92.0 && angle_deg < 109.0)
        {
        left_ranges.push_back(range);
        }
    }

    // Compute mean for right
    if (!right_ranges.empty())
    {
        float right_mean = std::accumulate(right_ranges.begin(), right_ranges.end(), 0.0f) / right_ranges.size();
        if (right_mean > 1.0 && right_mean < 2.0)
        {
        // ROS_INFO("Detecting a person on the right, mean range: %.2f, stop the robot!", right_mean);
        stop = true;
        }
        else
        {
        stop = false;
        }
    }

    // Compute mean for left
    if (!left_ranges.empty())
    {
        float left_mean = std::accumulate(left_ranges.begin(), left_ranges.end(), 0.0f) / left_ranges.size();
        if (left_mean >= 0.9 && left_mean < 2.0)
        {
        // ROS_INFO("Detecting a person on the left, mean range: %.2f, slow down the robot!", left_mean);
        slow_down= true;
        }
        else
        {
        slow_down= false;
        }
    }
    if (stop)
    {
        // move_group.setMaxVelocityScalingFactor(0.01);
        move_group.stop();
        ROS_INFO("Stopping the robot...");
        
    }
    else if (slow_down)
    {
        move_group.setMaxVelocityScalingFactor(0.2);
        ROS_INFO("Slowing down the robot...");
    }
    else
    {   
        move_group.setMaxVelocityScalingFactor(0.8);
        // ROS_INFO("No obstacles detected, robot can proceed.");
    }
    }

    // Save point cloud to a file
    void save(PickAndPlace& pick_and_place)
    {
        ros::Rate rate(10);
        ROS_INFO("Waiting for point cloud...");

        while (ros::ok())
        {
            if (has_saved)
            {
                ROS_INFO("Point cloud saved. Starting processing...");

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
            std::string input_pcd  = path_input + "test2.pcd";
            std::string output_pcd = path_output + "filtered_points.pcd";

            pick_and_place.process_cylinder(input_pcd, output_pcd,tf_buffer);
            pick_and_place.process_hole(input_pcd, output_pcd,tf_buffer);
            // ros::spinOnce();
            // rate.sleep();
        
    }

    // Depth camera callback
    
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


void process_cylinder(const std::string& input_pcd, const std::string& output_pcd, tf2_ros::Buffer& tf_buffer)
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
    sor.setMeanK(50); // Number of nearest neighbors to use for mean distance estimation
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
    cylinder_segmentation.setRadiusLimits(0.016, 0.022); // Set the radius limits for cylinder  //play


    
    // get the inliers and coefficients of the cylinder
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    

    // extract the cylinder from the point cloud
    pcl::ExtractIndices<pcl::PointXYZ> cylinder_extracted;
    pcl::ExtractIndices<pcl::PointNormal>   cylinder_indices_extractor_temp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_cylinders(new pcl::PointCloud<pcl::PointXYZ>);

    int count=0;
    int l=0;
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
        geometry_msgs::PointStamped point_world;
        try {
 
            point_world = tf_buffer.transform(point_cam, "world", ros::Duration(1.0));

            std::cout << "Centroid in world frame: ("
                    << point_world.point.x << ", "
                    << point_world.point.y << ", "
                    << point_world.point.z << ")" << std::endl;
                    

        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF transform failed: %s", ex.what());
        }

        centroid_cylinder[0]=point_world.point.x;
        centroid_cylinder[1]=point_world.point.y;
        centroid_cylinder[2]=point_world.point.z;

        if(cylinder_cloud->points.size() > 90){
            *all_cylinders += *cylinder_cloud;
            l++;
        }
        

        cylinder_extracted.setNegative(true);
        cylinder_extracted.filter(*cloud_filtered);

        // processing normals
        cylinder_indices_extractor_temp.setInputCloud(cloud_normals);
        cylinder_indices_extractor_temp.setIndices(inliers_cylinder);
        cylinder_indices_extractor_temp.setNegative(true);
        cylinder_indices_extractor_temp.filter(*cloud_normals);
        if (count>=1)
        {
            break;
        }
        count++;
    }
    else{
        std::cout << "No more cylinders. Terminating segmentation." << std::endl;
        break;
    }

    }


    pcl::PCDWriter cloud_writer;
    cloud_writer.write<pcl::PointXYZ>(output_pcd, *all_cylinders, false);
    // cloud_writer.write<pcl::PointXYZ>(output_pcd, *cloud_filtered, false);


}

// detect hole

void process_hole(const std::string& input_pcd, const std::string& output_pcd, tf2_ros::Buffer& tf_buffer)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader cloud_reader;
    cloud_reader.read(input_pcd, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 1. Voxel downsampling
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.003f, 0.003f, 0.003f);
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


    // 1️ Transform to world frame
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
    ec.setClusterTolerance(0.01);  // 2 cm tolerance between points in the same object
    ec.setMinClusterSize(50);      // min points in a cluster
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
        if (std::abs(cloud_world->points[idx].z - max_z) <= 0.02f) {
            top_surfaces->points.push_back(cloud_world->points[idx]);
        }
    }
}

    top_surfaces->width = top_surfaces->points.size();
    top_surfaces->height = 1;
    top_surfaces->is_dense = true;


      // segment a cylinder using RANSAC
    // get the normals of the point cloud (how the points are oriented in space)
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimator;
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(top_surfaces);
    normal_estimator.setKSearch(30); // Evaluate number of neighbors to use for normal estimation //play
    normal_estimator.compute(*cloud_normals);

    // segmentation of the cylinder from the normals
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::PointNormal> circle_segmentation;
    circle_segmentation.setOptimizeCoefficients(true);
    circle_segmentation.setModelType(pcl::SACMODEL_CIRCLE2D);
    circle_segmentation.setMethodType(pcl::SAC_RANSAC);
    circle_segmentation.setNormalDistanceWeight(0.05); // Set the distance threshold for cylinder
    circle_segmentation.setMaxIterations(10000); // Set the maximum number of iterations for RANSAC
    circle_segmentation.setDistanceThreshold(0.005); // Set the distance threshold for cylinder //play
    circle_segmentation.setRadiusLimits(0.018, 0.03); // Set the radius limits for cylinder  //play


    
    // get the inliers and coefficients of the cylinder
    pcl::PointIndices::Ptr inliers_circle(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_circle(new pcl::ModelCoefficients);
    

    // extract the cylinder from the point cloud
    pcl::ExtractIndices<pcl::PointXYZ> circle_extracted;
    pcl::ExtractIndices<pcl::PointNormal>   circle_indices_extractor_temp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_circle(new pcl::PointCloud<pcl::PointXYZ>);

    int count=0;
    int l=0;
    while(true)
    {
    pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud(new pcl::PointCloud<pcl::PointXYZ>);


    circle_segmentation.setInputCloud(top_surfaces);
    circle_segmentation.setInputNormals(cloud_normals);
    circle_segmentation.segment(*inliers_circle, *coefficients_circle);

    circle_extracted.setInputCloud(top_surfaces);
    circle_extracted.setIndices(inliers_circle);
    circle_extracted.setNegative(false); // Extract the cylinder
    circle_extracted.filter(*circle_cloud);

    

    if(!circle_cloud->points.empty()){
        // compute the centroid of the cylinder
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*circle_cloud, centroid);



        geometry_msgs::PointStamped point_cam;
        point_cam.header.stamp = ros::Time(0);  // Use latest TF
        point_cam.header.frame_id = "rgbd_camera_depth_optical_frame";  // <-- Replace with your actual camera frame
        point_cam.point.x = centroid[0];
        point_cam.point.y = centroid[1];
        point_cam.point.z = centroid[2];



        if (centroid[2]> 1.2)
        {

            circle_extracted.setNegative(true);
            circle_extracted.filter(*top_surfaces);

            // processing normals
            circle_indices_extractor_temp.setInputCloud(cloud_normals);
            circle_indices_extractor_temp.setIndices(inliers_circle);
            circle_indices_extractor_temp.setNegative(true);
            circle_indices_extractor_temp.filter(*cloud_normals);
            continue; // Skip if the centroid is too high
        }

        
        if (centroid[0] < 0.47)
        {
            circle_extracted.setNegative(true);
            circle_extracted.filter(*top_surfaces);

            // processing normals
            circle_indices_extractor_temp.setInputCloud(cloud_normals);
            circle_indices_extractor_temp.setIndices(inliers_circle);
            circle_indices_extractor_temp.setNegative(true);
            circle_indices_extractor_temp.filter(*cloud_normals);
            continue; // Skip if the centroid is too low
        }
        // print the centroid of the cylinder
        std::cout << "Centroid of the circle in the camera frame: ("
                << centroid[0] << ", " 
                << centroid[1] << "," 
                << centroid[2] << ")" << std::endl;

        // Transform the point to the world frame
        // geometry_msgs::PointStamped point_world;
        // try {
            
        //     point_world = tf_buffer.transform(point_cam, "world", ros::Duration(1.0));

        //     std::cout << "Centroid in world frame: ("
        //             << point_world.point.x << ", "
        //             << point_world.point.y << ", "
        //             << point_world.point.z << ")" << std::endl;

        // } catch (tf2::TransformException& ex) {
        //     ROS_WARN("TF transform failed: %s", ex.what());
        // }

        centroid_hole[0]=centroid[0];
        centroid_hole[1]=centroid[1];
        centroid_hole[2]=centroid[2];


        if(circle_cloud->points.size() > 90){
            *all_circle += *circle_cloud;
            l++;
        }


        circle_extracted.setNegative(true);
        circle_extracted.filter(*top_surfaces);

        // processing normals
        circle_indices_extractor_temp.setInputCloud(cloud_normals);
        circle_indices_extractor_temp.setIndices(inliers_circle);
        circle_indices_extractor_temp.setNegative(true);
        circle_indices_extractor_temp.filter(*cloud_normals);

    }
    else{
        std::cout << "No more circles. Terminating segmentation." << std::endl;
        break;
        got_centroid= true; // Set flag to true if we found a circle
    }

    }

   

    pcl::PCDWriter cloud_writer;
    cloud_writer.write<pcl::PointXYZ>(output_pcd, *all_circle, false);
    // ROS_INFO(" Saved %d detected pegs to %s", count, output_pcd.c_str());
}




    // Gripper control functions
    void openGripper(trajectory_msgs::JointTrajectory& posture)
    {
        posture.joint_names.resize(1);
        posture.joint_names[0] = "finger_joint";
        posture.points.resize(1);
        posture.points[0].positions.resize(1);
        posture.points[0].positions[0] = 0; // Open gripper
        posture.points[0].time_from_start = ros::Duration(1.0); // Increased duration
        ROS_INFO("Opening gripper: position = %f", posture.points[0].positions[0]);
    }

    void closedGripper(trajectory_msgs::JointTrajectory& posture)
    {
        posture.joint_names.resize(1);
        posture.joint_names[0] = "finger_joint";
        posture.points.resize(1);
        posture.points[0].positions.resize(1);
        posture.points[0].positions[0] = 0.3; // Close gripper 0.4
        posture.points[0].time_from_start = ros::Duration(1.0);
        ROS_INFO("Closing gripper: position = %f", posture.points[0].positions[0]);
    }
    
    // UR5 pick and place functions
    void ur5_pick(moveit::planning_interface::MoveGroupInterface& move_group)
    {
        std::vector<moveit_msgs::Grasp> grasps;
        grasps.resize(1);

        // Grasp pose
        grasps[0].grasp_pose.header.frame_id = "world";
        tf2::Quaternion orientation;
        orientation.setRPY(M_PI, 0, M_PI_2);
        grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
        grasps[0].grasp_pose.pose.position.x = centroid_cylinder[0]+0.003; // -0.02
        grasps[0].grasp_pose.pose.position.y = centroid_cylinder[1]-0.032; // +0.01
        // grasps[0].grasp_pose.pose.position.x = 0.3; // -0.02
        // grasps[0].grasp_pose.pose.position.y = 0.6; // +0.01
        grasps[0].grasp_pose.pose.position.z = 1.32; // Adjusted height
        ROS_INFO("Grasp pose: x=%f, y=%f, z=%f", grasps[0].grasp_pose.pose.position.x,
                 grasps[0].grasp_pose.pose.position.y, grasps[0].grasp_pose.pose.position.z);

        // Pre-grasp approach
        grasps[0].pre_grasp_approach.direction.header.frame_id = "world";
        grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
        grasps[0].pre_grasp_approach.min_distance = 0.12;
        grasps[0].pre_grasp_approach.desired_distance = 0.15;

        // // Post-grasp retreat
        // grasps[0].post_grasp_retreat.direction.header.frame_id = "world";
        // grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
        // grasps[0].post_grasp_retreat.min_distance = 0.1;
        // grasps[0].post_grasp_retreat.desired_distance = 0.25;

        openGripper(grasps[0].pre_grasp_posture);
        closedGripper(grasps[0].grasp_posture);

        move_group.setSupportSurfaceName("table1");

      
        if (!move_group.pick("cs1", grasps))
        {

        
            ROS_ERROR("Pick operation failed!");
            last_operation_failed = true;
        }
        else
        {    attachObject();
            last_operation_failed = false;
        }
    }

    void attachCollisionObject()
    {
        moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = "tool0";
        attached_object.object = collision_objects[2]; // cs1
        attached_object.object.operation = moveit_msgs::CollisionObject::ADD;
        attached_object.touch_links = {
            "robotiq_85_right_finger_tip_link",
            "robotiq_85_left_finger_tip_link",
            "robotiq_85_right_inner_knuckle_link",
            "robotiq_85_left_inner_knuckle_link",
            "wrist_3_link",
            "tool0",
            "robotiq_85_right_inner_knuckle_link",
            "robotiq_85_left_inner_knuckle_link",
            "robotiq_85_right_finger_link",
            "robotiq_85_left_finger_link",
            "robotiq_85_right_finger_tip_link",
            "robotiq_85_left_finger_tip_link"
           
        };
        planning_scene_interface.applyAttachedCollisionObject(attached_object);
        ROS_INFO("Applied attached collision object: cs1 to tool0");
    }

    void detachCollisionObject()
    {
        moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = "tool0";
        attached_object.object.operation = moveit_msgs::CollisionObject::REMOVE;
        attached_object.object.id = "cs1";
        planning_scene_interface.applyAttachedCollisionObject(attached_object);
        ROS_INFO("Detached collision object: cs1 from tool0");
    }

    void ur5_place(moveit::planning_interface::MoveGroupInterface& group)
    {
        std::vector<moveit_msgs::PlaceLocation> place_location;
        place_location.resize(1);

        place_location[0].place_pose.header.frame_id = "world";
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, 0);
        place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
        place_location[0].place_pose.pose.position.x = centroid_hole[0]; //-0.02
        place_location[0].place_pose.pose.position.y = centroid_hole[1]-0.045; // +0.01
        place_location[0].place_pose.pose.position.z = 1.18; // Adjusted height
        ROS_INFO("Place pose: x=%f, y=%f, z=%f", place_location[0].place_pose.pose.position.x,
                 place_location[0].place_pose.pose.position.y, place_location[0].place_pose.pose.position.z);

        place_location[0].pre_place_approach.direction.header.frame_id = "world";
        place_location[0].pre_place_approach.direction.vector.z = -1.0;
        place_location[0].pre_place_approach.min_distance = 0.08;
        place_location[0].pre_place_approach.desired_distance = 0.18;

        // place_location[0].post_place_retreat.direction.header.frame_id = "world";
        // place_location[0].post_place_retreat.direction.vector.z = 1.0;
        // place_location[0].post_place_retreat.min_distance = 0.1;
        // place_location[0].post_place_retreat.desired_distance = 0.25;
        // place_location[0].allowed_touch_objects.push_back("cs1");
        // place_location[0].allowed_touch_objects.push_back("wrist_3_link");
        openGripper(place_location[0].post_place_posture);

        group.setSupportSurfaceName("table2");
        group.allowReplanning(true);
        group.setGoalTolerance(0.03);
        if (!group.place("cs1", place_location))
            {    
                ROS_ERROR("Place operation failed!");
                last_operation_failed = true;
            }
        else
            {
                detachObject();
                last_operation_failed = false;
        
            }
    }

    void attachObject()
    {
        gazebo_ros_link_attacher::Attach srv;
        srv.request.model_name_1 = "robot";
        srv.request.link_name_1 = "wrist_3_link";
        srv.request.model_name_2 = "blue_cylinder1";
        srv.request.link_name_2 = "base_link";

        if (attach_client.call(srv) && srv.response.ok)
            ROS_INFO("Object attached successfully: %s::%s to %s::%s",
                     srv.request.model_name_1.c_str(), srv.request.link_name_1.c_str(),
                     srv.request.model_name_2.c_str(), srv.request.link_name_2.c_str());
        else
            ROS_ERROR("Failed to attach object: %s::%s to %s::%s",
                      srv.request.model_name_1.c_str(), srv.request.link_name_1.c_str(),
                      srv.request.model_name_2.c_str(), srv.request.link_name_2.c_str());
    }

    void detachObject()
    {
        gazebo_ros_link_attacher::Attach srv;
        srv.request.model_name_1 = "robot";
        srv.request.link_name_1 = "wrist_3_link";
        srv.request.model_name_2 = "blue_cylinder1";
        srv.request.link_name_2 = "base_link";

        if (detach_client.call(srv) && srv.response.ok)
            ROS_INFO("Object detached successfully: %s::%s from %s::%s",
                     srv.request.model_name_1.c_str(), srv.request.link_name_1.c_str(),
                     srv.request.model_name_2.c_str(), srv.request.link_name_2.c_str());
        else
            ROS_ERROR("Failed to detach object: %s::%s from %s::%s",
                      srv.request.model_name_1.c_str(), srv.request.link_name_1.c_str(),
                      srv.request.model_name_2.c_str(), srv.request.link_name_2.c_str());
    }

    void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
    {
        collision_objects.resize(7); // Adjusted to match number of objects

        // Table 1
        collision_objects[0].id = "table1";
        collision_objects[0].header.frame_id = "world";
        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
        collision_objects[0].primitives[0].dimensions = {1.5, 0.8, 1.015};
        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = 0.6;
        collision_objects[0].primitive_poses[0].position.y = -0.4;
        collision_objects[0].primitive_poses[0].position.z = 0.505;
        collision_objects[0].operation = moveit_msgs::CollisionObject::ADD;

        // Table 2
        collision_objects[1].id = "table2";
        collision_objects[1].header.frame_id = "world";
        collision_objects[1].primitives.resize(1);
        collision_objects[1].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
        collision_objects[1].primitives[0].dimensions = {1.5, 0.8, 1.015};
        collision_objects[1].primitive_poses.resize(1);
        collision_objects[1].primitive_poses[0].position.x = 0.6;
        collision_objects[1].primitive_poses[0].position.y = 0.402;
        collision_objects[1].primitive_poses[0].position.z = 0.505;
        collision_objects[1].operation = moveit_msgs::CollisionObject::ADD;

        // Small Cylinder (cs1)
        collision_objects[2].id = "cs1";
        collision_objects[2].header.frame_id = "world";
        collision_objects[2].primitives.resize(1);
        collision_objects[2].primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
        collision_objects[2].primitives[0].dimensions.resize(2);
        collision_objects[2].primitives[0].dimensions[0] = 0.2;   // height
        collision_objects[2].primitives[0].dimensions[1] = 0.02;  // radius
        collision_objects[2].primitive_poses.resize(1);
        collision_objects[2].primitive_poses[0].position.x = 0.3;
        collision_objects[2].primitive_poses[0].position.y = 0.6;
        collision_objects[2].primitive_poses[0].position.z = 1.12;
        collision_objects[2].primitive_poses[0].orientation.w = 1.0;
        collision_objects[2].operation = moveit_msgs::CollisionObject::ADD;

        // Bin Base
        collision_objects[3].id = "bin_base";
        collision_objects[3].header.frame_id = "world";
        collision_objects[3].primitives.resize(1);
        collision_objects[3].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
        collision_objects[3].primitives[0].dimensions = {0.5, 1.5, 0.74};
        collision_objects[3].primitive_poses.resize(1);
        collision_objects[3].primitive_poses[0].position.x = -0.6;
        collision_objects[3].primitive_poses[0].position.y = 0.0;
        collision_objects[3].primitive_poses[0].position.z = 0.35;
        collision_objects[3].operation = moveit_msgs::CollisionObject::ADD;

        // Bin 1
        collision_objects[4].id = "bin1";
        collision_objects[4].header.frame_id = "world";
        collision_objects[4].primitives.resize(1);
        collision_objects[4].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
        collision_objects[4].primitives[0].dimensions = {0.4, 0.4, 0.02};
        collision_objects[4].primitive_poses.resize(1);
        collision_objects[4].primitive_poses[0].position.x = -0.6;
        collision_objects[4].primitive_poses[0].position.y = 0.5;
        collision_objects[4].primitive_poses[0].position.z = 0.76;
        collision_objects[4].operation = moveit_msgs::CollisionObject::ADD;

        // Bin 2
        collision_objects[5].id = "bin2";
        collision_objects[5].header.frame_id = "world";
        collision_objects[5].primitives.resize(1);
        collision_objects[5].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
        collision_objects[5].primitives[0].dimensions = {0.4, 0.4, 0.02};
        collision_objects[5].primitive_poses.resize(1);
        collision_objects[5].primitive_poses[0].position.x = -0.6;
        collision_objects[5].primitive_poses[0].position.y = 0.0;
        collision_objects[5].primitive_poses[0].position.z = 0.76;
        collision_objects[5].operation = moveit_msgs::CollisionObject::ADD;

        // Bin 3
        collision_objects[6].id = "bin3";
        collision_objects[6].header.frame_id = "world";
        collision_objects[6].primitives.resize(1);
        collision_objects[6].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
        collision_objects[6].primitives[0].dimensions = {0.4, 0.4, 0.02};
        collision_objects[6].primitive_poses.resize(1);
        collision_objects[6].primitive_poses[0].position.x = -0.6;
        collision_objects[6].primitive_poses[0].position.y = -0.5;
        collision_objects[6].primitive_poses[0].position.z = 0.76;
        collision_objects[6].operation = moveit_msgs::CollisionObject::ADD;

        planning_scene_interface.applyCollisionObjects(collision_objects);
        ROS_INFO("Added %zu collision objects to planning scene", collision_objects.size());
    }

    void execution(PickAndPlace& pick_and_place,
             moveit::planning_interface::MoveGroupInterface& group,
             moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
    {
        // This function can be used to execute the pick and place operations
        // or any other logic that needs to be executed periodically.
        ros::Rate rate(10); // 10 Hz
        int step = 1;
        while (ros::ok())
        {
            switch (step)
            {
                case 1: // Add collision objects
                    if (stop)
                    {
                        ROS_INFO("Step 1: Waiting for clear path");
                        ros::Duration(2.0).sleep();
                        break; // stay in step 1
                    }
                    ROS_INFO("Adding collision objects");
                    pick_and_place.addCollisionObject(planning_scene_interface);
                    ros::Duration(2.0).sleep();
                    step++;
                    break;

                case 2: // Pick
                    if (stop)
                    {
                        ROS_INFO("Step 2: Waiting for clear path");
                        ros::Duration(2.0).sleep();
                        break; // retry step 2
                    }
                    if (slow_down)
                        {
                            group.setMaxVelocityScalingFactor(0.4);
                            group.setMaxAccelerationScalingFactor(0.4);
                            // ROS_INFO("Applying slowdown: 40% speed");
                        }
                    else
                        {
                            group.setMaxVelocityScalingFactor(1.0);
                            group.setMaxAccelerationScalingFactor(1.0);
                            // ROS_INFO("Applying default speed: 80% speed");
                        }
                    pick_and_place.ur5_pick(group);
                    if (last_operation_failed)
                    {
                        ROS_WARN("Pick failed, retrying step 2");
                        ros::Duration(2.0).sleep();
                        break; // retry step 2
                    }
                    step++;
                    break;

                case 3: // Attach object
                    if (stop)
                    {
                        ROS_INFO("Step 3: Waiting for clear path");
                        ros::Duration(2.0).sleep();
                        break; // retry step 3
                    }
                    pick_and_place.attachCollisionObject();
                    ros::Duration(2.0).sleep();
                    step++;
                    break;

                case 4: // Place
                    if (stop)
                    {
                        ROS_INFO("Step 4: Waiting for clear path");
                        ros::Duration(2.0).sleep();
                        break; // retry step 4
                    }
                    if (slow_down)
                        {
                            group.setMaxVelocityScalingFactor(0.4);
                            group.setMaxAccelerationScalingFactor(0.4);
                            // ROS_INFO("Applying slowdown: 40% speed");
                        }
                    else
                        {
                            group.setMaxVelocityScalingFactor(1.0);
                            group.setMaxAccelerationScalingFactor(1.0);
                            // ROS_INFO("Applying default speed: 80% speed");
                        }
                    pick_and_place.ur5_place(group);
                    if (last_operation_failed)
                    {
                        ROS_WARN("Place failed, retrying step 4");
                        ros::Duration(2.0).sleep();
                        break; // retry step 4
                    }
                    step++;
                    break;

                case 5: // Detach
                    if (stop)
                    {
                        ROS_INFO("Step 5: Waiting for clear path");
                        ros::Duration(2.0).sleep();
                        break; // retry step 5
                    }
                    pick_and_place.detachCollisionObject();
                    ros::Duration(2.0).sleep();
                    step++; // finished
                    break;

                default:
                    ROS_INFO("Task completed");
                    ros::shutdown();
                    break;
            }

            rate.sleep();
            ros::spinOnce();
        }
    }


private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface gripper_group;
    ros::ServiceClient attach_client;
    ros::ServiceClient detach_client;
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;  // start the TF listener


    // laser scan variables
    bool stop = false;
    bool slow_down = false;
    
    // save pcl to pcd
    bool has_saved = false;
    bool got_centroid = false;

    bool last_operation_failed = false;

    // Subscriber for laser scan
    ros::Subscriber sub;

    // Subscriber for point cloud
    ros::Subscriber sub1;

    // std::string path_input;

    // std::string path_output;
    // std::string input_pcd;
    // std::string output_pcd;
    std::array<double, 3> centroid_cylinder;
    std::array<double, 3> centroid_hole;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_pick_and_place");
    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("/scan", 10, scanCallback);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(2.0).sleep(); // Increased wait for initialization
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("ur5_manipulator");
    group.setPlanningTime(40.0);
    group.setNumPlanningAttempts(10);
    group.setMaxVelocityScalingFactor(1.0);
    group.setMaxAccelerationScalingFactor(1.0);
    group.setGoalTolerance(0.03);
    

    PickAndPlace pick_and_place(nh);

    pick_and_place.save(pick_and_place);

    ros::WallDuration(5.0).sleep();

    pick_and_place.execution(pick_and_place, group, planning_scene_interface);


    ros::waitForShutdown();
    return 0;
}