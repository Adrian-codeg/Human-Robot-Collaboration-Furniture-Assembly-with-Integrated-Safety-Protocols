#include <ros/ros.h>
#include <iostream>
#include <math.h>

#include <array>
#include<vector>
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

std::array<std::array<double, 4>,3> centroid_hole;
std::array<std::array<double, 4>,3> centroid_cylinder;
int row = 0;
int row_hole = 0;

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
    cylinder_segmentation.setDistanceThreshold(0.001); // Set the distance threshold for cylinder //play
    cylinder_segmentation.setRadiusLimits(0.016, 0.022); // Set the radius limits for cylinder  //play


    
    // get the inliers and coefficients of the cylinder
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    

    // extract the cylinder from the point cloud
    pcl::ExtractIndices<pcl::PointXYZ> cylinder_extracted;
    pcl::ExtractIndices<pcl::PointNormal>   cylinder_indices_extractor_temp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_cylinders(new pcl::PointCloud<pcl::PointXYZ>);


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
        
        else if(cylinder_cloud->points.size() > 100){
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
        std::cout << "Centroid of the cylinder in the camera frame: ("
                << centroid[0] << ", " 
                << centroid[1] << "," 
                << centroid[2] << ")" << std::endl;

        // Transform the point to the world frame
        try {
            geometry_msgs::PointStamped point_world;
            point_world = tf_buffer.transform(point_cam, "world", ros::Duration(1.0));

            std::cout << "Centroid in world frame: ("
                    << point_world.point.x << ", "
                    << point_world.point.y << ", "
                    << point_world.point.z << ")" << std::endl;

            centroid_cylinder[row][0]=point_world.point.x;
            centroid_cylinder[row][1]=point_world.point.y;
            centroid_cylinder[row][2]=point_world.point.z;


        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF transform failed: %s", ex.what());
        }



        cylinder_extracted.setNegative(true);
        cylinder_extracted.filter(*cloud_filtered);

        // processing normals
        cylinder_indices_extractor_temp.setInputCloud(cloud_normals);
        cylinder_indices_extractor_temp.setIndices(inliers_cylinder);
        cylinder_indices_extractor_temp.setNegative(true);
        cylinder_indices_extractor_temp.filter(*cloud_normals);
        row++;
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


    //  Transform to world frame
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
    ec.setClusterTolerance(0.05);  // 2 cm tolerance between points in the same object
    ec.setMinClusterSize(10);      // min points in a cluster
    ec.setMaxClusterSize(50000);   // max points in a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_world);
    ec.extract(cluster_indices);


// filter top surfaces
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
        if (std::abs(cloud_world->points[idx].z - max_z) <= 0.01f) {
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

    // high curvature clouds for XYZ and normals
    pcl::PointCloud<pcl::PointXYZ>::Ptr high_curvature_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr high_curvature_normals(new pcl::PointCloud<pcl::PointNormal>);

    for (size_t i = 0; i < cloud_normals->size(); ++i) {
        if (cloud_normals->points[i].curvature > 0.02f) { // adjust threshold
            high_curvature_xyz->push_back(top_surfaces->points[i]);
            high_curvature_normals->push_back(cloud_normals->points[i]);
        }
    }
    

    // segmentation of the cylinder from the normals
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::PointNormal> circle_segmentation;
    circle_segmentation.setOptimizeCoefficients(true);
    circle_segmentation.setModelType(pcl::SACMODEL_CIRCLE2D);
    circle_segmentation.setMethodType(pcl::SAC_RANSAC);
    circle_segmentation.setNormalDistanceWeight(0.05); // Set the distance threshold for cylinder
    circle_segmentation.setMaxIterations(10000); // Set the maximum number of iterations for RANSAC
    circle_segmentation.setDistanceThreshold(0.005); // Set the distance threshold for cylinder //play
    circle_segmentation.setRadiusLimits(0.015, 0.03); // Set the radius limits for cylinder  //play


    
    // get the inliers and coefficients of the cylinder
    pcl::PointIndices::Ptr inliers_circle(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_circle(new pcl::ModelCoefficients);
    

    // extract the cylinder from the point cloud
    pcl::ExtractIndices<pcl::PointXYZ> circle_extracted;
    pcl::ExtractIndices<pcl::PointNormal>   circle_indices_extractor_temp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_circle(new pcl::PointCloud<pcl::PointXYZ>);


int l = 0;

// loop to find curvative points
while (!high_curvature_xyz->empty()) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Segment circle from high-curvature points
    circle_segmentation.setInputCloud(high_curvature_xyz);
    circle_segmentation.setInputNormals(high_curvature_normals);
    circle_segmentation.segment(*inliers_circle, *coefficients_circle);

    if (inliers_circle->indices.empty()) {
        std::cout << "No more circles. Terminating segmentation." << std::endl;
        break;
    }

    // Extract the segmented circle
    circle_extracted.setInputCloud(high_curvature_xyz);
    circle_extracted.setIndices(inliers_circle);
    circle_extracted.setNegative(false);
    circle_extracted.filter(*circle_cloud);

    if (circle_cloud->points.empty()) {
        std::cout << "Segmented circle has no points. Skipping." << std::endl;
        break;
    }

    // Compute centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*circle_cloud, centroid);

   
    // Optional: transform to world frame
    geometry_msgs::PointStamped point_cam;
    point_cam.header.stamp = ros::Time(0);
    point_cam.header.frame_id = "rgbd_camera_depth_optical_frame";
    point_cam.point.x = centroid[0];
    point_cam.point.y = centroid[1];
    point_cam.point.z = centroid[2];
    

    // Uncomment if TF transform is needed
    // try {
    //     geometry_msgs::PointStamped point_world = tf_buffer.transform(point_cam, "world", ros::Duration(1.0));
    //     std::cout << "Centroid in world frame: ("
    //               << point_world.point.x << ", "
    //               << point_world.point.y << ", "
    //               << point_world.point.z << ")" << std::endl;
    // } catch (tf2::TransformException& ex) {
    //     ROS_WARN("TF transform failed: %s", ex.what());
    // }

    // Skip circles that are too high
    if (centroid[2] > 1.2) {
        ROS_WARN("Skipping this circle due to height.");
         // Remove the high circle from input clouds
        circle_extracted.setNegative(true);
        circle_extracted.filter(*high_curvature_xyz);

        circle_indices_extractor_temp.setInputCloud(high_curvature_normals);
        circle_indices_extractor_temp.setIndices(inliers_circle);
        circle_indices_extractor_temp.setNegative(true);
        circle_indices_extractor_temp.filter(*high_curvature_normals);

        continue;
        
    } else if (circle_cloud->points.size() > 90) {
        *all_circle += *circle_cloud;
        l++;
    }

    std::cout << "Centroid of the circle in camera frame: ("
              << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ")" << std::endl;

    centroid_hole[row_hole][0] = centroid[0];
    centroid_hole[row_hole][1] = centroid[1];
    centroid_hole[row_hole][2] = centroid[2];
    

    // Remove extracted circle from input clouds
    circle_extracted.setNegative(true);
    circle_extracted.filter(*high_curvature_xyz);

    circle_indices_extractor_temp.setInputCloud(high_curvature_normals);
    circle_indices_extractor_temp.setIndices(inliers_circle);
    circle_indices_extractor_temp.setNegative(true);
    circle_indices_extractor_temp.filter(*high_curvature_normals);
    row_hole++;
}

    // else{
    //     std::cout << "No more circles. Terminating segmentation." << std::endl;
    //     break;
    // }

    
   

    pcl::PCDWriter cloud_writer;
    cloud_writer.write<pcl::PointXYZ>(output_pcd, *all_circle, false);
    // ROS_INFO(" Saved %d detected pegs to %s", count, output_pcd.c_str());
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

    process_cylinder(input_pcd, output_pcd,tf_buffer);
    process_hole(input_pcd, output_pcd,tf_buffer);


    return 0;
}