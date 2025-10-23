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

using namespace tf2;

const double tau = 2 * M_PI;

class PickAndPlace
{
public:
    PickAndPlace(ros::NodeHandle& nh)
        : planning_scene_interface(),
          move_group("ur5_manipulator"),
          gripper_group("robotiq_gripper")
    {
        move_group.setPlanningTime(30.0);
        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);
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
    }

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
        posture.points[0].positions[0] = 0.4; // Close gripper
        posture.points[0].time_from_start = ros::Duration(1.0);
        ROS_INFO("Closing gripper: position = %f", posture.points[0].positions[0]);
    }

    void ur5_pick(moveit::planning_interface::MoveGroupInterface& move_group)
    {
        std::vector<moveit_msgs::Grasp> grasps;
        grasps.resize(1);

        // Grasp pose
        grasps[0].grasp_pose.header.frame_id = "world";
        tf2::Quaternion orientation;
        orientation.setRPY(M_PI, 0, M_PI_2);
        grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
        grasps[0].grasp_pose.pose.position.x = 0.3;
        grasps[0].grasp_pose.pose.position.y = -0.6;
        grasps[0].grasp_pose.pose.position.z = 1.32;
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
            ROS_ERROR("Pick operation failed!");
        else
            attachObject();
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
            "wrist_3_link"
            "tool0"
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
        place_location[0].place_pose.pose.position.x = 0.48; //-0.02
        place_location[0].place_pose.pose.position.y = -0.49; // +0.01
        place_location[0].place_pose.pose.position.z = 1.2;
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
            ROS_ERROR("Place operation failed!");
        else
            detachObject();
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
        collision_objects[2].primitive_poses[0].position.y = -0.6;
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

private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface gripper_group;
    ros::ServiceClient attach_client;
    ros::ServiceClient detach_client;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_pick_and_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(2.0).sleep(); // Increased wait for initialization
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("ur5_manipulator");
    group.setPlanningTime(30.0);
    group.setNumPlanningAttempts(10);
    group.setMaxVelocityScalingFactor(0.5);
    group.setMaxAccelerationScalingFactor(0.5);
    group.setGoalTolerance(0.03);

    PickAndPlace pick_and_place(nh);

    ROS_INFO("Adding collision objects");
    pick_and_place.addCollisionObject(planning_scene_interface);
    ros::WallDuration(2.0).sleep();

    // ROS_INFO("Moving to home position");
    // group.setNamedTarget("home");
    // if (!group.move())
    //     ROS_ERROR("Failed to move to home position");
    // ros::WallDuration(2.0).sleep();

    ROS_INFO("Executing pick operation");
    pick_and_place.ur5_pick(group);
    

    ROS_INFO("Attaching collision object");
    pick_and_place.attachCollisionObject();
    ros::WallDuration(5.0).sleep();

    // ROS_INFO("Moving to home position");
    // group.setNamedTarget("home");
    // if (!group.move())
    //     ROS_ERROR("Failed to move to home position");
    // ros::WallDuration(5.0).sleep();

    ROS_INFO("Executing place operation");
    pick_and_place.ur5_place(group);
    

    ROS_INFO("Detaching collision object");
    pick_and_place.detachCollisionObject();
    ros::WallDuration(2.0).sleep();

    ros::waitForShutdown();
    return 0;
}