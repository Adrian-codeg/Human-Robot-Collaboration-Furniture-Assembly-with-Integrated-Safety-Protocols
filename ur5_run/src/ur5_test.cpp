#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_srvs/Empty.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(2.0);

    moveit::planning_interface::MoveGroupInterface group("ur5_manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    // group.setNamedTarget("home");
    // group.move();

    group.setStartStateToCurrentState();
    group.setPlanningTime(100.0);

    // Target position
    geometry_msgs::Pose target_pose1;
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0,  M_PI_2);  // Rotate 180 degrees around X-axis
    target_pose1.orientation = tf2::toMsg(q);


    target_pose1.position.x = 0.3;
    target_pose1.position.y = -0.3;
    target_pose1.position.z = 1.33;
    group.setPoseTarget(target_pose1);

    


    // Clear octomap to avoid false collisions
    // ros::ServiceClient clear_octomap_client = nh.serviceClient<std_srvs::Empty>("/clear_octomap");
    // std_srvs::Empty empty_srv;
    // clear_octomap_client.call(empty_srv);

    // visualize the planning
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    ROS_INFO("visualizeing plan %s", success.val ? "":"FAILED");
    
    // Clear octomap before execution
    // clear_octomap_client.call(empty_srv);
    // move the group arm
    group.move();

    moveit::planning_interface::MoveGroupInterface gripper_group("robotiq_gripper");
    gripper_group.setNamedTarget("close");
    gripper_group.move();

    ros::shutdown();
    return 0;

}