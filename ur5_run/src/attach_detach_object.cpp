#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cobot_planner");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(1);
    spin.start();

    moveit::planning_interface::MoveGroupInterface group("ur5_manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::PlanningSceneInterface current_scene;
    sleep(2);

    moveit_msgs::CollisionObject cube;
    cube.id = "grasping_object";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.03;
    primitive.dimensions[1] = 0.03;
    primitive.dimensions[2] = 0.08;

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.185;
    pose.position.y = 0.1;
    pose.position.z = 1.874;

    cube.primitives.push_back(primitive);
    cube.primitive_poses.push_back(pose);
    cube.operation = cube.ADD;
        cube.header.frame_id = "base_link";
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(cube);

    current_scene.addCollisionObjects(collision_objects);
    sleep(4);

    // Attaching
    ROS_INFO("Attaching object to the robot");
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "gripper_connector";
    attached_object.object = cube;
    current_scene.applyAttachedCollisionObject(attached_object);
    sleep(10);

    // Detaching
    ROS_INFO("Detaching object to the robot");
    cube.operation = cube.REMOVE;
    attached_object.link_name = "gripper_connector";
    attached_object.object = cube;
    current_scene.applyAttachedCollisionObject(attached_object);
    sleep(4);
    ros::shutdown();

}