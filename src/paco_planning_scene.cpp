#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <paco_actions/baxter_utils.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "paco_planning_scene");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandlePtr node_handle_ptr(new ros::NodeHandle("~"));
    ros::Duration sleep_time(5.0);
    sleep_time.sleep();

    ros::Publisher planning_scene_publisher = node_handle_ptr->advertise<moveit_msgs::PlanningScene>("/paco_planning_scene", 1);

    ROS_INFO("[PACO planning scene]: initializing Moveit planning scene.");

    BaxterUtils baxter_manager(node_handle_ptr);
    baxter_manager.return_arms();

    ROS_INFO("[PACO planning scene]: arms ready.");

    // TODO subscribe in increase_angle_node and decrease_angle_node
    while (planning_scene_publisher.getNumSubscribers() < 1) {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    moveit_msgs::CollisionObject object;
    object.header.frame_id = "base";
    object.id = "table";
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.60;
    pose.position.z = -0.1;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.75;
    primitive.dimensions[1] = 1.70;
    primitive.dimensions[2] = 0.01;

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    object.operation = object.ADD;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.world.collision_objects.push_back(object);
    planning_scene_publisher.publish(planning_scene);

    ROS_INFO("[PACO planning scene]: scene initialization complete.");

    sleep_time.sleep();
    ros::waitForShutdown();
    return 0;
}
