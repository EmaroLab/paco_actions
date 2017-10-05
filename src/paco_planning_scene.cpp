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
#include <tf/transform_listener.h>

ros::Publisher* planning_scene_publisher_ptr = NULL;
moveit_msgs::PlanningScene paco_planning_scene;
tf::TransformListener* tf_listener_ptr = NULL;
tf::StampedTransform tmp_transform;
int links_n;

void add_setup_objs(){
    paco_planning_scene.world.collision_objects.clear();
    moveit_msgs::CollisionObject object;
    shape_msgs::SolidPrimitive primitive;

    // Adding table

    object.header.frame_id = "base";
    object.id = "table";
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.60;
    pose.position.z = -0.12; // -0.1

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.75;
    primitive.dimensions[1] = 1.70;
    primitive.dimensions[2] = 0.01;

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    object.operation = object.ADD;

    paco_planning_scene.world.collision_objects.push_back(object);

    // Adding Kinect bounding box

    object.header.frame_id = "head_camera";
    object.id = "kinect";
    pose.orientation.w = 0.7;
    pose.orientation.x = -0.07;
    pose.orientation.y = -0.05;
    pose.orientation.z = -0.7;
    pose.position.x = 0;
    pose.position.z = 0.12;

    primitive.dimensions[0] = 0.26; // 0.06
    primitive.dimensions[1] = 0.28;
    primitive.dimensions[2] = 0.10;

    object.primitives.clear();
    object.primitives.push_back(primitive);
    object.primitive_poses.clear();
    object.primitive_poses.push_back(pose);

    paco_planning_scene.world.collision_objects.push_back(object);
    planning_scene_publisher_ptr->publish(paco_planning_scene);
}

void update_articulated_obj(){
    paco_planning_scene.world.collision_objects.clear();
    shape_msgs::SolidPrimitive primitive;
    std::stringstream link;

    moveit_msgs::CollisionObject object;
    object.header.frame_id = "base";

    for (int i = 0; i< links_n; i++){
        link << "/link" << std::to_string(i);

        try {
            tf_listener_ptr->waitForTransform("/base", link.str(), ros::Time::now(), ros::Duration(1.0));
            tf_listener_ptr->lookupTransform("/base", link.str(), ros::Time(0), tmp_transform);
        }catch (tf::TransformException ex){
            ROS_WARN("[paco_planning_scene]: cannot see %s. Skipping...", link.str());
            continue;
        }

        object.header.frame_id = "base";
        object.id = link.str().substr(1);
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(tmp_transform, pose);

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.02; //0.04
        primitive.dimensions[1] = 0.12;
        primitive.dimensions[2] = 0.02;

        object.primitives.clear();
        object.primitives.push_back(primitive);
        object.primitive_poses.clear();
        object.primitive_poses.push_back(pose);
        object.operation = object.ADD;

        paco_planning_scene.world.collision_objects.push_back(object);
        link.str("");
    }
    planning_scene_publisher_ptr->publish(paco_planning_scene);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "paco_planning_scene");
    ros::AsyncSpinner spinner(1);

    ros::NodeHandlePtr node_handle_ptr(new ros::NodeHandle("~"));

    ros::Publisher planning_scene_publisher =
            node_handle_ptr->advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    planning_scene_publisher_ptr = &planning_scene_publisher;

    // TODO subscribe in increase_angle_node and decrease_angle_node
    while (planning_scene_publisher_ptr->getNumSubscribers() < 1) ros::WallDuration(0.5).sleep();

    BaxterUtils baxter_manager(node_handle_ptr);
    baxter_manager.return_arms();
    baxter_manager.open_gripper(baxter_manager.BOTH);

    ROS_INFO("[PACO planning scene]: initializing Moveit planning scene.");
    tf::TransformListener tf_listener;
    tf_listener_ptr = &tf_listener;

    int refresh_period;
    node_handle_ptr->param("/paco/links_number", links_n, 5);
    node_handle_ptr->param("/paco/frame_acquisition_timeout", refresh_period, 1);

    ROS_INFO("[PACO planning scene]: arms ready.");

    paco_planning_scene.is_diff = true;

    ROS_INFO("[PACO planning scene]: scene initialization complete. Updating object links.");

    ros::Duration wait(refresh_period);
    spinner.start();
    while (ros::ok()){
        add_setup_objs();
//        update_articulated_obj();
        wait.sleep();
    }
    spinner.stop();
    return 0;
}
