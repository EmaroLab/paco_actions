#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <sstream>

#include "rosplan_action_interface/RPActionInterface.h"

#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "std_srvs/Empty.h"
#include <angles/angles.h>
#include "baxter_utils.h"
#include <Eigen/Dense>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#ifndef KCL_movebase
#define KCL_movebase

/**
 * Executes actions planned by ROSPlan to manipulate an articulated object.
 */

namespace KCL_rosplan {

    class PacoIncreaseAngle: public RPActionInterface
    {

    private:
        enum ACTION_DESCRIPTORS{
            RIGHT, LEFT, H_GRASP, V_GRASP
        };

        struct ActionDescriptor{
            int arm, mode;
            std::string gripper_frame, gripper_name, link_name;
            geometry_msgs::PoseStamped gripper_pose;
            tf::StampedTransform link_transform;
        };

        const Eigen::Matrix4d joint_transl;
        ros::ServiceClient clear_costmaps_client;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client;
        tf::TransformListener listener;
        double rotation_angle;
        ActionDescriptor child_action_descriptor;
        ActionDescriptor root_action_descriptor;
        moveit::planning_interface::MoveGroupInterface move_group;

        void get_y_rot_matrix(Eigen::Affine3d& rot_matrix, double angle);
        void get_z_rot_matrix(Eigen::Affine3d& rot_matrix, double angle);
        void compute_hover_pose(ActionDescriptor &descriptor);
        void compute_child_hover_pose();
        void compute_root_hover_pose();
        bool exit();


        geometry_msgs::Pose get_circ_cclockwise_waypoints(std::vector<geometry_msgs::Pose> &waypoints,
                                                          const tf::StampedTransform& link_to_world_transform,
                                                          const tf::StampedTransform& gripper_to_world_transform);

    public:
        /* constructor */
        PacoIncreaseAngle(ros::NodeHandle &nh, std::string &actionserver);

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);



    };
}
#endif