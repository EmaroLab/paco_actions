#include "paco_actions/increase_angle_node.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

    ros::NodeHandlePtr nh_ptr = NULL;

    /* constructor */
    PacoIncreaseAngle::PacoIncreaseAngle(ros::NodeHandle &nh, std::string &actionserver)
            : action_client(actionserver, true) {
        params.name = "increase_angle";
        // costmap client
        clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    }

    /* action dispatch callback */
    bool PacoIncreaseAngle::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        ROS_INFO("Executing action %s, params: %s %s %s", msg->name.c_str(), msg->parameters[0].value.c_str(),
                  msg->parameters[1].value.c_str(), msg->parameters[2].value.c_str());

        // parsing msg
        std::string root_link;
        std::string child_link = msg->parameters.begin()->value;
        int child_link_n = stoi(child_link.substr(4));
        child_link_n == 0 ? (root_link = "link0") : (root_link = "link%s", std::to_string(child_link_n - 1));

        // acquiring links positions
        tf::TransformListener listener;

        tf::StampedTransform root_link_transform;
        tf::StampedTransform child_link_transform;
        ros::Time now = ros::Time::now();

        try{
            listener.waitForTransform("/base", ("/%s", child_link), now, ros::Duration(10.0));
            listener.lookupTransform("/base", ("/%s", child_link),
                                     ros::Time(0), child_link_transform);
            if (root_link != child_link){
                listener.waitForTransform("/base", ("/%s", root_link), now, ros::Duration(10.0));
                listener.lookupTransform("/base", ("/%s", root_link),
                                         ros::Time(0), root_link_transform);
            }
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }
        ROS_INFO("[PACO increase action]: links acquired.");

        // select grippers

        std::string root_arm;
        std::string child_arm;

        if (child_link_transform.getOrigin().getY() >= root_link_transform.getOrigin().getY()){
            child_arm = "left_arm";
            root_arm  = "right_arm";
        }else{
            child_arm = "right_arm";
            root_arm  = "left_arm";
        }

        // moving
        ros::Publisher gripper_control = nh_ptr->advertise<baxter_gripper_manager::GripperCommand>("baxter_gripper_control", 100);

        baxter_gripper_manager::GripperCommand gripper_command;
        planning_interface::MotionPlanRequest req;
        planning_interface::MotionPlanResponse res;

        moveit::planning_interface::MoveGroupInterface move_group("both_arms");
        req.group_name = "arms";

        geometry_msgs::PoseStamped root_pose;
        geometry_msgs::PoseStamped child_pose;

        // move to links

        gripper_command.arm = "both";
        gripper_command.command = "open";
        gripper_control.publish(gripper_command);

        if (root_link != "link0"){
            root_pose.header.frame_id = "base";
            root_pose.pose.position.x = root_link_transform.getOrigin().getX();
            root_pose.pose.position.y = root_link_transform.getOrigin().getY();
            root_pose.pose.position.z = root_link_transform.getOrigin().getZ() + 0.15;
            root_pose.pose.orientation.y = 1.0;
        }

        child_pose.header.frame_id = "base";
        child_pose.pose.position.x = child_link_transform.getOrigin().getX();
        child_pose.pose.position.y = child_link_transform.getOrigin().getY();
        child_pose.pose.position.z = child_link_transform.getOrigin().getZ() + 0.15;
        child_pose.pose.orientation.y = 1.0;

        move_group.setPoseTarget(root_pose, root_arm);
        move_group.setPoseTarget(child_pose, child_arm);

        ROS_INFO("[PACO increase action]: getting closer to links.");
        if (!move_group.move()){
            ROS_ERROR("[PACO increase action]: unable to reach target positions.");
            return false;
        }

        // approach

        if (root_link != "link0") {
            root_pose.header.frame_id = "base";
            root_pose.pose.position.x = root_link_transform.getOrigin().getX();
            root_pose.pose.position.y = root_link_transform.getOrigin().getY();
            root_pose.pose.position.z = root_link_transform.getOrigin().getZ();
            root_pose.pose.orientation.y = 1.0;
        }

        child_pose.header.frame_id = "base";
        child_pose.pose.position.x = child_link_transform.getOrigin().getX();
        child_pose.pose.position.y = child_link_transform.getOrigin().getY();
        child_pose.pose.position.z = child_link_transform.getOrigin().getZ();
        child_pose.pose.orientation.y = 1.0;

        move_group.setPoseTarget(root_pose, root_arm);
        move_group.setPoseTarget(child_pose, child_arm);

        ROS_INFO("[PACO increase action]: approaching links.");
        if (!move_group.move()){
            ROS_ERROR("[PACO increase action]: unable to reach target positions.");
            return false;
        }

        gripper_command.arm = "both";
        gripper_command.command = "close";
        gripper_control.publish(gripper_command);

        ros::Duration d(2.5);
        d.sleep();
        return true;
    }
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "paco_increase_angle");
    KCL_rosplan::nh_ptr = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    std::string actionserver;
    KCL_rosplan::nh_ptr->param("action_server", actionserver, std::string("/increase_angle"));

    // create PDDL action subscriber
    KCL_rosplan::PacoIncreaseAngle increaseAngle(*KCL_rosplan::nh_ptr, actionserver);



    // listen for action dispatch
    ros::Subscriber ds = KCL_rosplan::nh_ptr->subscribe("/kcl_rosplan/action_dispatch", 1000,
                                      &KCL_rosplan::RPActionInterface::dispatchCallback,
                                      dynamic_cast<KCL_rosplan::RPActionInterface*>(&increaseAngle));
    increaseAngle.runActionInterface();

    return 0;
}