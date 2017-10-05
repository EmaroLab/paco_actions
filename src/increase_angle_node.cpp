#include "paco_actions/increase_angle_node.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

    ros::NodeHandlePtr nh_ptr = NULL;
    BaxterUtils *baxter_manager_ptr = NULL;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    boost::shared_ptr<tf::TransformListener> tf_;
    int resolution;

    /* constructor */
    PacoIncreaseAngle::PacoIncreaseAngle(ros::NodeHandle &nh, std::string &actionserver)
            : action_client(actionserver, true),
              rotation_angle(2 * 3.14159 / resolution),
              move_group("both_arms"),
              joint_transl((Eigen::Matrix4d() << 1.0, 0.0, 0.0, 0.0,
                                                 0.0, 1.0, 0.0, 0.06,
                                                 0.0, 0.0, 1.0, 0.0,
                                                 0.0, 0.0, 0.0, 1.0).finished()){
        params.name = "increase_angle";
        clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        move_group.allowReplanning(true);
        move_group.setPlannerId("RRTConnectkConfigDefault");
    }

    /**
     * Generates a target pose for moveit to hover 5cm above a target frame (link).
     * The pose is perpendicular to the table if the link is vertical to the robot (ACTION_DESCRIPTORS::V_GRASP).
     * The pose is inclined 45° wrt the table if the link is horizontal (ACTION_DESCRIPTORS::H_GRASP).
     * Incline angle also depend on which arm is being used got the grasp.
     * @param obj_pos[in] pose of the target link
     * @param target_pos[out] desired hover pose
     * @param ActionDescriptor descriptor action descriptor of the child/root link to act on
     */
    void PacoIncreaseAngle::compute_hover_pose(ActionDescriptor &descriptor) {

        descriptor.gripper_pose.pose.position.x = descriptor.link_transform.getOrigin().getX();
        descriptor.gripper_pose.pose.position.y = descriptor.link_transform.getOrigin().getY();
        descriptor.gripper_pose.pose.position.z = descriptor.link_transform.getOrigin().getZ() + 0.05;

        if (descriptor.mode == ACTION_DESCRIPTORS::V_GRASP){
                    descriptor.gripper_pose.pose.orientation.x = 0;
                    descriptor.gripper_pose.pose.orientation.y = 1;
                    descriptor.gripper_pose.pose.orientation.z = 0;
                    descriptor.gripper_pose.pose.orientation.w = 0;
            }
        if (descriptor.mode == ACTION_DESCRIPTORS::H_GRASP){
            if (descriptor.arm == ACTION_DESCRIPTORS::RIGHT){
                descriptor.gripper_pose.pose.orientation.x = -0.6533;
                descriptor.gripper_pose.pose.orientation.y = 0.6533;
                descriptor.gripper_pose.pose.orientation.z = 0.2706;
                descriptor.gripper_pose.pose.orientation.w = 0.2706;
                descriptor.gripper_pose.pose.position.y   += 0.02;
            }
            if (descriptor.arm == ACTION_DESCRIPTORS::LEFT){
                descriptor.gripper_pose.pose.orientation.x = 0.6533;
                descriptor.gripper_pose.pose.orientation.y = -0.6533;
                descriptor.gripper_pose.pose.orientation.z = 0.2706;
                descriptor.gripper_pose.pose.orientation.w = 0.2706;
                descriptor.gripper_pose.pose.position.y   -= 0.02;
            }
        }

    }

    void PacoIncreaseAngle::compute_child_hover_pose() {
        compute_hover_pose(child_action_descriptor);
    }

    void PacoIncreaseAngle::compute_root_hover_pose() {
        compute_hover_pose(root_action_descriptor);
    }

    /**
     * Gets a rotation matrix around y for a given rotation angle.
     * @param rot_matrix[out]
     * @param angle[in]
     */
    void PacoIncreaseAngle::get_y_rot_matrix(Eigen::Affine3d& rot_matrix, double angle) {
        rot_matrix.matrix().Zero();
        rot_matrix(0,0) = cos(angle);
        rot_matrix(0,2) = sin(angle);
        rot_matrix(1,1) = 1;
        rot_matrix(2,0) = -sin(angle);
        rot_matrix(2,2) = cos(angle);
        rot_matrix(3,3) = 1;
    }

    /**
     * Gets a rotation matrix around z for a given rotation angle.
     * @param rot_matrix[out]
     * @param angle[in]
     */
    void PacoIncreaseAngle::get_z_rot_matrix(Eigen::Affine3d& rot_matrix, double angle) {
        rot_matrix.matrix().Zero();
        rot_matrix(0,0) = cos(angle);
        rot_matrix(0,1) = -sin(angle);
        rot_matrix(1,0) = sin(angle);
        rot_matrix(1,1) = cos(angle);
        rot_matrix(2,2) = 1;
        rot_matrix(3,3) = 1;
    }

    /**
     * Generates 5 intermediate waypoints on a circular arc center at a link joint. These can be used with
     * moveit `computeCartesianPath` to generate a circular trajectory.
     * @param link_name[in] name of the link to rotate
     * @param increase_angle[in] rotation angle
     * @param link_to_world_transform[in] transform of the link wrt to the world
     * @param waypoints[out] vector of desired waypoints
     * @return The pose of the last last waypoint
     */
    geometry_msgs::Pose PacoIncreaseAngle::get_circ_cclockwise_waypoints(std::vector<geometry_msgs::Pose> &waypoints,
                                                                         const tf::StampedTransform& link_to_world_transform,
                                                                         const tf::StampedTransform& gripper_to_world_transform){
        // wTg = wTl * lTj * T_rot_z(increase_angle) * (lTj)' * lTg
        geometry_msgs::Pose tmp_pose;
        double step_angle = rotation_angle / 5;
        double tilt_angle = 0.15708; // use to transition the arm between tilted/untilted configuration, 9 deg per step
        if (child_action_descriptor.mode == ACTION_DESCRIPTORS::V_GRASP) tilt_angle = -tilt_angle;

        // get wTl
        Eigen::Affine3d wTl;
        tf::transformTFToEigen(link_to_world_transform, wTl);

        // compute lTg = (wTl)' * wTg (implies computing initial wTg, marker not visible now)
        Eigen::Affine3d lTg;
        Eigen::Affine3d wTg;
        tf::transformTFToEigen(gripper_to_world_transform, wTg);
        lTg = wTl.inverse() * wTg;

        // lTj = joint_transl
        Eigen::Affine3d lTj;
        lTj.matrix() = joint_transl;

        tmp_pose.position.x = gripper_to_world_transform.getOrigin().getX();
        tmp_pose.position.y = gripper_to_world_transform.getOrigin().getY();
        tmp_pose.position.z = gripper_to_world_transform.getOrigin().getZ();
        tmp_pose.orientation.x = gripper_to_world_transform.getRotation().getX();
        tmp_pose.orientation.y = gripper_to_world_transform.getRotation().getY();
        tmp_pose.orientation.z = gripper_to_world_transform.getRotation().getZ();
        tmp_pose.orientation.w = gripper_to_world_transform.getRotation().getW();

        waypoints.push_back(tmp_pose);

        Eigen::Affine3d T_rot_z, T_rot_y;
        tf::Transform wTg_trans;

        //creating waypoints sampling arc in 5 steps, max arc tested: 90°
        for (int i = 1; i <= 5; i++){

            // compute T_rot_z and wTg
            get_z_rot_matrix(T_rot_z, step_angle * i);
            get_y_rot_matrix(T_rot_y, tilt_angle * i);
            wTg = wTl * lTj * T_rot_z * lTj.inverse() * lTg * T_rot_y;

            tf::transformEigenToTF(wTg, wTg_trans);

            tmp_pose.position.x = wTg_trans.getOrigin().getX();
            tmp_pose.position.y = wTg_trans.getOrigin().getY();
            tmp_pose.position.z = wTg_trans.getOrigin().getZ();
            tmp_pose.orientation.x = wTg_trans.getRotation().getX();
            tmp_pose.orientation.y = wTg_trans.getRotation().getY();
            tmp_pose.orientation.z = wTg_trans.getRotation().getZ();
            tmp_pose.orientation.w = wTg_trans.getRotation().getW();

            waypoints.push_back(tmp_pose);
        }
        return tmp_pose;
    }

    bool PacoIncreaseAngle::exit(){

        baxter_manager_ptr->open_gripper(BaxterUtils::BOTH);

        // depart
        move_group.clearPoseTargets();
        move_group.setStartStateToCurrentState();

        root_action_descriptor.gripper_pose.pose.position.z += 0.07;
        child_action_descriptor.gripper_pose.pose.position.z += 0.07;

        move_group.setPoseTarget(root_action_descriptor.gripper_pose, root_action_descriptor.gripper_frame);
        move_group.setPoseTarget(child_action_descriptor.gripper_pose, child_action_descriptor.gripper_frame);

        if (!move_group.move()){
            ROS_ERROR("[PACO increase action]: unable to terminate action execution. Please manually reset the robot.");
            return false;
        }

        // return home
        baxter_manager_ptr->return_arms();
        return true;
    }

    /* action dispatch callback */
    bool PacoIncreaseAngle::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        ROS_INFO("[PACO increase action]: executing action %s, params: %s %s %s", msg->name.c_str(), msg->parameters[0].value.c_str(),
                  msg->parameters[1].value.c_str(), msg->parameters[2].value.c_str());

        // parsing msg
        std::stringstream tmp_sstream;
        std::stringstream root_link_name;
        child_action_descriptor.link_name = msg->parameters.begin()->value;
        int child_link_n = stoi(child_action_descriptor.link_name.substr(4));

        if (child_link_n > 0) root_link_name << "link" << std::to_string(child_link_n - 1);
        root_action_descriptor.link_name = root_link_name.str();

        // acquiring links positions

        try{
            tmp_sstream << "/" << child_action_descriptor.link_name.c_str();
            listener.waitForTransform("/base", tmp_sstream.str(), ros::Time::now(), ros::Duration(10.0));
            listener.lookupTransform("/base", tmp_sstream.str(), ros::Time(0), child_action_descriptor.link_transform);
            tmp_sstream.str("");
            if (child_action_descriptor.link_name != "link0"){
                tmp_sstream << "/" << root_action_descriptor.link_name;
                listener.waitForTransform("/base", tmp_sstream.str(), ros::Time::now(), ros::Duration(10.0));
                listener.lookupTransform("/base", tmp_sstream.str(), ros::Time(0), root_action_descriptor.link_transform);
            }
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            return false;
        }

        ROS_INFO("[PACO increase action]: links acquired.");

        // assign closest griper to links

        if (child_action_descriptor.link_transform.getOrigin().getY()
            >= root_action_descriptor.link_transform.getOrigin().getY()){

            child_action_descriptor.arm = ACTION_DESCRIPTORS::LEFT;
            child_action_descriptor.gripper_frame =  "left_gripper";
            child_action_descriptor.gripper_name = "left_arm";
            root_action_descriptor.arm  = ACTION_DESCRIPTORS::RIGHT;
            root_action_descriptor.gripper_frame = "right_gripper";
            root_action_descriptor.gripper_name = "right_arm";
            if (child_action_descriptor.link_name == "link0"){
                listener.waitForTransform("/base", "/right_gripper", ros::Time::now(), ros::Duration(10.0));
                listener.lookupTransform("/base", "/right_gripper", ros::Time(0), root_action_descriptor.link_transform);
            }
        }else{
            child_action_descriptor.arm = ACTION_DESCRIPTORS::RIGHT;
            child_action_descriptor.gripper_frame =  "right_gripper";
            child_action_descriptor.gripper_name = "right_arm";
            root_action_descriptor.arm  = ACTION_DESCRIPTORS::LEFT;
            root_action_descriptor.gripper_frame = "left_gripper";
            root_action_descriptor.gripper_name = "left_arm";
            if (child_action_descriptor.link_name == "link0"){
                listener.waitForTransform("/base", "/left_gripper", ros::Time::now(), ros::Duration(10.0));
                listener.lookupTransform("/base", "/left_gripper", ros::Time(0), root_action_descriptor.link_transform);
            }
        }

        // select grasp type (slightly inclined for horizontal-ish links, perpendicular for the others)
        // TODO replace hardcoded precision with a rosparam

        tfScalar roll, pitch, yaw;
        root_action_descriptor.link_transform.getBasis().getRPY(roll, pitch, yaw);
        yaw = angles::to_degrees(angles::normalize_angle(yaw));
        if ((-15 < yaw && yaw < 15) || (165 < yaw && yaw < 195) || (-180 < yaw && yaw < -165))
            root_action_descriptor.mode = ACTION_DESCRIPTORS::H_GRASP;
        else root_action_descriptor.mode = ACTION_DESCRIPTORS::V_GRASP;

        root_action_descriptor.link_transform.getBasis().getRPY(roll, pitch, yaw);
        yaw = angles::to_degrees(angles::normalize_angle(yaw));
        if ((-15 < yaw && yaw < 15) || (165 < yaw && yaw < 195) || (-180 < yaw && yaw < -165))
            child_action_descriptor.mode = ACTION_DESCRIPTORS::H_GRASP;
        else child_action_descriptor.mode = ACTION_DESCRIPTORS::V_GRASP;

        // moving
        root_action_descriptor.gripper_pose.header.frame_id = "base";
        compute_root_hover_pose();

        child_action_descriptor.gripper_pose.header.frame_id = "base";
        compute_child_hover_pose();

        move_group.setPoseTarget(root_action_descriptor.gripper_pose, root_action_descriptor.gripper_frame);
        move_group.setPoseTarget(child_action_descriptor.gripper_pose, child_action_descriptor.gripper_frame);

        ROS_INFO("[PACO increase action]: getting closer to links.");
        if (!move_group.move()){
            ROS_ERROR("[PACO increase action]: unable to reach target positions.");
            exit();
            return false;
        }else ROS_INFO("[PACO increase action]: approaching links.");

        // approach

        root_action_descriptor.gripper_pose.pose.position.z -= 0.07;
        child_action_descriptor.gripper_pose.pose.position.z -= 0.07;

        move_group.clearPoseTargets();
        move_group.setPoseTarget(root_action_descriptor.gripper_pose, root_action_descriptor.gripper_frame);
        move_group.setPoseTarget(child_action_descriptor.gripper_pose, child_action_descriptor.gripper_frame);

        if (!move_group.move()){
            ROS_ERROR("[PACO increase action]: unable to reach target positions.");
            exit();
            return false;
        }

        baxter_manager_ptr->close_gripper(BaxterUtils::BOTH);

        // rotating child link

        tf::StampedTransform child_gripper_to_base_transform, root_gripper_to_base_transform;
        try{
            listener.waitForTransform("/base", child_action_descriptor.gripper_frame, ros::Time::now(), ros::Duration(10.0));
            listener.lookupTransform("/base", child_action_descriptor.gripper_frame, ros::Time(0), child_gripper_to_base_transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            exit();
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        std::vector<geometry_msgs::Pose> waypoints;
        moveit_msgs::RobotTrajectory trajectory;

        child_action_descriptor.gripper_pose.pose = get_circ_cclockwise_waypoints(waypoints,
                                                                                  child_action_descriptor.link_transform,
                                                                                  child_gripper_to_base_transform);

        moveit::planning_interface::MoveGroupInterface move_group_cartesian(child_action_descriptor.gripper_name);
        move_group_cartesian.allowReplanning(true);
        move_group_cartesian.setPlannerId("RRTConnectkConfigDefault");

        move_group_cartesian.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, false);

        move_group_cartesian.plan(plan);
        move_group_cartesian.setStartStateToCurrentState();
        move_group_cartesian.setGoalPositionTolerance(0.02);
        plan.trajectory_ = trajectory;

        ROS_INFO("[PACO increase action]: rotating link.");
        if (!move_group_cartesian.execute(plan)) {
            // recover current position and return home
            try{
                listener.waitForTransform("/base", child_action_descriptor.gripper_frame, ros::Time::now(), ros::Duration(10.0));
                listener.lookupTransform("/base", child_action_descriptor.gripper_frame, ros::Time(0), child_gripper_to_base_transform);
                listener.waitForTransform("/base", root_action_descriptor.gripper_frame, ros::Time::now(), ros::Duration(10.0));
                listener.lookupTransform("/base", root_action_descriptor.gripper_frame, ros::Time(0), root_gripper_to_base_transform);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s", ex.what());
            }
            compute_child_hover_pose();
            compute_root_hover_pose();
            move_group.setPoseTarget(root_action_descriptor.gripper_pose, root_action_descriptor.gripper_frame);
            move_group.setPoseTarget(child_action_descriptor.gripper_pose, child_action_descriptor.gripper_frame);
            ROS_ERROR("[PACO increase action]: unable to completely rotate link. Aborting action.");
        }

        bool success = exit();
        ROS_INFO("[PACO increase action]: action successful.");
        return success;
    }
}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "paco_increase_angle");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    KCL_rosplan::nh_ptr = ros::NodeHandlePtr(new ros::NodeHandle("~"));
    KCL_rosplan::baxter_manager_ptr = new BaxterUtils(KCL_rosplan::nh_ptr);
    KCL_rosplan::nh_ptr->param("/paco/resolution", KCL_rosplan::resolution, 4);

    std::string actionserver;
    KCL_rosplan::nh_ptr->param("action_server", actionserver, std::string("/increase_angle"));

    // create PDDL action subscriber
    KCL_rosplan::PacoIncreaseAngle increaseAngle(*KCL_rosplan::nh_ptr, actionserver);

    // listen for action dispatch
    ros::Subscriber ds = KCL_rosplan::nh_ptr->subscribe("/kcl_rosplan/action_dispatch", 1000,
                                      &KCL_rosplan::RPActionInterface::dispatchCallback,
                                      dynamic_cast<KCL_rosplan::RPActionInterface*>(&increaseAngle));

    KCL_rosplan::tf_.reset(new tf::TransformListener());
    KCL_rosplan::planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description", KCL_rosplan::tf_));

    spinner.start();
    increaseAngle.runActionInterface();
    spinner.stop();
    return 0;
}