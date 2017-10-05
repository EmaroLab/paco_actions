/**
 * A set of utility functions for the Baxter robot
 * @author Alessio Capitanelli
 */

#include <paco_actions/baxter_utils.h>

BaxterUtils::BaxterUtils(ros::NodeHandlePtr nh_ptr) {
    node_handle_ptr = nh_ptr;
    right_joint_publisher = node_handle_ptr->advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);
    left_joint_publisher  = node_handle_ptr->advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1);
    grippers_publisher    = node_handle_ptr->advertise<baxter_gripper_manager::GripperCommand>("/baxter_gripper_control", 1);
}

/**
 * Opens the Baxter Grippers. `baxter_gripper_manager` must be running.
 * @param arm[in] BaxterUtils::BOTH, BaxterUtils::RIGHT or BaxterUtils::LEFT, else nothing happens
 */
void BaxterUtils::open_gripper(int arm) {
    baxter_gripper_manager::GripperCommand gripper_cmd;
    switch (arm){
        case ARM::BOTH:
            gripper_cmd.command = "open";
            gripper_cmd.arm = "both";
            grippers_publisher.publish(gripper_cmd);
            break;
        case ARM::LEFT:
            gripper_cmd.command = "open";
            gripper_cmd.arm = "left";
            grippers_publisher.publish(gripper_cmd);
            break;
        case ARM::RIGHT:
            gripper_cmd.command = "open";
            gripper_cmd.arm = "right";
            grippers_publisher.publish(gripper_cmd);
            break;
        default:
            break;
    }
    return;
}

/**
 * Closes the Baxter Grippers. `baxter_gripper_manager` must be running.
 * @param arm[in]BaxterUtils::BOTH, BaxterUtils::RIGHT or BaxterUtils::LEFT, else nothing happens
 */
void BaxterUtils::close_gripper(int arm) {
    baxter_gripper_manager::GripperCommand gripper_cmd;
    switch (arm){
        case ARM::BOTH:
            gripper_cmd.command = "close";
            gripper_cmd.arm = "both";
            grippers_publisher.publish(gripper_cmd);
            break;
        case ARM::LEFT:
            gripper_cmd.command = "close";
            gripper_cmd.arm = "left";
            grippers_publisher.publish(gripper_cmd);
            break;
        case ARM::RIGHT:
            gripper_cmd.command = "close";
            gripper_cmd.arm = "right";
            grippers_publisher.publish(gripper_cmd);
            break;
        default:
            break;
    }
    return;
}

/**
 * Returns the arm to a neutral position.
 * Each joint is set exactly in between its joint limits for maximum maneuverability.
 */
void BaxterUtils::return_arms() {
    move_joints({0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0, 0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0}, ARM::BOTH);
    return;
}

/**
 * Move arms joints to a desired joint configuration. A 0.2Hz filter is applied to smooth the movement
 * @param joint_cmd[in] size 7 for single arm queries and 14 for dual arm ([left, right])
 * @param arm[in] BaxterUtils::BOTH, BaxterUtils::RIGHT or BaxterUtils::LEFT, else nothing happens
 */
void BaxterUtils::move_joints(const std::vector<double>& joint_cmd, int arm) {
    if (arm != ARM::BOTH && joint_cmd.size() != 7) {
        ROS_ERROR("[BaxterUtils]: joint commands must be of length 7 for single arm queries, %i provided.", (int)joint_cmd.size());
        return;
    }else{
        if (joint_cmd.size() != 14) {
            ROS_ERROR("[BaxterUtils]: joint commands must be of length 14 for dual arm queries, %i provided.", (int)joint_cmd.size());
            return;
        }
    }

    std::vector<double> right_arm_state, left_arm_state, both_arm_state;
    std::vector<double> right_arm_ref, left_arm_ref;
    std::vector<double> right_arm_cmd, left_arm_cmd;
    bool stop_right_arm, stop_left_arm;

    get_arm_joint_states(both_arm_state, ARM::BOTH);

    if (arm == ARM::LEFT)  {
        left_arm_state  = std::vector<double>(both_arm_state.begin(), both_arm_state.begin() +6);
        left_arm_ref    = joint_cmd;
        stop_left_arm   = check_threshold(joint_diff(left_arm_ref, left_arm_state));
        stop_right_arm  = true;
    }
    else if (arm == ARM::RIGHT) {
        right_arm_state = std::vector<double>(both_arm_state.begin() + 7, both_arm_state.end());
        right_arm_ref   = joint_cmd;
        stop_right_arm  = check_threshold(joint_diff(right_arm_ref, right_arm_state));
        stop_left_arm   = true;
    }
    else if (arm == ARM::BOTH)  {
        left_arm_state  = std::vector<double>(both_arm_state.begin(), both_arm_state.begin() + 6);
        right_arm_state = std::vector<double>(both_arm_state.begin() + 7, both_arm_state.begin() + 13);
        left_arm_ref    = std::vector<double>(joint_cmd.begin(), joint_cmd.begin() + 6);
        right_arm_ref   = std::vector<double>(joint_cmd.begin() + 7, joint_cmd.begin() + 13);
        stop_left_arm   = check_threshold(joint_diff(left_arm_ref, left_arm_state));
        stop_right_arm  = check_threshold(joint_diff(right_arm_ref, right_arm_state));
    }
    else {
        ROS_ERROR("[BaxterUtils]: invalid arm argument.");
        return;
    }

    ros::Rate pub_rate(100);
    while(!stop_right_arm || !stop_left_arm) {
        get_arm_joint_states(both_arm_state, ARM::BOTH);
        if (arm != ARM::RIGHT && !stop_left_arm){
            left_arm_cmd.clear();
            std::transform(left_arm_state.begin(), left_arm_state.end(), left_arm_ref.begin(),
                           std::back_inserter(left_arm_cmd),
                           [](double a, double b) { return (0.012488 * a) + (0.98751 * b); }); }
        if (arm != ARM::LEFT && !stop_right_arm) {
            right_arm_cmd.clear();
            std::transform(right_arm_state.begin(), right_arm_state.end(), right_arm_ref.begin(),
                           std::back_inserter(right_arm_cmd),
                           [](double a, double b) { return (0.012488 * a) + (0.98751 * b); }); }

        if (arm != ARM::RIGHT && !stop_left_arm)  send_joint_positions(left_arm_cmd,  ARM::LEFT);
        if (arm != ARM::LEFT  && !stop_right_arm) send_joint_positions(right_arm_cmd, ARM::RIGHT);

        if (arm != ARM::RIGHT && !stop_left_arm) {
            left_arm_state = std::vector<double>(both_arm_state.begin(), both_arm_state.begin() + 6);
            stop_left_arm = check_threshold(joint_diff(left_arm_ref, left_arm_state));
        }

        if (arm != ARM::LEFT  && !stop_right_arm) {
            right_arm_state = std::vector<double>(both_arm_state.begin() + 7, both_arm_state.end());
            stop_right_arm = check_threshold(joint_diff(right_arm_ref, right_arm_state));
        }
        pub_rate.sleep();
    }
}

/**
 * Send a single joint command to one or both Baxter arms
 * @param joint_cmd[in] size 7 for single arm queries and 14 for dual arm ([left, right])
 * @param arm[in] BaxterUtils::BOTH, BaxterUtils::RIGHT or BaxterUtils::LEFT, else nothing happens
 */
void BaxterUtils::send_joint_positions(const std::vector<double>& joint_cmd, int arm) {
    baxter_core_msgs::JointCommand cmd;
    cmd.mode = cmd.POSITION_MODE;
    if (arm != ARM::RIGHT) {
        cmd.names = left_joint_names;
        cmd.command = joint_cmd;
        left_joint_publisher.publish(cmd);
    }
    if (arm != ARM::LEFT) {
        cmd.names = right_joint_names;
        if (arm == ARM::BOTH) cmd.command = std::vector<double>(joint_cmd.begin() + 7, joint_cmd.end());
        else cmd.command = joint_cmd;
        right_joint_publisher.publish(cmd);
    }
    return;
}

/**
 * Return the current arm(s) joint state
 * @param arm_state[out] vector where to save the current arm(s) state
 * @param arm[in] arm BaxterUtils::BOTH, BaxterUtils::RIGHT or BaxterUtils::LEFT, else returns an empty vector
 * @return The current arms(s) configuration ([left, right])
 */
void BaxterUtils::get_arm_joint_states(std::vector<double>& arm_state, int arm) {
    sensor_msgs::JointState robot_state =
            *(ros::topic::waitForMessage<sensor_msgs::JointState>("/robot/joint_states", ros::Duration(1)));
    if (robot_state.name.size() != 17) return; // discard bad inputs
    arm_state.clear();
    if (arm != ARM::RIGHT)  arm_state.insert(arm_state.end(), robot_state.position.begin() +2, robot_state.position.begin() + 9);
    if (arm != ARM::LEFT)   arm_state.insert(arm_state.end(), robot_state.position.begin() +9, robot_state.position.begin() + 16);
    // swap joint values to be coherent with joint_states
    std::iter_swap(arm_state.begin(),    arm_state.begin() + 2);
    std::iter_swap(arm_state.begin() +1, arm_state.begin() + 3);
    if (arm == ARM::BOTH)  {
        std::iter_swap(arm_state.begin() + 8, arm_state.begin() + 10);
        std::iter_swap(arm_state.begin() + 9, arm_state.begin() + 11);
    }
    return;
}

/**
 * Computes the difference between two joint configurations
 * @param joint_conf1[in]
 * @param joint_conf2[in]
 * @return @param joint_conf1 - @param joint_conf2
 */
std::vector<double> BaxterUtils::joint_diff(const std::vector<double>& joint_conf1, const std::vector<double>& joint_conf2) {
    std::vector<double> result;
    std::transform(joint_conf1.begin(), joint_conf1.end(), joint_conf2.begin(), std::back_inserter(result),
                   [](double a, double b) { return a - b; });
    return result;
}

/**
 * Checks all the elements of a vector are beneath a certain threshold
 * @param vec[in]
 * @param threshold[in]
 * @return True if all members are below @param threshold, False otherwise
 */
bool BaxterUtils::check_threshold(const std::vector<double>& vec, double threshold) {
    for (int i = 0; i < vec.size(); i++){
        if (fabs(vec[i]) > threshold) return false;
    }
    return true;
}