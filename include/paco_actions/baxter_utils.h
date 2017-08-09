#ifndef PACO_ACTIONS_BAXTER_UTILS_H
#define PACO_ACTIONS_BAXTER_UTILS_H

#endif //PACO_ACTIONS_BAXTER_UTILS_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_gripper_manager/GripperCommand.h>
#include <vector>
#include <algorithm>

class BaxterUtils {
private:
    std::vector<std::string> right_joint_names = {"right_s0", "right_s1", "right_e0", "right_e1",
                                                  "right_w0", "right_w1", "right_w2"};
    std::vector<std::string> left_joint_names = {"left_s0", "left_s1", "left_e0", "left_e1",
                                                 "left_w0", "left_w1", "left_w2"};
    ros::NodeHandlePtr node_handle_ptr;
    ros::Publisher right_joint_publisher;
    ros::Publisher left_joint_publisher;
    ros::Publisher grippers_publisher;
    void send_joint_positions(const std::vector<double>& joint_cmd, int arm);
    void get_arm_joint_states(std::vector<double>& joint_state, int arm);
    std::vector<double> joint_diff(const std::vector<double>& joint_conf1, const std::vector<double>& joint_conf2);
    bool check_threshold(const std::vector<double>& vec, double threshold = 0.01);

public:
    BaxterUtils(ros::NodeHandlePtr);
    void return_arms();
    void move_joints(const std::vector<double>& joint_cmd, int arm);
    void open_gripper(int arm);
    void close_gripper(int arm);

    enum ARM{
        BOTH, LEFT, RIGHT
    };

};

