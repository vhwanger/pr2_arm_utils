#pragma once
#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <control_msgs/JointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <ikfast_pr2/ik_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <kdl/frames.hpp>

using namespace std;

typedef actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction> TrajClient;
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;

class Arm
{
  public: 
    
    Arm(std::string arm_name);
    ~Arm();

    void stopArm();
    void setReferenceFrame(std::string frame){ reference_frame_ = frame;};
    void sendArmToPose(double pose[], double move_time);
    void sendArmToPose(geometry_msgs::PoseStamped pose, std::vector<double> initial, double move_time);
    void sendArmToPose(const KDL::Frame&, double free_angle, double move_time);
    void sendArmToPose(const KDL::Frame&, double move_time);
    void sendArmToPoseQuaternion(double pose[], double move_time);
    //void sendArmToPoses(std::vector<std::vector<double> > &poses, std::vector<double> move_times);
    bool sendArmToPoses(std::vector<KDL::Frame> &poses, std::vector<double> move_times);

    void sendArmToConfiguration(double configuration[], double move_time);
    void sendArmToConfigurations(std::vector<std::vector<double> > &configurations, std::vector<double> move_times);
    void sendArmToConfiguration(std::vector<float> configuration, double move_time);

    bool computeIK(const geometry_msgs::Pose &pose, std::vector<double> jnt_pos, std::vector<double> &solution);
    bool computeIK(const geometry_msgs::PoseStamped &pose, std::vector<double> jnt_pos, std::vector<double> &solution);
    bool computeIK(const KDL::Frame& pose, std::vector<double> jnt_pos, std::vector<double> &solution);
    // DEPRECATED bool performFK(const std::vector<double> jnt_pos, std::vector<double> &cart_pose);
    bool performFK(const std::vector<double> jnt_pos, KDL::Frame& pose);


    KDL::Frame transformToolFrameToWristFrame(KDL::Frame tool_frame);



    // 0 is closed, 1 is open, effort of 50 is gentle
    bool moveGripper(double position, double force);
    bool closeGripper();
    bool openGripper();

    void getCurrentArmConfiguration(vector<double>& current_angles);
    // UPDATE THIS void getCurrentArmPose(vector<double>& cpose);

  private:
    ros::NodeHandle nh_;
    //ros::Subscriber joint_states_subscriber_;
  
    IKFastPR2 ikfast_solver_;
    std::string arm_name_;
    std::vector<std::string> joint_names_;
    std::string reference_frame_;

    std::string controller_state_name_;

    tf::TransformListener tf_;
    TrajClient* traj_client_;
    GripperClient* gripper_client_;
};


