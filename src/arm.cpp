#include <pr2_arm_utils/arm.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <time.h>

Arm::Arm(std::string arm_name)
{
    reference_frame_ = "base_link";

    arm_name_ = arm_name;

    if(arm_name_.compare("left") == 0)
    {
        controller_state_name_ = "l_arm_controller/state";
        joint_names_.push_back("l_shoulder_pan_joint");
        joint_names_.push_back("l_shoulder_lift_joint");
        joint_names_.push_back("l_upper_arm_roll_joint");
        joint_names_.push_back("l_elbow_flex_joint");
        joint_names_.push_back("l_forearm_roll_joint");
        joint_names_.push_back("l_wrist_flex_joint");
        joint_names_.push_back("l_wrist_roll_joint");
        traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);
        gripper_client_ = new GripperClient("l_gripper_controller/gripper_action", true);
    }
    else if (arm_name_.compare("right") == 0)
    {
        controller_state_name_ = "r_arm_controller/state";
        joint_names_.push_back("r_shoulder_pan_joint");
        joint_names_.push_back("r_shoulder_lift_joint");
        joint_names_.push_back("r_upper_arm_roll_joint");
        joint_names_.push_back("r_elbow_flex_joint");
        joint_names_.push_back("r_forearm_roll_joint");
        joint_names_.push_back("r_wrist_flex_joint");
        joint_names_.push_back("r_wrist_roll_joint");
        traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
        gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);
    }
    else
    {
        ROS_ERROR("Arm Class was initialized with %s, needs to be right or left", arm_name_.c_str());
    }

    while(!traj_client_->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the joint_trajectory_action server");

    ROS_INFO("Initialized successfully initialized.");
}

Arm::~Arm()
{
    delete traj_client_;
    delete gripper_client_;
}

void Arm::stopArm()
{
    ROS_INFO("Stopping Arm by canceling all trajectory goals.");
    traj_client_->cancelAllGoals();
}

void Arm::sendArmToConfiguration(double configuration[7], double move_time)
{
    control_msgs::JointTrajectoryGoal goal;

    goal.trajectory.header.seq = 0;
    goal.trajectory.header.stamp = ros::Time::now();
    goal.trajectory.header.frame_id = reference_frame_;

    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(7);

    for(unsigned int i = 0; i < 7; i++)
    {
        goal.trajectory.points[0].positions[i] = configuration[i];
        goal.trajectory.joint_names.push_back(joint_names_[i]);
        ROS_INFO("%s is going to %f", joint_names_[i].c_str(), configuration[i]);
    }

    goal.trajectory.points[0].velocities.resize(7);
    //  for (size_t j = 0; j < 7; ++j)
    //    goal.trajectory.points[0].velocities[j] = 1;

    goal.trajectory.points[0].time_from_start = ros::Duration(move_time);

    ROS_INFO("[arm] Trajectory client will now send the goal...");
    double start_time = ros::Time::now().toSec();
    traj_client_->sendGoal(goal);
    ROS_INFO("[arm] Sending goal to controller took %f seconds (no feedback here)", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
}

void Arm::sendArmToConfiguration(std::vector<float> configuration, double move_time)
{
    control_msgs::JointTrajectoryGoal goal;

    goal.trajectory.header.seq = 0;
    goal.trajectory.header.stamp = ros::Time::now();
    goal.trajectory.header.frame_id = reference_frame_;

    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(7);

    for(unsigned int i = 0; i < 7; i++)
    {
        goal.trajectory.points[0].positions[i] = configuration[i];
        goal.trajectory.joint_names.push_back(joint_names_[i]);
    }

    goal.trajectory.points[0].velocities.resize(7);
    //  for (size_t j = 0; j < 7; ++j)
    //    goal.trajectory.points[0].velocities[j] = 1;

    goal.trajectory.points[0].time_from_start = ros::Duration(move_time);

    ROS_INFO("[arm] Trajectory client will now send the goal...");
    double start_time = ros::Time::now().toSec();
    traj_client_->sendGoal(goal);
    ROS_INFO("sending goal to controller took %f seconds (no feedback here)", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
}

void Arm::sendArmToConfigurations(std::vector<std::vector<double> > &configurations, std::vector<double> move_times)
{
    control_msgs::JointTrajectoryGoal goal;

    goal.trajectory.header.seq = 0;
    goal.trajectory.header.stamp = ros::Time::now();
    goal.trajectory.header.frame_id = reference_frame_;

    goal.trajectory.points.resize(configurations.size());
    double accum_time = 0;
    for(unsigned int i = 0; i < 7; i++)
        goal.trajectory.joint_names.push_back(joint_names_[i]);

    for(unsigned int i = 0; i < configurations.size(); i++)
    {
        goal.trajectory.points[i].time_from_start = ros::Duration(accum_time + move_times[i]);
        accum_time += move_times[i];

        goal.trajectory.points[i].positions.resize(7);
        for(unsigned int j = 0; j < 7; j++)
        {
            goal.trajectory.points[i].positions[j] = configurations[i][j];
        }

        goal.trajectory.points[i].velocities.resize(7);
        for (size_t j = 0; j < 7; ++j)
            goal.trajectory.points[i].velocities[j] = 0;
    }

    double start_time = ros::Time::now().toSec();
    traj_client_->sendGoal(goal);
    ROS_DEBUG("sending goal to controller took %f seconds.", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
}

/*
   void Arm::sendArmToPose(double pose[6])
   {
   conductor::ExecuteCartesianIKTrajectory::Request request;
   conductor::ExecuteCartesianIKTrajectory::Response response;

   btQuaternion quaternion;
   geometry_msgs::Quaternion quaternion_msg;

   ros::ServiceClient client = nh_.serviceClient<conductor::ExecuteCartesianIKTrajectory>("ik_trajectory", true);

   request.header.frame_id = reference_frame_;
   request.header.stamp = ros::Time::now();

   request.poses.resize(1);
   request.poses[0].position.x = pose[0];
   request.poses[0].position.y = pose[1];
   request.poses[0].position.z = pose[2];

   quaternion.setRPY(pose[3],pose[4],pose[5]);
   tf::quaternionTFToMsg(quaternion, quaternion_msg);

   request.poses[0].orientation = quaternion_msg;

   if(client.call(request,response))
   {
   if(response.success)
   ROS_DEBUG("successfully went to pose");
   else
   ROS_ERROR("wtf bitch. can't go to pose");
   }

   }
   */

void Arm::sendArmToPose(double pose[6], double move_time)
{
    tf::Quaternion quaternion;
    geometry_msgs::Quaternion quaternion_msg;
    quaternion.setRPY(pose[3],pose[4],pose[5]);
    tf::quaternionTFToMsg(quaternion, quaternion_msg);

    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = pose[0];
    pose_msg.position.y = pose[1];
    pose_msg.position.z = pose[2];
    pose_msg.orientation = quaternion_msg;

    std::vector<double> jnt_pos(7,0), solution(7,0);

    double start_time = ros::Time::now().toSec();
    if(computeIK(pose_msg,jnt_pos,solution))
    {
        ROS_DEBUG("computed IK solution in %0.3f seconds", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
    }
    else
        return;

    double configuration[7];

    for(int i = 0; i < 7; i++)
        configuration[i] = solution[i];

    sendArmToConfiguration(configuration, move_time);
}

void Arm::sendArmToPose(geometry_msgs::PoseStamped pose, std::vector<double> initial, double move_time)
{
    ROS_INFO("[arm] Sending arm to pose:  %0.3f %0.3f %0.3f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    std::vector<double> jnt_pos, solution(7,0);
    geometry_msgs::PoseStamped pose_out = pose;;

    ROS_INFO("[arm] Computing IK..."); 

    jnt_pos = initial;

    if(pose.header.frame_id.compare("torso_lift_link") != 0)
    {
        pose.header.stamp = ros::Time();
        tf_.transformPose(std::string("torso_lift_link"), pose, pose_out);
        ROS_INFO("[arm] Converting pose to:  %0.3f %0.3f %0.3f", pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z);
    }

    ROS_INFO("[arm] Computing IK..."); 
    double start_time = ros::Time::now().toSec();
    if(computeIK(pose_out,jnt_pos,solution))
    {
        ROS_DEBUG("computed IK solution in %0.3f seconds", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
    }
    else
    {
        ROS_WARN("[arm] Failed to compute IK");
        return;
    }

    double configuration[7];

    for(int i = 0; i < 7; i++)
        configuration[i] = solution[i];

    ROS_INFO("[arm] Sending arm to the configuration now...");

    sendArmToConfiguration(configuration, move_time);

    ROS_INFO("[arm] Sent arm to configuration");
}

void Arm::sendArmToPose(const KDL::Frame& pose, double free_angle, double move_time){
    vector<double> angles(7,0);
    vector<double> soln(7,0);
    if (!computeIK(pose, angles, soln)){
        ROS_ERROR("Couldn't compute IK!");
    }
    sendArmToConfiguration(&soln[0], move_time);
}

void Arm::sendArmToPose(const KDL::Frame& pose, double move_time){
    vector<double> angles(7,0);
    vector<double> soln(7,0);
    bool success = false;
    bool search_free_angle = true;
    ROS_INFO("here");
    if (arm_name_ == "right"){
        ROS_INFO("right");
        success = ikfast_solver_.ikRightArm(pose, 2.3324484900413505, &soln, search_free_angle);
    } else if (arm_name_ == "left"){
        ROS_INFO("left");
        success = ikfast_solver_.ikLeftArm(pose, .000948, &soln, search_free_angle);
    }

    KDL::Frame fk;
    performFK(soln, fk);
    ROS_INFO("solution found was %f %f %f %f %f %f %f",
              soln[0],
              soln[1],
              soln[2],
              soln[3],
              soln[4],
              soln[5],
              soln[6]);
    ROS_INFO("sendArmToPose computed. fk says it's gone to %f %f %f",
             fk.p.x(),
             fk.p.y(),
             fk.p.z());

    if (!success){
        ROS_ERROR("ik failed!");
    } else {
        sendArmToConfiguration(&soln[0], move_time);
    }
}

void Arm::sendArmToPoseQuaternion(double pose[7], double move_time)
{
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = pose[0];
    pose_msg.position.y = pose[1];
    pose_msg.position.z = pose[2];
    pose_msg.orientation.x = pose[3];
    pose_msg.orientation.y = pose[4];
    pose_msg.orientation.z = pose[5];
    pose_msg.orientation.w = pose[6];

    std::vector<double> jnt_pos(7,0), solution(7,0);

    double start_time = ros::Time::now().toSec();
    if(computeIK(pose_msg,jnt_pos,solution))
    {
        ROS_DEBUG("computed IK solution in %0.3f seconds", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
    }
    else
        return;

    double configuration[7];

    for(int i = 0; i < 7; i++)
        configuration[i] = solution[i];

    sendArmToConfiguration(configuration, move_time);
}

//void Arm::sendArmToPoses(std::vector<std::vector<double> > &poses, std::vector<double> move_times)
//{
//    geometry_msgs::Pose pose_msg;
//    tf::Quaternion quaternion;
//    geometry_msgs::Quaternion quaternion_msg;
//    std::vector<std::vector<double> > configurations(poses.size(), std::vector<double> (7,0));
//
//    std::vector<double> jnt_pos(7,0), solution(7,0);
//
//    for(unsigned int i =0; i < poses.size(); i++)
//    {
//        quaternion.setRPY(poses[i][3],poses[i][4],poses[i][5]);
//        tf::quaternionTFToMsg(quaternion, quaternion_msg);
//
//        pose_msg.position.x = poses[i][0];
//        pose_msg.position.y = poses[i][1];
//        pose_msg.position.z = poses[i][2];
//        pose_msg.orientation = quaternion_msg;
//
//        double start_time = ros::Time::now().toSec();
//        if(computeIK(pose_msg,jnt_pos,configurations[i]))
//            ROS_DEBUG("[sendArmToPoses] Computed IK solution in %0.3f seconds", ros::Duration(ros::Time::now().toSec() - start_time).toSec());
//        else
//            return;
//    }
//
//    sendArmToConfigurations(configurations, move_times);
//}

bool Arm::sendArmToPoses(vector<KDL::Frame> &poses, vector<double> move_times){
    vector<vector<double> > configurations;
    for (size_t i=0; i < poses.size(); i++){
        vector<double> angles(7,0);
        vector<double> soln(7,0);
        if (!computeIK(poses[i], angles, soln)){
            ROS_ERROR("Couldn't compute IK!");
            return false;
        }
        configurations.push_back(soln);
    }
    sendArmToConfigurations(configurations, move_times);
    return true;
}


bool Arm::computeIK(const geometry_msgs::Pose &pose, std::vector<double> jnt_pos, std::vector<double> &solution)
{
    geometry_msgs::PoseStamped spose;

    spose.pose = pose;
    spose.header.stamp = ros::Time();
    spose.header.frame_id = reference_frame_;

    if(!computeIK(spose, jnt_pos, solution))
        return false;

    return true;
}

bool Arm::computeIK(const geometry_msgs::PoseStamped &pose, std::vector<double> jnt_pos, std::vector<double> &solution)
{
    KDL::Frame kdl_pose;
    tf::poseMsgToKDL(pose.pose, kdl_pose);
    return computeIK(kdl_pose, jnt_pos, solution);
}

bool Arm::computeIK(const KDL::Frame& kdl_pose, std::vector<double> jnt_pos, std::vector<double> &solution){
    bool success = false;
    if (arm_name_ == "right"){
        success = ikfast_solver_.ikRightArm(kdl_pose, jnt_pos[2], &solution);
    } else if (arm_name_ == "left"){
        success = ikfast_solver_.ikLeftArm(kdl_pose, jnt_pos[2], &solution);
    }
    return success;
}

//bool Arm::performFK(std::vector<double> jnt_pos, std::vector<double> &cart_pose)
//{
//    KDL::Frame pose;
//    if (!performFK(jnt_pos, pose)){
//        return false;
//    }
//    cart_pose.clear();
//    cart_pose.push_back(pose.p.x());
//    cart_pose.push_back(pose.p.y());
//    cart_pose.push_back(pose.p.z());
//    double qx, qy, qz, qw;
//    pose.M.GetQuaternion(qx, qy, qz, qw);
//    cart_pose.push_back(qx);
//    cart_pose.push_back(qy);
//    cart_pose.push_back(qz);
//    cart_pose.push_back(qx);
//    return true;
//}

bool Arm::performFK(std::vector<double> jnt_pos, KDL::Frame& pose)
{
    if (arm_name_ == "right"){
        pose = ikfast_solver_.fkRightArm(jnt_pos);
    } else if (arm_name_ == "left"){
        pose = ikfast_solver_.fkLeftArm(jnt_pos);
    }
    return true;
}

void Arm::getCurrentArmConfiguration(vector<double>& current_angles)
{
    //get a single message from the topic 'r_arm_controller/state' or 'l_arm_controller/state'
    ROS_DEBUG("getCurrentArmConfiguration");
    control_msgs::JointTrajectoryControllerStateConstPtr state_msg = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>(controller_state_name_, nh_, ros::Duration(2.0));
    ROS_DEBUG("got it");
    //extract the joint angles from it
    if(state_msg != NULL)
        current_angles = state_msg->actual.positions;
}

//void Arm::getCurrentArmPose(vector<double>& cpose)
//{
//    // Get the current joint angles of the robot.
//    vector<double> current_angles;
//    getCurrentArmConfiguration(current_angles);
//
//    // Perform forward kinematics to figure out what the arm pose is.
//    vector<double> current_pose;
//    performFK(current_angles, current_pose);
//
//    cpose = current_pose;
//}

/*
   int main(int argc, char**argv)
   {
   ros::init(argc,argv,"arm");

   Arm arm("right");
   double pose[6] = {0.7,-0.2,0.74, 0,0,0};

   arm.sendArmToPose(pose);

   sleep(15);

   return 0;
   }
   */

bool Arm::moveGripper(double position, double force){
    control_msgs::GripperCommandGoal movement;
    movement.command.position = position;
    movement.command.max_effort = force;  // Do not limit effort (negative)

    gripper_client_->sendGoal(movement);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        return true;
    else
        return false;
}

bool Arm::closeGripper(){
    return moveGripper(0, 50);
}

bool Arm::openGripper(){
    return moveGripper(.08, -1);
}

KDL::Frame Arm::transformToolFrameToWristFrame(KDL::Frame tool_frame){
    KDL::Vector v(.18,0,0);
    KDL::Rotation rot = KDL::Rotation::Quaternion(0,0,0,1);
    KDL::Frame static_transform(rot,v);
    return tool_frame * static_transform.Inverse();
}
