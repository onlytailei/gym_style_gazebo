/*************************************************************************
  > File Name: gazebo_env_io.cpp
  > Author: TAI Lei
  > Mail: lei.tai@my.cityu.edu.hk
  > Created Time: Mo 08 Mai 2017 16:54:01 CEST
 ************************************************************************/
#define _USE_MATH_DEFINES
#include <iostream>
#include <ignition/math.hh>
#include <assert.h>
#include <algorithm>
#include <chrono>
#include <thread>
//#include <math.h>
#include <cmath>
#include <time.h>
#include <mutex>
#include <ctime>
#include <random>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h> //roslogging
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/SetModelState.h>
#include <actor_services/SetPose.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "gazebo_env_io.h"
#include "task_env_io.h"
#include <typeinfo>

//protect the read and write for topic vectors
std::mutex topic_mutex;

/// statecallback function
template<typename topicType>
void RL::GetNewTopic<topicType>::StateCallback(const topicType& msg_){
  std::lock_guard<std::mutex> lock(topic_mutex);
  StateVector.push_back(msg_);
  if (StateVector.size() > 5)
    StateVector.erase(StateVector.begin());
}

////////////////////////////
template<typename topicType>
RL::GetNewTopic<topicType>::GetNewTopic(ros::NodeHandlePtr rosNode_pr,
    const std::string topic_name){
  StateSub = rosNode_pr->subscribe(topic_name, 1, &GetNewTopic::StateCallback, this);
}

///////////////////////////
RL::ParamLoad::ParamLoad(ros::NodeHandlePtr rosNode_pr_):
  rosNodeConstPtr(rosNode_pr_){
    assert(rosNodeConstPtr->getParam("/COLLISION_TH", collision_th));
    assert(rosNodeConstPtr->getParam("/COLLISION_REWARD", collisionReward));
    assert(rosNodeConstPtr->getParam("/TERMINAL_REWARD", terminalReward));
    assert(rosNodeConstPtr->getParam("/HARD_PED_TH", hard_ped_th));
    assert(rosNodeConstPtr->getParam("/TARGET_TH", target_th));
    assert(rosNodeConstPtr->getParam("/ROBOT_X_START", robot_x_start));
    assert(rosNodeConstPtr->getParam("/ROBOT_X_END", robot_x_end));
    assert(rosNodeConstPtr->getParam("/ROBOT_Y_START", robot_y_start));
    assert(rosNodeConstPtr->getParam("/ROBOT_Y_END", robot_y_end));
    assert(rosNodeConstPtr->getParam("/ROBOT_YAW_START", robot_yaw_start));
    assert(rosNodeConstPtr->getParam("/ROBOT_YAW_END", robot_yaw_end));
    assert(rosNodeConstPtr->getParam("/ACTION_SLEEP_TIME", action_sleep_time));
    assert(rosNodeConstPtr->getParam("/TIME_DISCOUNT", time_discount));
    assert(rosNodeConstPtr->getParam("/MAX_LINEAR_VAL", max_lin_vel));
    assert(rosNodeConstPtr->getParam("/MAX_ANGULAR_VAL", max_ang_vel));
    assert(rosNodeConstPtr->getParam("/ENABLE_COLLISIOM_TERMINAL", enable_collision_terminal));
    assert(rosNodeConstPtr->getParam("/ENABLE_CONTINUOUS_CONTROL", enable_continuous_control));
    assert(rosNodeConstPtr->getParam("/ENABLE_PED",enable_ped));
    assert(rosNodeConstPtr->getParam("/DEPTH_FOV",depth_fov));
    assert(rosNodeConstPtr->getParam("/NEIGHBOR_RANGE",neighbor_range));
  }

////////////////////////////
RL::TaskEnvIO::TaskEnvIO(
    const std::string service_name,
    const std::string node_name,
    const float sleeping_time):
  GazeboEnvIO(node_name),
  state_1(new RL::GetNewTopic<RL::STATE_1_TYPE>(this->rosNode_pr, "/camera/depth/image_raw")),
  state_2(new RL::GetNewTopic<RL::STATE_2_TYPE>(this->rosNode_pr, "/gazebo/model_states")),
  paramlist(new RL::ParamLoad(this->rosNode_pr)),
  random_engine(0),
  dis(-1,1),  // noise generator
  target_gen(0,1),  //noise generator
  target_pose(0,0,0),
  sleeping_time_(sleeping_time){
    assert(rosNode_pr->getParam("/TARGET_X",target_pose.X()));
    assert(rosNode_pr->getParam("/TARGET_Y",target_pose.Y()));
    ActionPub = this->rosNode_pr->advertise<RL::ACTION_TYPE>("/mobile_base/commands/velocity", 1);
    PytorchService = this->rosNode_pr->advertiseService(service_name, &TaskEnvIO::ServiceCallback, this);
    SetModelPositionClient = this->rosNode_pr->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); 
    GetModelPositionClient = this->rosNode_pr->serviceClient<gazebo_msgs::SetModelState>("/gazebo/get_model_state"); 
    //if (paramlist->enable_ped){SetActorTargetClient = this->rosNode_pr->serviceClient<actor_services::SetPose>("/actor2/SetActorTarget");}
  }

// Set a separate callbackqueue for this service callback 
// Otherwise, the other callbacks will not work when there is a reset loop
bool RL::TaskEnvIO::ServiceCallback(
    gym_style_gazebo::SocialForce::Request &req,
    gym_style_gazebo::SocialForce::Response &res){
  
  //std::chrono::time_point<std::chrono::system_clock> start, end;
  //start = std::chrono::system_clock::now();
  if (req.reset){
    this->reset();
  }
    // reset over until the termial and collison are all free
    while (terminal_flag || CollisionCheck(robot_ignition_state)){
      ROS_ERROR("Reset loop");
      this->reset();
    }
  
  actionPub(req.sf_force_x, req.sf_force_y); 
  
  std::this_thread::sleep_for(std::chrono::milliseconds(paramlist->action_sleep_time));
  
  std::unique_lock<std::mutex> state_2_lock(topic_mutex);
  this->newStates = state_2->StateVector.back();
  state_2_lock.unlock();
  
  robot_ignition_state = RL::gazePose2IgnPose(findPosebyName(RL::ROBOT_NAME));
  
  res.reward = rewardCalculate(robot_ignition_state);
  res.terminal = terminal_flag;

  // ROS_ERROR("=================================");
  ROS_ERROR("Reward: %f", res.reward);
   
  //build image state
  std::unique_lock<std::mutex> state_1_lock(topic_mutex);
  res.depth_img = *(state_1->StateVector.back());
  state_1_lock.unlock();
  
  //end = std::chrono::system_clock::now();
  //std::chrono::duration<double> elapsed_seconds = end-start;
  //ROS_ERROR("time: %f", elapsed_seconds.count());

  return true;
}


void RL::TaskEnvIO::actionPub(const float sf_x, const float sf_y){
  
  ignition::math::Vector3d desired_force = this->target_pose-robot_ignition_state.Pos();
  ignition::math::Angle desired_yaw= std::atan2(desired_force.Y(), desired_force.X())-robot_ignition_state.Rot().Yaw();
  desired_yaw.Normalize(); 
  double final_force_x = desired_force_param * desired_force.Length() * std::cos(desired_yaw.Radian())- social_force_param * sf_x;
  double final_force_y = desired_force_param * desired_force.Length() * std::sin(desired_yaw.Radian())-social_force_param * sf_y;
  
  
  // TODO trans the force to robot velocity  need double check
  geometry_msgs::Twist action_out;
  action_out.angular.z = final_force_x * paramlist->max_ang_vel;  
  action_out.linear.x =  final_force_y * paramlist->max_lin_vel;  
  ActionPub.publish(action_out);
}

///////////////////////
float RL::TaskEnvIO::rewardCalculate(const ignition::math::Pose3d robot_ign_pose_){
  if (TargetCheck(robot_ign_pose_)){
    terminal_flag = true;
    return paramlist->terminalReward;}
  //// from 0.3 to hard_ped_th, -0.02 to -0.05
  //else if (ped_relative_distance < (paramlist->hard_ped_th+0.3)){
    //if (ped_relative_distance<paramlist->hard_ped_th){return paramlist->collisionReward;}
    //else {return -0.1*(paramlist->hard_ped_th+0.3-ped_relative_distance)-0.02;}
  //}
  else if (CollisionCheck(robot_ign_pose_)){
    terminal_flag = paramlist->enable_collision_terminal;
    return paramlist->collisionReward;}
  else {
    terminal_flag = false;
    return paramlist->time_discount;
  }
  return 0;
}

///////////////////////
bool RL::TaskEnvIO::terminalCheck(){
  return terminal_flag;
}

///////////////////////
bool RL::TaskEnvIO::CollisionCheck(ignition::math::Pose3d robot_pose_) const{
  
  for(int i=0;i<RL::ACTOR_NUMERS;i++){
    ignition::math::Pose3d ped_pose_ = RL::gazePose2IgnPose(findPosebyName(RL::ACTOR_NAME_BASE+std::to_string(i)));
    ignition::math::Vector3d ped_direction = ped_pose_.Pos() - robot_pose_.Pos();
    ignition::math::Angle ped_yaw = std::atan2(ped_direction.Y(), ped_direction.X()) - robot_pose_.Rot().Yaw();
    if (std::fabs(ped_yaw.Radian()) < paramlist->depth_fov * 0.5 && ped_direction.Length() < paramlist->neighbor_range)
      return true;
  }
  return false;
  //std::unique_lock<std::mutex> laser_scan_lock(topic_mutex);
  //assert(laser_scan->StateVector.size()>0);
  //std::vector<float> range_array = laser_scan->StateVector.back()->ranges;
  //laser_scan_lock.unlock();
  //range_array.erase(std::remove_if(range_array.begin(), 
        //range_array.end(), 
        //[](float x){return !std::isfinite(x);}), 
      //range_array.end());
  //float min_scan = *std::min_element(std::begin(range_array), std::end(range_array));
  //return  (range_array.size()==0)? true : (min_scan< (paramlist->collision_th)?true:false);
}


///////////////////////
bool RL::TaskEnvIO::reset() {
  // TODO maker sure these is the robot state and target pose

  // Set a new position for one ped
  //if (paramlist->enable_ped){
    //float target_x = target_gen(random_engine)*7-3.5; 
    //float target_y = target_gen(random_engine)*7-3.5;
    //setActorTarget(std::copysign(target_x, dis(random_engine)),
        //std::copysign(target_y, dis(random_engine)));
  //}
  
  // Set a new position for the robot and target, if change target position, should also chagne the br of target
  const float _x = target_gen(random_engine)*(paramlist->robot_x_end-paramlist->robot_x_start)+paramlist->robot_x_start; 
  const float _y = target_gen(random_engine)*(paramlist->robot_y_end-paramlist->robot_y_start)+paramlist->robot_y_start;
  const float _yaw = target_gen(random_engine)*(paramlist->robot_yaw_end-paramlist->robot_yaw_start)+paramlist->robot_yaw_start;
  const geometry_msgs::Quaternion _q_robot = tf::createQuaternionMsgFromYaw(_yaw);
  setModelPosition(_x,_y,_q_robot);
  robot_ignition_state = RL::gazePose2IgnPose(findPosebyName(RL::ROBOT_NAME));
  //setModelPosition(target_pose.x,target_pose.y,_q_target, RL::TARGET_NAME);
  return true;
}

bool RL::TaskEnvIO::setModelPosition(const float x, const float y, const geometry_msgs::Quaternion q, const std::string model_name_ ){

  gazebo_msgs::SetModelState SetModelState_srv;
  geometry_msgs::Point Start_position;
  Start_position.x = x;
  Start_position.y = y;
  Start_position.z = 0.0;

  geometry_msgs::Pose Start_pose;
  Start_pose.position = Start_position;
  Start_pose.orientation = q;

  gazebo_msgs::ModelState Start_modelstate;
  Start_modelstate.model_name = (std::string)model_name_;
  Start_modelstate.pose = Start_pose;

  SetModelState_srv.request.model_state = Start_modelstate;
  return SetModelPositionClient.call(SetModelState_srv);
}


bool RL::TaskEnvIO::setActorTarget(const float x_, const float y_){
  actor_services::SetPose SetActorTarget_srv;
  SetActorTarget_srv.request.set_flag = true;
  SetActorTarget_srv.request.new_x = x_;
  SetActorTarget_srv.request.new_y = y_;
  return SetActorTargetClient.call(SetActorTarget_srv);
}

/////////////////////
//double RL::TaskEnvIO::getRobotYaw(geometry_msgs::Quaternion &state_quat_) const {
//tf::Quaternion quat;
//tf::quaternionMsgToTF(state_quat_, quat);
//double roll, pitch, yaw;
//tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
//return yaw;
//}

/////////////////////
bool RL::TaskEnvIO::TargetCheck(ignition::math::Pose3d robot_pose_){
  float target_distance = (target_pose-robot_pose_.Pos()).Length();
  return (target_distance < paramlist->target_th)? true : false;
}

/////////////////////
geometry_msgs::Pose RL::TaskEnvIO::findPosebyName(const std::string model_name) const {
  geometry_msgs::Pose model_state;
  auto idx_ = std::find(newStates.name.begin(), newStates.name.end(),model_name)-newStates.name.begin();
  //assert(idx_ < names.size());
  //model_state.model_name = model_name;
  model_state = newStates.pose.at(idx_);
  //model_state.twist = newStates.twist.at(idx_);
  return model_state;
}



//void RL::TaskEnvIO::updatePedStates(const geometry_msgs::Pose robot_pose_, const gazebo_msgs::ModelStates newStates_, const std::vector<std::string> names_){


  //for(int i=0;i<RL::ACTOR_NUMERS;i++){
    //auto idx_ = std::find(names_.begin(), names_.end(),RL::ACTOR_NAME_BASE+std::to_string(i))-names_.begin();
    //geometry_msgs::Pose actor_pose_ = newStates_.pose.at(idx_);
    ////geometry_msgs::Twist actor_twist = newStates_.twist.at(idx_);
    //float robot_yaw = getQuaternionYaw(robot_pose_.orientation);
    ////float actor_yaw = getQuaternionYaw(actor_pose_.orientation);
    //float angleref = (atan2(actor_pose_.position.y-robot_pose_.position.y, actor_pose_.position.x-robot_pose_.position.x) - robot_yaw);
    
    //float angleref_norm = (std::abs(angleref)>M_PI? -2*M_PI*std::copysign(1, angleref)+angleref:angleref);
    //assert(std::abs(angleref_norm/M_PI)<=1);
    //float distanceref = std::sqrt(pow((actor_pose_.position.x-robot_pose_.position.x), 2) + pow((actor_pose_.position.y-robot_pose_.position.y), 2));
    
    ////TODO check collision or not based on angleref_norm and distanceref
    ////assert(std::abs(relative_yaw_norm)<1);
  //}
//}

float RL::TaskEnvIO::getQuaternionYaw(const geometry_msgs::Quaternion &state_quat_) const {
  tf::Quaternion quat;
  tf::quaternionMsgToTF(state_quat_, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return (float)yaw;
}

float RL::TaskEnvIO::rewardCalculate(){
  return 0.0;
}

ignition::math::Pose3d RL::gazePose2IgnPose(const geometry_msgs::Pose pose_) {
  return ignition::math::Pose3d(
      pose_.position.x,
      pose_.position.y,
      pose_.position.z,
      pose_.orientation.w,
      pose_.orientation.x,
      pose_.orientation.y,
      pose_.orientation.z);
}
