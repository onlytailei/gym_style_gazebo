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
#define PI 3.14159265359

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
    assert(rosNodeConstPtr->getParam("/ACTOR_NUMBER",actor_number));
    assert(rosNodeConstPtr->getParam("/DESIRED_FORCE_FACTOR",desired_force_factor));
    assert(rosNodeConstPtr->getParam("/SOCIAL_FORCE_FACTOR",social_force_factor));
    //assert(rosNodeConstPtr->getParam("/NEIGHBOR_RANGE",neighbor_range));
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
    ActionPub = this->rosNode_pr->advertise<RL::ACTION_TYPE>("/cmd_vel", 1);
    //VisPub = this->rosNode_pr->advertise<visualization_msgs::Marker>("visualization_marker", 0);
    PytorchService = this->rosNode_pr->advertiseService(service_name, &TaskEnvIO::ServiceCallback, this);
    SetModelPositionClient = this->rosNode_pr->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); 
    GetModelPositionClient = this->rosNode_pr->serviceClient<gazebo_msgs::SetModelState>("/gazebo/get_model_state"); 
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
    ROS_ERROR("Reset the episode");
    // reset over until the termial and collison are all free
    while (CollisionCheck(robot_ignition_state)){
      ROS_ERROR("Reset loop");
      this->reset();
    }
  }
  
  if (!req.reset){
    actionPub(req.sf_force_x, req.sf_force_y); 
  }
  
  std::this_thread::sleep_for(std::chrono::milliseconds(paramlist->action_sleep_time));
  
  std::unique_lock<std::mutex> state_2_lock(topic_mutex);
  assert(state_2->StateVector.size()>0);
  this->newStates = state_2->StateVector.back();
  state_2_lock.unlock();
  
  robot_ignition_state = RL::gazePose2IgnPose(findPosebyName(RL::ROBOT_NAME));
  
  res.reward = rewardCalculate(robot_ignition_state);
  res.terminal = terminal_flag;

  // ROS_ERROR("=================================");
  //ROS_ERROR("Reward: %f", res.reward);
   
  //build image state
  std::unique_lock<std::mutex> state_1_lock(topic_mutex);
  assert(state_1->StateVector.size()>0);
  res.depth_img = *(state_1->StateVector.back());
  state_1_lock.unlock();
  

  ignition::math::Vector3d desired_force = this->target_pose-robot_ignition_state.Pos();
  ignition::math::Angle desired_yaw = std::atan2(desired_force.Y(), desired_force.X())-robot_ignition_state.Rot().Yaw();
  desired_yaw.Normalize(); 
  res.desired_force_x = std::cos(desired_yaw.Radian()); 
  res.desired_force_y = std::sin(desired_yaw.Radian()); 

  //end = std::chrono::system_clock::now();
  //std::chrono::duration<double> elapsed_seconds = end-start;
  //ROS_ERROR("time: %f", elapsed_seconds.count());
  if (res.terminal){
    geometry_msgs::Twist action_out;
    action_out.angular.z = 0;
    action_out.linear.x = 0;
    ActionPub.publish(action_out);
  }

  return true;
}

/////////////////////
void RL::TaskEnvIO::actionPub(const float sf_x, const float sf_y){
  //ignition::math::Vector3d desired_force = this->target_pose-robot_ignition_state.Pos();
  //ignition::math::Angle desired_yaw = std::atan2(desired_force.Y(), desired_force.X())-robot_ignition_state.Rot().Yaw();
  //desired_yaw.Normalize(); 
  //double final_force_x = paramlist->desired_force_factor * std::cos(desired_yaw.Radian()) + paramlist->social_force_factor * sf_x;
  //double final_force_y = paramlist->desired_force_factor * std::sin(desired_yaw.Radian()) + paramlist->social_force_factor * sf_y;
 
  //ignition::math::Angle final_direction = std::atan2(final_force_y, final_force_x);
  //final_direction.Normalize();  
 
  //ROS_ERROR("final force x: %lf, final force y: %lf", final_force_x, final_force_y);
  //ROS_ERROR("final raw angle: %lf", final_direction.Radian());

  geometry_msgs::Twist action_out;
  action_out.linear.x = sf_x * paramlist->max_lin_vel;
  action_out.angular.z = sf_y * paramlist->max_ang_vel;  
  //action_out.angular.z = action_out.angular.z<-1.0?-1.0:action_out.angular.z;  
  //action_out.angular.z = action_out.angular.z> 1.0? 1.0:action_out.angular.z;  
  //action_out.linear.x = action_out.linear.x < 0.2 ? 0.2: action_out.linear.x; 
  //action_out.linear.x = action_out.linear.x > 1 ? 1: action_out.linear.x; 
  ActionPub.publish(action_out);
}

///////////////////////
float RL::TaskEnvIO::rewardCalculate(const ignition::math::Pose3d robot_ign_pose_){
  if (TargetCheck(robot_ign_pose_)){
    terminal_flag = true;
    return paramlist->terminalReward;}
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
  
  robot_pose_.Pos().Z()=0;
  for(int i=0;i<paramlist->actor_number;i++){
    ignition::math::Pose3d ped_pose_ = RL::gazePose2IgnPose(findPosebyName(RL::ACTOR_NAME_BASE+std::to_string(i)));
    ped_pose_.Pos().Z()=0;
    ignition::math::Vector3d ped_direction = ped_pose_.Pos() - robot_pose_.Pos();
    ignition::math::Angle ped_yaw = std::atan2(ped_direction.Y(), ped_direction.X()) - robot_pose_.Rot().Yaw();
    ped_yaw.Normalize();
    //ROS_ERROR("ped yaw %lf, length %lf", ped_yaw.Radian(), ped_direction.Length());
    if (std::fabs(ped_yaw.Radian()) < (paramlist->depth_fov * 0.5)/180*PI && ped_direction.Length() < paramlist->collision_th){
      ROS_ERROR("===========Collision confirmed!!!!============");
      return true;
    }
  }
  return false;
}

///////////////////////
bool RL::TaskEnvIO::reset() {

  geometry_msgs::Twist action_out;
  action_out.angular.z = 0;
  action_out.linear.x = 0;
  ActionPub.publish(action_out);
  
  std::unique_lock<std::mutex> state_2_lock(topic_mutex);
  assert(state_2->StateVector.size()>0);
  this->newStates = state_2->StateVector.back();
  state_2_lock.unlock();

  const float _x = target_gen(random_engine)*(paramlist->robot_x_end-paramlist->robot_x_start)+paramlist->robot_x_start; 
  const float _y = target_gen(random_engine)*(paramlist->robot_y_end-paramlist->robot_y_start)+paramlist->robot_y_start;
  const float _yaw = target_gen(random_engine)*(paramlist->robot_yaw_end-paramlist->robot_yaw_start)+paramlist->robot_yaw_start;
  const geometry_msgs::Quaternion _q_robot = tf::createQuaternionMsgFromYaw(_yaw);
  setModelPosition(_x,_y,_q_robot);
  
  for(int i=0;i<paramlist->actor_number;i++){
    float ped_x;
    float ped_y;
    const float ped_yaw = target_gen(random_engine)*(paramlist->robot_yaw_end-paramlist->robot_yaw_start)+paramlist->robot_yaw_start+PI;
    const geometry_msgs::Quaternion _q_ped = tf::createQuaternionMsgFromYaw(ped_yaw);
    assert(this->rosNode_pr->getParam("/TARGET_X_ACTOR_"+std::to_string(i), ped_x));
    assert(this->rosNode_pr->getParam("/TARGET_Y_ACTOR_"+std::to_string(i), ped_y));
    ROS_ERROR("actor %d, x: %f, y: %f",i, ped_x, ped_y);
    setModelPosition(ped_x, ped_y, _q_ped, "actor"+std::to_string(i)); 
  }

  robot_ignition_state = RL::gazePose2IgnPose(findPosebyName(RL::ROBOT_NAME));
  return false;
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
  model_state = newStates.pose.at(idx_);
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

ignition::math::Quaterniond RL::yaw2Quaterniond(const ignition::math::Angle yaw_){
  return ignition::math::Quaterniond(0,0,yaw_.Radian());
}
