/*************************************************************************
  > File Name: gazebo_env_io.cpp
  > Author: TAI Lei
  > Mail: lei.tai@my.cityu.edu.hk
  > Created Time: Mo 08 Mai 2017 16:54:01 CEST
 ************************************************************************/
#define _USE_MATH_DEFINES
#include <iostream>
#include <assert.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <math.h>
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
    assert(rosNodeConstPtr->getParam("/COLLISION_TH",collision_th));
    assert(rosNodeConstPtr->getParam("/DISTANCE_COEF",distance_coef));
    assert(rosNodeConstPtr->getParam("/FAIL_REWARD",failReward));
    assert(rosNodeConstPtr->getParam("/TERMINAL_REWARD",terminalReward));
    assert(rosNodeConstPtr->getParam("/TARGET_TH",target_th));
    assert(rosNodeConstPtr->getParam("/TARGET_X",origin_x));
    assert(rosNodeConstPtr->getParam("/TARGET_Y",origin_y));
    assert(rosNodeConstPtr->getParam("/TARGET_START",target_start));
    assert(rosNodeConstPtr->getParam("/TARGET_END",target_end));
    assert(rosNodeConstPtr->getParam("/TIME_DISCOUNT",time_discount));
    assert(rosNodeConstPtr->getParam("/MAX_LINEAR_VAL",max_lin_vel));
    assert(rosNodeConstPtr->getParam("/MAX_ANGULAR_VAL",max_ang_vel));
    assert(rosNodeConstPtr->getParam("/ENABLE_COLLISIOM_TERMINAL",enable_collision_terminal));
    assert(rosNodeConstPtr->getParam("/ENABLE_CONTINUOUS_CONTROL",enbale_continuous_control));
    assert(rosNodeConstPtr->getParam("/ENABLE_PED",enbale_ped));
}

////////////////////////////
RL::TaskEnvIO::TaskEnvIO(
    const std::string service_name,
    const std::string node_name,
    const float sleeping_time):
  GazeboEnvIO(node_name),
  state_1(new RL::GetNewTopic<RL::STATE_1_TYPE>(this->rosNode_pr, "/camera/depth/image_raw")),
  state_2(new RL::GetNewTopic<RL::STATE_2_TYPE>(this->rosNode_pr, "/gazebo/model_states")),
  laser_scan(new RL::GetNewTopic<sensor_msgs::LaserScanConstPtr>(this->rosNode_pr, "/fakescan")),
  paramlist(new RL::ParamLoad(this->rosNode_pr)),
  target_pose_{0,0},
  robot_state_{{0,0,0,0}}, //double brace for std::array
  random_engine(0),
  dis(-1,1),  // noise generator
  target_gen(0,1),  //noise generator
  sleeping_time_(sleeping_time){

    ActionPub = this->rosNode_pr->advertise<RL::ACTION_TYPE>("/mobile_base/commands/velocity", 1);
    PytorchService = this->rosNode_pr->advertiseService(service_name, &TaskEnvIO::ServiceCallback, this);
    SetRobotPositionClient = this->rosNode_pr->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); 
    if (paramlist->enable_ped){SetActorTargetClient = this->rosNode_pr->serviceClient<actor_services::SetPose>("/actor2/SetActorTarget");}
  }

// Set a separate callbackqueue for this service callback 
// Otherwise, the other callbacks will not work when there is a reset loop
bool RL::TaskEnvIO::ServiceCallback(
    gym_style_gazebo::PytorchRL::Request &req,
    gym_style_gazebo::PytorchRL::Response &res){

  if (req.reset){
    this->reset();
    // reset over until the termial and collison are all free
    while (terminal_flag or collision_check()){
      ROS_ERROR("Reset loop");
      this->reset();
    }
  }
  actionPub(req.action); 
  getRobotState(); //update robot state
  res.reward = rewardCalculate();
  res.terminal = terminal_flag;
  std::unique_lock<std::mutex> state_1_lock(topic_mutex);
  cv_ptr = cv_bridge::toCvCopy(state_1->StateVector.back(), state_1->StateVector.back()->encoding);
  state_1_lock.unlock();
  
  //ROS_ERROR("=================================");
  ROS_ERROR("Reward: %f", res.reward);
  
  {
  res.state_1.layout.dim.push_back(std_msgs::MultiArrayDimension());
  res.state_1.layout.dim.push_back(std_msgs::MultiArrayDimension());
  res.state_1.layout.dim[0].size = cv_ptr->image.rows;
  res.state_1.layout.dim[0].stride = cv_ptr->image.cols*cv_ptr->image.rows;
  res.state_1.layout.dim[0].label = "height";
  res.state_1.layout.dim[1].size = cv_ptr->image.cols;
  res.state_1.layout.dim[1].stride = cv_ptr->image.cols;
  res.state_1.layout.dim[1].label = "width";
  res.state_1.data.clear();
  std::vector<float> output_img((float*)cv_ptr->image.data, (float*)cv_ptr->image.data + cv_ptr->image.cols * cv_ptr->image.rows);
  res.state_1.data.reserve(cv_ptr->image.cols*cv_ptr->image.rows);
  res.state_1.data.insert(res.state_1.data.end(), output_img.begin(), output_img.end());
  }
  //Build second state
  {
  res.state_2.layout.dim.push_back(std_msgs::MultiArrayDimension());
  res.state_2.layout.dim[0].size = robot_state_.size();
  res.state_2.layout.dim[0].stride = robot_state_.size();
  res.state_2.layout.dim[0].label = "roobot state";
  res.state_2.data.clear();
  res.state_2.data.reserve(4);
  res.state_2.data.insert(res.state_2.data.end(), robot_state_.begin(),robot_state_.end());
  }
  return true;
}

void RL::TaskEnvIO::actionPub(geometry_msgs::Twist action_out){
  
  if (paramlist->enable_continuous_control){
    //velocity angular
    robot_state_.at(2) = action_out.angular.z;
    action_out.angular.z = action_out.angular.z*paramlist->max_ang_vel;  
    //velocity linear
    robot_state_.at(3) = action_out.linear.x;
    action_out.linear.x = action_out.linear.x*paramlist->max_lin_vel;  
    
    ActionPub.publish(action_out);
  }
  else {
    //TODO: pub the action and return the velocity realtime
    ActionPub.publish(action_out);
  };
}

///////////////////////
float RL::TaskEnvIO::rewardCalculate(){
  if (collision_check()){
    terminal_flag = paramlist->enable_collision_terminal;
    return paramlist->failReward;}
  else if (target_check()){
    terminal_flag = true;
    return paramlist->terminalReward;}
  else {
    terminal_flag = false;
    //float temp = previous_distance;
    previous_distance = robot_state_.at(1);
    //return distance_coef * (temp - robot_state_.at(1))-time_discount;}
    return paramlist->time_discount;}
  return 0;
}

///////////////////////
bool RL::TaskEnvIO::terminalCheck(){
  return terminal_flag;
}

///////////////////////
bool RL::TaskEnvIO::collision_check(){
  std::unique_lock<std::mutex> laser_scan_lock(topic_mutex);
  assert(laser_scan->StateVector.size()>0);
  std::vector<float> range_array = laser_scan->StateVector.back()->ranges;
  laser_scan_lock.unlock();
  //float min_scan_ = *std::min_element(std::begin(range_array), std::end(range_array));
  //float max_scan_ = *std::max_element(std::begin(range_array), std::end(range_array));
  //ROS_ERROR("Before max: %f, min: %f", max_scan_, min_scan_);
  range_array.erase(std::remove_if(range_array.begin(), 
        range_array.end(), 
        [](float x){return !std::isfinite(x);}), 
      range_array.end());
  float min_scan = *std::min_element(std::begin(range_array), std::end(range_array));
  //float max_scan = *std::max_element(std::begin(range_array), std::end(range_array));
  //ROS_ERROR("after max: %f, min: %f, range_size: %zu", max_scan, min_scan, range_array.size());
  return  (range_array.size()==0)? true : (min_scan< (paramlist->collision_th)?true:false);
}

/// use tf to call the robot position DEPRECATED too slow
bool RL::TaskEnvIO::target_check(){
  return (robot_state_.at(1) <paramlist->target_th)? true : false;
}

///////////////////////
bool RL::TaskEnvIO::reset() {
  // TODO
  // Set random position for pedes
  
  //ROS_ERROR("=======Reset======");
  // Set a new position for the target
  float _x = target_gen(random_engine)*(paramlist->target_end-paramlist->target_start)+paramlist->target_start; 
  float _y = target_gen(random_engine)*(paramlist->target_end-paramlist->target_start)+paramlist->target_start;
  target_pose_.x = std::copysign(_x, dis(random_engine));
  target_pose_.y = std::copysign(_y, dis(random_engine));

  // Set a new position for one ped
  float target_x = target_gen(random_engine)*(1.7-0.5)+0.5; 
  float target_y = target_gen(random_engine)*(1.7-0.5)+0.5;
  setActorTarget(std::copysign(target_x, dis(random_engine)),
                std::copysign(target_y, dis(random_engine)));

  rosNode_pr->setParam("/TARGET_X",target_pose_.x);
  rosNode_pr->setParam("/TARGET_Y",target_pose_.y);
  // Set a new target for the robot
  setRobotPosition();
  getRobotState(); //update robot state
  previous_distance = robot_state_.at(1);
  rewardCalculate(); //check if it is terminal
  return true;
}

bool RL::TaskEnvIO::setRobotPosition(){
  
  gazebo_msgs::SetModelState SetModelState_srv;
  geometry_msgs::Point Start_position;
  Start_position.x = 0.0;
  Start_position.y = 0.0;
  Start_position.z = 0.0;

  geometry_msgs::Quaternion Start_orientation;
  Start_orientation.x = 0.0;
  Start_orientation.y = 0.0;
  Start_orientation.z = 0.0;
  Start_orientation.w = 1.0;

  geometry_msgs::Pose Start_pose;
  Start_pose.position = Start_position;
  Start_pose.orientation = Start_orientation;

  gazebo_msgs::ModelState Start_modelstate;
  Start_modelstate.model_name = (std::string)"mobile_base";
  Start_modelstate.pose = Start_pose;

  SetModelState_srv.request.model_state = Start_modelstate;
  return SetRobotPositionClient.call(SetModelState_srv);
}


bool RL::TaskEnvIO::setActorTarget(const float x_, const float y_){
  actor_services::SetPose SetActorTarget_srv;
  SetActorTarget_srv.request.set_flag = true;
  SetActorTarget_srv.request.new_x = x_;
  SetActorTarget_srv.request.new_y = y_;
  return SetActorTargetClient.call(SetActorTarget_srv);
}

///////////////////////
double RL::TaskEnvIO::getRobotYaw(geometry_msgs::Quaternion &state_quat_) const {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(state_quat_, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

///////////////////////
void RL::TaskEnvIO::getRobotState(){

  std::unique_lock<std::mutex> state_2_lock(topic_mutex);
  gazebo_msgs::ModelStates newStates = state_2->StateVector.back();
  state_2_lock.unlock();
  std::vector<std::string> names = newStates.name;
  auto idx_ = std::find(names.begin(), names.end(),"mobile_base")-names.begin();
  assert(idx_ < names.size());
  geometry_msgs::Pose pose_ = newStates.pose.at(idx_);
  
  double yaw_= getRobotYaw(pose_.orientation);

  //relative angle normalize to -1 to 1
  float angle_in = (atan2(target_pose_.y-pose_.position.y,
      target_pose_.x-pose_.position.x) - yaw_)/M_PI;
  robot_state_.at(0) = (std::abs(angle_in)>1? -2*std::copysign(1, angle_in)+angle_in:angle_in);
  assert(std::abs(robot_state_.at(0))<1);
  //relative distance (TODO normalize)
  robot_state_.at(1) = sqrt(pow((target_pose_.x-pose_.position.x), 2) +
      pow((target_pose_.y-pose_.position.y), 2));



  //ROS_ERROR("=================================");
  //ROS_ERROR("target_pose x: %f", target_pose_.x);
  //ROS_ERROR("target_pose y: %f", target_pose_.y);
}


