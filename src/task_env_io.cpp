/*************************************************************************
  > File Name: gazebo_env_io.cpp
  > Author: TAI Lei
  > Mail: lei.tai@my.cityu.edu.hk
  > Created Time: Mo 08 Mai 2017 16:54:01 CEST
 ************************************************************************/

#include <iostream>
#include <assert.h>
#include <algorithm>
#include <math.h>
#include <time.h>
#include <ctime>
#include "gazebo_env_io.h"
#include "task_env_io.h"

/// statecallback function 
template<typename topicType>
void RL::GetNewTopic<topicType>::StateCallback(const topicType& msg_){
  StateVector.push_back(msg_);
  if (StateVector.size() > 5) 
    StateVector.erase(StateVector.begin());
}

////////////////////////////
template<typename topicType>
RL::GetNewTopic<topicType>::GetNewTopic(ros::NodeHandlePtr rosNode_,
    const std::string topic_name){
  StateSub = rosNode_->subscribe(topic_name, 1, &GetNewTopic::StateCallback, this);   
}

////////////////////////////
RL::TaskEnvIO::TaskEnvIO(
    const std::string service_name,
    const std::string node_name,
    const float sleeping_time):
  GazeboEnvIO(node_name), 
  state_1(new RL::GetNewTopic<RL::STATE_1_TYPE>(this->rosNode, "/camera/depth/image_raw")),
  state_2(new RL::GetNewTopic<RL::STATE_2_TYPE>(this->rosNode, "/gazebo/model_states")),
  laser_scan(new RL::GetNewTopic<sensor_msgs::LaserScanConstPtr>(this->rosNode, "/scan")),
  target_pose_{0,0},
  robot_state_{0,0,0,0},
  sleeping_time_(sleeping_time){

    rosNode->getParam("COLLISION_TH",collision_th);
    rosNode->getParam("DISTANCE_COEF",distance_coef);
    rosNode->getParam("FAIL_REWARD",failReward);
    rosNode->getParam("TERMINAL_REWARD",terminalReward);
    rosNode->getParam("TARGET_TH",target_th);
    rosNode->getParam("TARGET_X",target_pose_.x);
    rosNode->getParam("TARGET_Y",target_pose_.y);
    rosNode->getParam("TIME_DISCOUNT",time_discount);

    assert(collision_th!=0.0);

    ActionPub = this->rosNode->advertise<RL::ACTION_TYPE>("/mobile_base/command/velolcity", 1);
    PytorchService = this->rosNode->advertiseService(service_name, &TaskEnvIO::ServiceCallback, this);
  }

///////////////////////
bool RL::TaskEnvIO::ServiceCallback(
    gym_style_gazebo::PytorchRL::Request &req,
    gym_style_gazebo::PytorchRL::Response &res){

  if (req.reset){
    while (terminal_flag)
      reset();
  }
  
  ActionPub.publish(req.action);
  //ros::Duration(sleeping_time_).sleep();
  getRobotState(); //update robot state
  res.reward = rewardCalculate();
  res.terminal = terminal_flag;
  res.state_1 = *((state_1->StateVector).back());
  res.state_2 = (state_2->StateVector.back());
  return true;
}

///////////////////////
float RL::TaskEnvIO::rewardCalculate(){
  if (collision_check()){
    terminal_flag = true;
    return failReward;}
  else if (terminalCheck()){
    terminal_flag = true;
    return terminalReward;}
  else {
    terminal_flag = false;
    return distance_coef * (previous_distance - robot_state_.distance)-time_discount;}
  previous_distance = robot_state_.distance;
  return 0;
}

///////////////////////
bool RL::TaskEnvIO::terminalCheck(){
  return terminal_flag;
}

///////////////////////
bool RL::TaskEnvIO::collision_check(){
  std::vector<float> range_array = laser_scan->StateVector.back()->ranges;
  float min_scan = *std::min_element(std::begin(range_array), std::end(range_array));
  return  (min_scan < collision_th)? true : false;
}

/// use tf to call the robot position DEPRECATED too slow
bool RL::TaskEnvIO::target_check(){
  return (robot_state_.distance <target_th)? true : false;
}

///////////////////////
bool RL::TaskEnvIO::reset() {
  // Set a new target for the robot 
  // Set a new position for the robot
  // Set random position for pedes

  return true;
}


///////////////////////
void RL::TaskEnvIO::getRobotState(){
  gazebo_msgs::ModelStates newStates = state_2->StateVector.back();
  std::vector<std::string> names = newStates.name; 
  auto idx_ = std::find(names.begin(), names.end(),"mobile_base")-names.begin(); 
  assert(idx_ < names.size());
  geometry_msgs::Twist twist_ = newStates.twist.at(idx_);
  geometry_msgs::Pose pose_ = newStates.pose.at(idx_);

  //Robot State Update
  robot_state_.angle = atan2(target_pose_.x-pose_.position.x, 
      target_pose_.y-pose_.position.y);

  robot_state_.distance = sqrt(pow((target_pose_.x-pose_.position.x), 2) + 
      pow((target_pose_.y-pose_.position.y), 2));

  robot_state_.ang_vel = twist_.angular.z;

  robot_state_.lin_vel = sqrt(pow(twist_.linear.x, 2) + 
      pow(twist_.linear.y, 2));
}


/// use tf to call the robot pose and speed DEPRECATED too slow
float RL::TaskEnvIO::getRobotStateTF(){
  std::clock_t c_start = std::clock(); 
  tf::StampedTransform transform_;
  try{
    tf_listener.lookupTransform("base_link",
        "target_pose",
        ros::Time(0),
        transform_); 
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  std::clock_t c_end = std::clock();
  std::cout<< "call target tf time: "<<c_end-c_start<<std::endl;

  float angle_ = atan2(transform_.getOrigin().y(),
      transform_.getOrigin().x());

  float distance_ = sqrt(pow(transform_.getOrigin().x(), 2) + 
      pow(transform_.getOrigin().y(), 2));

  std::cout<<"angle: "<<angle_<<std::endl;
  std::cout<<"distance: "<<distance_<<std::endl;


  std::clock_t c_start_2 = std::clock(); 
  geometry_msgs::Twist twsit_;
  try{
    tf_listener.lookupTwist("default_world",
        "base_link",
        ros::Time(0),
        ros::Duration(0.01),
        twsit_ ); 
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep(); 
  }

  std::clock_t c_end_2 = std::clock();
  std::cout<< "call twist tf time: "<<c_end_2-c_start_2<<std::endl;

  float angle_vel_ = twsit_.angular.z;

  float lin_vel_ = sqrt(pow(twsit_.linear.x, 2) + 
      pow(twsit_.linear.y, 2));

  std::cout<<"angular vel:  "<<angle_vel_<<std::endl;
  std::cout<<"linear vel: "<<lin_vel_<<std::endl;

  return 0;
}




