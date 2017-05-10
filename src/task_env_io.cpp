/*************************************************************************
  > File Name: gazebo_env_io.cpp
  > Author: TAI Lei
  > Mail: lei.tai@my.cityu.edu.hk
  > Created Time: Mo 08 Mai 2017 16:54:01 CEST
 ************************************************************************/

#include <iostream>
#include <assert.h>
#include <algorithm>
#include "gazebo_env_io.h"
#include "task_env_io.h"

/// statecallback function 
template<typename topicType>
void RL::GetNewTopic<topicType>::StateCallback(const topicType& msg_){
  StateVector.push_back(msg_);
  if (StateVector.size() > 2) 
    StateVector.erase(StateVector.begin());
}

template<typename topicType>
RL::GetNewTopic<topicType>::GetNewTopic(ros::NodeHandlePtr rosNode_,
   const std::string topic_name){
 StateSub = rosNode_->subscribe(topic_name, 1, &GetNewTopic::StateCallback, this);   
}

RL::TaskEnvIO::TaskEnvIO(
    const std::string service_name,
    const std::string node_name,
    const float sleeping_time):
  GazeboEnvIO(node_name), 
  state_1(new RL::GetNewTopic<RL::STATE_1_TYPE>(this->rosNode, "/camera/depth/image_raw")),
  state_2(new RL::GetNewTopic<RL::STATE_2_TYPE>(this->rosNode, "/gazebo/ModelStates")),
  laser_scan(new RL::GetNewTopic<sensor_msgs::LaserScanConstPtr>(this->rosNode, "/scan")),
  sleeping_time_(sleeping_time){ 
    ActionPub = this->rosNode->advertise<RL::ACTION_TYPE>("/mobile_base/command/velolcity", 1);
    PytorchService = this->rosNode->advertiseService(service_name, &TaskEnvIO::ServiceCallback, this);
  }

bool RL::TaskEnvIO::ServiceCallback(
    gym_style_gazebo::PytorchRL::Request &req,
    gym_style_gazebo::PytorchRL::Response &res){
  ActionPub.publish(req.action);
  ros::Duration(sleeping_time_).sleep();
  res.state_1 = *(state_1->StateVector.back());
  res.state_2 = state_2->StateVector.back();
  res.terminal = terminalCheck();
  res.reward = rewardCalculate();
  return true;
}

float RL::TaskEnvIO::rewardCalculate() const{
  
  return 0;
}

bool RL::TaskEnvIO::terminalCheck() const{
   
  return true;
}

bool RL::TaskEnvIO::collision_check(){
  std::vector<float> range_array = laser_scan->StateVector.back()->ranges;
  float collision_distance= 0.0;
  rosNode->getParam("COLLISION_TH",collision_distance);
  assert(collision_distance!=0.0);
  
  float min_scan = *std::min_element(std::begin(range_array), std::end(range_array));
  return  (min_scan < collision_distance)? true : false;

}

bool RL::TaskEnvIO::target_check(){
  // Check the distance to target
  // Check if we arrive at the target
  
  return false;
}

bool RL::TaskEnvIO::reset() const{
  // Set a new target for the robot 
  // Set a new position for the robot
  // Set random position for pedes
  
  return true;
}
