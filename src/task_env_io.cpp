/*************************************************************************
  > File Name: gazebo_env_io.cpp
  > Author: TAI Lei
  > Mail: lei.tai@my.cityu.edu.hk
  > Created Time: Mo 08 Mai 2017 16:54:01 CEST
 ************************************************************************/

#include <iostream>
#include <algorithm>
#include "task_env_io.h"

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
  state_1(new RL::GetNewTopic<sensor_msgs::ImageConstPtr>(this->rosNode, "/camera/depth/image_raw")),
  state_2(new RL::GetNewTopic<gazebo_msgs::ModelStates>(this->rosNode, "/gazebo/ModelStates")),
  laser_scan(new RL::GetNewTopic<sensor_msgs::LaserScanConstPtr>(this->rosNode, "/scan")),
  sleeping_time_(sleeping_time){ 
    ActionPub = this->rosNode->advertise<geometry_msgs::Twist>("/mobile_base/command/velolcity", 1);
    PytorchService = this->rosNode->advertiseService(service_name, &TaskEnvIO::ServiceCallback, this);
  }


bool RL::TaskEnvIO::ServiceCallback(
    gym_style_gazebo::PytorchRL::Request &req,
    gym_style_gazebo::PytorchRL::Response &res){
  ActionPub.publish(req.action);
  ros::Duration(sleeping_time_).sleep();
  res.state_1 = *(state_1->StateVector.back());
  res.state_2 = state_2->StateVector.back();
  res.reward = rewardCalculate();
  res.terminal = terminalCheck();
  return true;
}



float RL::TaskEnvIO::rewardCalculate() const{
  return 0;
}


bool RL::TaskEnvIO::terminalCheck() const{
  return true;
}


bool RL::TaskEnvIO::reset() const{
  return true;
}
