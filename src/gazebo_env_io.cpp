/*************************************************************************
  > File Name: gazebo_env_io.cpp
  > Author: TAI Lei
  > Mail: lei.tai@my.cityu.edu.hk
  > Created Time: Mo 08 Mai 2017 16:54:01 CEST
 ************************************************************************/

#include <iostream>
#include <algorithm>
#include <geometry_msgs/Twist.h>
#include "gazebo_env_io.h"
using namespace std;


template <class StateType, class ActionType>
RL::GazeboEnvIO<StateType, ActionType>::GazeboEnvIO(
    const std::string state_topic_name,
    const std::string action_topic_name,
    const std::string laser_topic_name,
    const std::string service_name){
  StateSub = n_.subscribe(state_topic_name, 1, &GazeboEnvIO::StateCallback, this);
  LaserSub = n_.subscribe(laser_topic_name, 1, &GazeboEnvIO::LaserCallback, this);

  PytorchService = n_.advertiseService(service_name, &GazeboEnvIO::ServiceCallback,this);
  ActionPub = n_.advertise<geometry_msgs::Twist>(action_topic_name, 1);
  
  n_.getParam("COLLISION_TH",collision_th_);
}


template <class StateType, class ActionType>
void RL::GazeboEnvIO<StateType, ActionType>::StateCallback(const StateType& msg){
  StatePtrVector.push_back(msg);
  if (StatePtrVector.size() > 2)
  {
    StatePtrVector.erase(StatePtrVector.begin());
  }
}

template <class StateType>
void GazeboEnvIO<StateType>::LaserCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  LaserPtrVector.push_back(scan);
  if (LaserPtrVector.size()>2)
  {
    LaserPtrVector.erase(LaserPtrVector.begin());
  }		
}

template <class StateType>
bool GazeboEnvIO<StateType>::CollisionCheck(){
	std::vector<float>  range_array = LaserPtrVector.back()->ranges;
  return collision_th_ > std::min_element(range_array.begin(),range_array.end()) ? true : false;
}

template <class StateType>
bool GazeboEnvIO<StateType>::ServiceCallback(
    gym_style_gazebo::PytorchRL::Request &req,
    gym_style_gazebo::PytorchRL::Response &res
    ){
//TODO finish this service to get action float
// and return image processing result
// reward
// terminal symbol
// pede + tf/set_position/
}

