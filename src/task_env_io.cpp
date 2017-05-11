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
#include "gazebo_env_io.h"
#include "task_env_io.h"

/// statecallback function 
template<typename topicType>
void RL::GetNewTopic<topicType>::StateCallback(const topicType& msg_){
  StateVector.push_back(msg_);
  if (StateVector.size() > 5) 
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
    
    rosNode->getParam("COLLISION_TH",collision_th);
    assert(collision_th!=0.0);
    ActionPub = this->rosNode->advertise<RL::ACTION_TYPE>("/mobile_base/command/velolcity", 1);
    PytorchService = this->rosNode->advertiseService(service_name, &TaskEnvIO::ServiceCallback, this);
  }

bool RL::TaskEnvIO::ServiceCallback(
    gym_style_gazebo::PytorchRL::Request &req,
    gym_style_gazebo::PytorchRL::Response &res){
  
  ActionPub.publish(req.action);
  res.terminal = terminalCheck();
  res.reward = rewardCalculate();
  //ros::Duration(sleeping_time_).sleep();
  //res.state_1 = *((state_1->StateVector).back());
  //res.state_2 = (state_2->StateVector.back());
  return true;
}

float RL::TaskEnvIO::rewardCalculate(){
  collision_check();
  target_check();
  getRobotState();
  return 0;
}

bool RL::TaskEnvIO::terminalCheck(){

  return true;
}

bool RL::TaskEnvIO::collision_check(){
  std::vector<float> range_array = laser_scan->StateVector.back()->ranges;
  //float collision_distance= 0.0;

  float min_scan = *std::min_element(std::begin(range_array), std::end(range_array));
  return  (min_scan < collision_th)? true : false;

}

bool RL::TaskEnvIO::target_check(){
  // Check the distance and angle to target
  // Check if we arrive at the target

  tf::StampedTransform transform_;
  try{
    tf_listener.lookupTransform("base_link",
        "target_pose",
        ros::Time(0),
        transform_); 
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep(); 
  }
  
  float angle_ = atan2(transform_.getOrigin().y(),
      transform_.getOrigin().x());
  
  float distance_ = sqrt(pow(transform_.getOrigin().x(), 2) + 
      pow(transform_.getOrigin().y(), 2));
  
  std::cout<<"angle: "<<angle_<<std::endl;
  std::cout<<"distance: "<<distance_<<std::endl;

  return true;
}

bool RL::TaskEnvIO::reset() {
  // Set a new target for the robot 
  // Set a new position for the robot
  // Set random position for pedes

  return true;
}



float RL::TaskEnvIO::getRobotState(){
  // get robot pose
  //gazebo_msgs::ModelStates newStates = state_2->StateVector.back();
  //std::vector<std::string> names = newStates.name; 
  //auto idx_ = std::find(names.begin(), names.end(),"mobile_base")-names.begin(); 
  //assert(idx_ < names.size());
  //geometry_msgs::Twist robot_twisit = newStates.twist.at(idx_);

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
  
  float angle_vel_ = twsit_.angular.z;
  
  float lin_vel_ = sqrt(pow(twsit_.linear.x, 2) + 
      pow(twsit_.linear.y, 2));
  
  std::cout<<"angular vel:  "<<angle_vel_<<std::endl;
  std::cout<<"linear vel: "<<lin_vel_<<std::endl;

  return true;
  

  //float x_ = robot_pose.position.x;
  //float y_ = robot_pose.position.y;

  //tf::Quaternion quat;
  //tf::quaternionMsgToTF(robot_pose.orientation, quat);

  //float rotation_angle = atan2(start_y-y, start_x-x) - yaw;
  //float norm_rotation_angle = (int(rotation_angle * 10000) % 31416 )*1.0/31416;
}
