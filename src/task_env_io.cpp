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
#include <cmath>
#include <time.h>
#include <ctime>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h> //roslogging
#include "gazebo_env_io.h"
#include "task_env_io.h"
#include <typeinfo>

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
  robot_state_{{0,0,0,0}}, //double brace for std::array
  sleeping_time_(sleeping_time){

    assert(rosNode->getParam("/COLLISION_TH",collision_th));
    assert(rosNode->getParam("/DISTANCE_COEF",distance_coef));
    assert(rosNode->getParam("/FAIL_REWARD",failReward));
    assert(rosNode->getParam("/TERMINAL_REWARD",terminalReward));
    assert(rosNode->getParam("/TARGET_TH",target_th));
    assert(rosNode->getParam("/TARGET_X",target_pose_.x));
    assert(rosNode->getParam("/TARGET_Y",target_pose_.y));
    assert(rosNode->getParam("/TIME_DISCOUNT",time_discount));

    ActionPub = this->rosNode->advertise<RL::ACTION_TYPE>("/mobile_base/commands/velocity", 1);
    PytorchService = this->rosNode->advertiseService(service_name, &TaskEnvIO::ServiceCallback, this);
  }

///////////////////////
bool RL::TaskEnvIO::ServiceCallback(
    gym_style_gazebo::PytorchRL::Request &req,
    gym_style_gazebo::PytorchRL::Response &res){

  // TODO the terminal_flag is always false now
  /*
  if (req.reset){
    while (terminal_flag)
      reset();
  }
  */

  ActionPub.publish(req.action);
  //ros::Duration(sleeping_time_).sleep();
  getRobotState(); //update robot state
  res.reward = rewardCalculate();
  res.terminal = terminal_flag;
  cv_ptr = cv_bridge::toCvCopy(state_1->StateVector.back(), state_1->StateVector.back()->encoding);
  //Build first state
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

///////////////////////
float RL::TaskEnvIO::rewardCalculate(){
  // TODO check the collision check alwasy return true
  
  if (collision_check()){
    terminal_flag = true;
    return failReward;}
  if (terminalCheck()){
    terminal_flag = true;
    return terminalReward;}
  else {
    terminal_flag = false;
    return distance_coef * (previous_distance - robot_state_.at(1))-time_discount;}
  previous_distance = robot_state_.at(1);
  return 0;
}



///////////////////////
bool RL::TaskEnvIO::terminalCheck(){
  return terminal_flag;
}

///////////////////////
bool RL::TaskEnvIO::collision_check(){
  std::vector<float> range_array = laser_scan->StateVector.back()->ranges;
  range_array.erase(std::remove_if(range_array.begin(), 
        range_array.end(), 
        [](float x){return !std::isfinite(x);}), 
      range_array.end());
  float min_scan = *std::min_element(std::begin(range_array), std::end(range_array));
  //std::cout<< "min_scan: "<< min_scan  <<std::endl;
  
  ROS_ERROR("min_scan: %f, typeinfo: %s", min_scan, typeid(min_scan).name());
  return  (min_scan < collision_th)? true : false;
}

/// use tf to call the robot position DEPRECATED too slow
bool RL::TaskEnvIO::target_check(){
  return (robot_state_.at(1) <target_th)? true : false;
}

///////////////////////
bool RL::TaskEnvIO::reset() {
  // Set a new target for the robot
  // Set a new position for the robot
  // Set random position for pedes

  getRobotState(); //update robot state
  rewardCalculate(); //check if it is terminal
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
  robot_state_.at(0) = atan2(target_pose_.x-pose_.position.x,
      target_pose_.y-pose_.position.y);

  robot_state_.at(1) = sqrt(pow((target_pose_.x-pose_.position.x), 2) +
      pow((target_pose_.y-pose_.position.y), 2));

  robot_state_.at(2) = twist_.angular.z;

  robot_state_.at(3) = sqrt(pow(twist_.linear.x, 2) +
      pow(twist_.linear.y, 2));

  //std::cout<<"==================="<<std::endl;
  //std::cout<<"target_pose x: "<<target_pose_.x<<std::endl;
  //std::cout<<"target_pose y: "<<target_pose_.y<<std::endl;
  //std::cout<<"robot_pose x: "<<pose_.position.x<<std::endl;
  //std::cout<<"robot_pose y: "<<pose_.position.y<<std::endl;
  //std::cout<< "angle: "<<robot_state_.at(0) <<std::endl;
  //std::cout<< "distance: "<<robot_state_.at(1) <<std::endl;
  //std::cout<< "ang_vel: "<<robot_state_.at(2) <<std::endl;
  //std::cout<< "lin_vel: "<<robot_state_.at(3) <<std::endl;
  //std::cout<< "collision_th: "<<collision_th <<std::endl;
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
