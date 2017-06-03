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
    assert(rosNodeConstPtr->getParam("/COLLISION_REWARD",collisionReward));
    assert(rosNodeConstPtr->getParam("/TERMINAL_REWARD",terminalReward));
    assert(rosNodeConstPtr->getParam("/TARGET_TH",target_th));
    assert(rosNodeConstPtr->getParam("/ROBOT_X_START",robot_x_start));
    assert(rosNodeConstPtr->getParam("/ROBOT_X_END",robot_x_end));
    assert(rosNodeConstPtr->getParam("/ROBOT_Y_START",robot_y_start));
    assert(rosNodeConstPtr->getParam("/ROBOT_Y_END",robot_y_end));
    assert(rosNodeConstPtr->getParam("/TIME_DISCOUNT",time_discount));
    assert(rosNodeConstPtr->getParam("/MAX_LINEAR_VAL",max_lin_vel));
    assert(rosNodeConstPtr->getParam("/MAX_ANGULAR_VAL",max_ang_vel));
    assert(rosNodeConstPtr->getParam("/ENABLE_COLLISIOM_TERMINAL",enable_collision_terminal));
    assert(rosNodeConstPtr->getParam("/ENABLE_CONTINUOUS_CONTROL",enable_continuous_control));
    assert(rosNodeConstPtr->getParam("/ENABLE_PED",enable_ped));
}

////////////////////////////
RL::TaskEnvIO::TaskEnvIO(
    const std::string service_name,
    const std::string node_name,
    const float sleeping_time):
  GazeboEnvIO(node_name),
  state_1(new RL::GetNewTopic<RL::STATE_1_TYPE>(this->rosNode_pr, "/camera/rgb/image_raw")),
  state_2(new RL::GetNewTopic<RL::STATE_2_TYPE>(this->rosNode_pr, "/gazebo/model_states")),
  laser_scan(new RL::GetNewTopic<sensor_msgs::LaserScanConstPtr>(this->rosNode_pr, "/fakescan")),
  paramlist(new RL::ParamLoad(this->rosNode_pr)),
  robot_state_{{0,0}}, //double brace for std::array
  random_engine(0),
  dis(-1,1),  // noise generator
  target_gen(0,1),  //noise generator
  sleeping_time_(sleeping_time){

    assert(rosNode_pr->getParam("/TARGET_X",target_pose.x));
    assert(rosNode_pr->getParam("/TARGET_Y",target_pose.y));
    ActionPub = this->rosNode_pr->advertise<RL::ACTION_TYPE>("/mobile_base/commands/velocity", 1);
    PytorchService = this->rosNode_pr->advertiseService(service_name, &TaskEnvIO::ServiceCallback, this);
    SetModelPositionClient = this->rosNode_pr->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); 
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
    while (terminal_flag || CollisionCheck()){
      ROS_ERROR("Reset loop");
      this->reset();
    }
  }
  actionPub(req.action); 
  res.reward = rewardCalculate();
  res.terminal = terminal_flag;
  
  //ROS_ERROR("=================================");
  ROS_ERROR("Reward: %f", res.reward);
  
  std::unique_lock<std::mutex> state_1_lock(topic_mutex);
  //cv_ptr = cv_bridge::toCvCopy(state_1->StateVector.back(), state_1->StateVector.back()->encoding);
  res.state_1 = *(state_1->StateVector.back());
  state_1_lock.unlock();
  
 
  //{
  //res.state_1.layout.dim.push_back(std_msgs::MultiArrayDimension());
  //res.state_1.layout.dim.push_back(std_msgs::MultiArrayDimension());
  //res.state_1.layout.dim.push_back(std_msgs::MultiArrayDimension());
  //res.state_1.layout.dim[0].size = cv_ptr->image.rows;
  //res.state_1.layout.dim[0].stride = cv_ptr->image.cols*cv_ptr->image.rows*cv_ptr->image.channels();
  //res.state_1.layout.dim[0].label = "height";
  //res.state_1.layout.dim[1].size = cv_ptr->image.cols;
  //res.state_1.layout.dim[1].stride = cv_ptr->image.cols*cv_ptr->image.channels();
  //res.state_1.layout.dim[1].label = "width";
  //res.state_1.layout.dim[2].size = cv_ptr->image.channels();
  //res.state_1.layout.dim[2].stride = cv_ptr->image.channels();
  //res.state_1.layout.dim[2].label = "channel";
  //res.state_1.data.clear();
  //// convert CV_16UC3 to CV_32FC3 so that we do not need to change the service head files. 
  //cv::Mat _imgf;
  //cv_ptr->image.convertTo(_imgf, CV_32FC3);
  //const std::vector<float> output_img((float*)_imgf.data, (float*)_imgf.data + cv_ptr->image.cols * cv_ptr->image.rows*cv_ptr->image.channels());
  //res.state_1.data.reserve(cv_ptr->image.cols*cv_ptr->image.rows*cv_ptr->image.channels());
  //res.state_1.data.insert(res.state_1.data.end(), output_img.begin(), output_img.end());
  //}
  
  //Build second state  NOTE: right now useless
  {
  res.state_2.layout.dim.push_back(std_msgs::MultiArrayDimension());
  res.state_2.layout.dim[0].size = robot_state_.size();
  res.state_2.layout.dim[0].stride = robot_state_.size();
  res.state_2.layout.dim[0].label = "roobot state";
  res.state_2.data.clear();
  res.state_2.data.reserve(robot_state_.size());
  res.state_2.data.insert(res.state_2.data.end(), robot_state_.begin(),robot_state_.end());
  }
  return true;
}

void RL::TaskEnvIO::actionPub(geometry_msgs::Twist action_out){
    action_out.angular.z = action_out.angular.z*paramlist->max_ang_vel;  
    action_out.linear.x = action_out.linear.x*paramlist->max_lin_vel;  
    ActionPub.publish(action_out);
}

///////////////////////
float RL::TaskEnvIO::rewardCalculate(){
  if (TargetCheck()){
    terminal_flag = true;
    return paramlist->terminalReward;}
  else if (CollisionCheck()){
    terminal_flag = paramlist->enable_collision_terminal;
    return paramlist->collisionReward;}
  else {
    terminal_flag = false;
    return paramlist->time_discount;}
  return 0;
}

///////////////////////
bool RL::TaskEnvIO::terminalCheck(){
  return terminal_flag;
}

///////////////////////
bool RL::TaskEnvIO::CollisionCheck() const{
  std::unique_lock<std::mutex> laser_scan_lock(topic_mutex);
  assert(laser_scan->StateVector.size()>0);
  std::vector<float> range_array = laser_scan->StateVector.back()->ranges;
  laser_scan_lock.unlock();
  range_array.erase(std::remove_if(range_array.begin(), 
        range_array.end(), 
        [](float x){return !std::isfinite(x);}), 
      range_array.end());
  float min_scan = *std::min_element(std::begin(range_array), std::end(range_array));
  return  (range_array.size()==0)? true : (min_scan< (paramlist->collision_th)?true:false);
}


///////////////////////
bool RL::TaskEnvIO::reset() {
  
  // Set a new position for one ped
  if (paramlist->enable_ped){
    float target_x = target_gen(random_engine)*(1.7-0.5)+0.5; 
    float target_y = target_gen(random_engine)*(1.7-0.5)+0.5;
    setActorTarget(std::copysign(target_x, dis(random_engine)),
                std::copysign(target_y, dis(random_engine)));
  }
  // Set a new position for the robot and target, if change target position, should also chagne the br of target
  const float _x = target_gen(random_engine)*(paramlist->robot_x_end-paramlist->robot_x_start)+paramlist->robot_x_start; 
  const float _y = target_gen(random_engine)*(paramlist->robot_y_end-paramlist->robot_y_start)+paramlist->robot_y_start;
  const geometry_msgs::Quaternion _q_robot = tf::createQuaternionMsgFromYaw(dis(random_engine)*3.14);
  const geometry_msgs::Quaternion _q_target = tf::createQuaternionMsgFromYaw(dis(random_engine)*3.14);
  setModelPosition(_x,_y,_q_robot);
  setModelPosition(target_pose.x,target_pose.y,_q_target, RL::TARGET_NAME);
  rewardCalculate(); //check if it is terminal
  return true;
}

bool RL::TaskEnvIO::setModelPosition(const float x, const float y, const geometry_msgs::Quaternion q, const std::string model_name_ ){
  
  gazebo_msgs::SetModelState SetModelState_srv;
  geometry_msgs::Point Start_position;
  Start_position.x = x;
  Start_position.y = y;
  Start_position.z = 0.0;

  //geometry_msgs::Quaternion Start_orientation;
  //Start_orientation.x = 0.0;
  //Start_orientation.y = 0.0;
  //Start_orientation.z = q_z;
  //Start_orientation.w = q_w;

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

///////////////////////
//double RL::TaskEnvIO::getRobotYaw(geometry_msgs::Quaternion &state_quat_) const {
    //tf::Quaternion quat;
    //tf::quaternionMsgToTF(state_quat_, quat);
    //double roll, pitch, yaw;
    //tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    //return yaw;
//}

///////////////////////
bool RL::TaskEnvIO::TargetCheck(){

  std::unique_lock<std::mutex> state_2_lock(topic_mutex);
  gazebo_msgs::ModelStates newStates = state_2->StateVector.back();
  state_2_lock.unlock();
  std::vector<std::string> names = newStates.name;
  auto idx_ = std::find(names.begin(), names.end(),RL::ROBOT_NAME)-names.begin();
  auto target_idx_ = std::find(names.begin(), names.end(),RL::TARGET_NAME)-names.begin();
  assert(idx_ < names.size() && target_idx_ < names.size());
  geometry_msgs::Pose pose_ = newStates.pose.at(idx_);
  geometry_msgs::Pose barrel_pose_ = newStates.pose.at(target_idx_);
  
  // TODO: about robot speed useless now
  //geometry_msgs::Twist robot_twist = newStates.twist.at(idx_);
  //robot_state_.at(0) = robot_twist.linear.x;
  //robot_state_.at(1) = robot_twist.linear.y;
  //robot_state_.at(2) = robot_twist.linear.z;
  //robot_state_.at(3) = robot_twist.angular.x;
  //robot_state_.at(4) = robot_twist.angular.y;
  //robot_state_.at(5) = robot_twist.angular.z;
  
  float target_distance = sqrt(pow((barrel_pose_.position.x-pose_.position.x), 2) +
      pow((barrel_pose_.position.y-pose_.position.y), 2));
  return (target_distance <paramlist->target_th)? true : false;
}


