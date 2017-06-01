/*************************************************************************
  > File Name: rl_util.cpp
  > Author: TAI Lei
  > Mail: lei.tai@my.cityu.edu.hk
  > Created Time: Di 09 Mai 2017 18:44:43 CEST
 ************************************************************************/

#ifndef _TASK_ENV_IO_H
#define _TASK_ENV_IO_H

#include <iostream>
#include <memory>
#include <vector>
#include <random>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "gazebo_env_io.h"

namespace RL {
  // Template class for subscriber 
  template<typename topicType>
    class GetNewTopic{

      public: std::vector<topicType> StateVector;
      private: ros::Subscriber StateSub;
               void StateCallback(const topicType& );

      public: 
               GetNewTopic(ros::NodeHandlePtr, const std::string);
    };

  // typedef
  using STATE_1_TYPE = sensor_msgs::ImageConstPtr;
  using STATE_2_TYPE = gazebo_msgs::ModelStates;
  using ACTION_TYPE = geometry_msgs::Twist;
  // angle, distance, ang_vel, lin_vel
  using ROBOT_STATE = std::array<float, 4>; 
  
  // target pose structure
  struct TargetPose {
    float x;
    float y;
  };

  // Param Class
  class ParamLoad{
    private:
      const ros::NodeHandlePtr rosNodeConstPtr;
    public:
      ParamLoad(ros::NodeHandlePtr);
      float collision_th;
      float target_th;
      float terminalReward;
      float failReward;
      float distance_coef;
      float time_discount;
      float max_lin_vel;
      float max_ang_vel;
      bool enable_collision_terminal;
      bool enable_continuous_control;
      bool enable_ped;
      float origin_x;
      float origin_y;
      float target_start;
      float target_end;
  };


  // Main class inherit from gazeboenvio
  class TaskEnvIO : public RL::GazeboEnvIO{
    private:
      ros::Publisher ActionPub;
      std::shared_ptr<RL::GetNewTopic<RL::STATE_1_TYPE>> state_1;
      std::shared_ptr<RL::GetNewTopic<RL::STATE_2_TYPE>> state_2;
      std::shared_ptr<RL::GetNewTopic<sensor_msgs::LaserScanConstPtr>> laser_scan;
      ros::ServiceClient SetRobotPositionClient;
      ros::ServiceClient SetActorTargetClient;
      

      const std::shared_ptr<ParamLoad> paramlist;

      bool setRobotPosition();
      bool setActorTarget(const float, const float);
      bool collision_check();
      bool target_check();
      void getRobotState();
      double getRobotYaw(geometry_msgs::Quaternion &) const;
      void actionPub(geometry_msgs::Twist);

      float previous_distance;
      bool terminal_flag;
      
      RL::TargetPose target_pose_;
      RL::ROBOT_STATE robot_state_;
      cv_bridge::CvImagePtr cv_ptr;
      
      std::mt19937 random_engine;
      std::uniform_real_distribution<> dis;
      std::uniform_real_distribution<> target_gen;

      
      const float sleeping_time_;

      virtual bool ServiceCallback(
          gym_style_gazebo::PytorchRL::Request&,
          gym_style_gazebo::PytorchRL::Response&);

      virtual float rewardCalculate();
      virtual bool terminalCheck();
      virtual bool reset();

    public: 
      TaskEnvIO(
          const std::string service_name="pytorch_io_service",
          const std::string node_name="gazebo_env_io",
          const float sleeping_time=0.2);
  };
}

#endif
