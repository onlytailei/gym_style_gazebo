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
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
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

  // Main class inherit from gazeboenvio
  class TaskEnvIO : public RL::GazeboEnvIO{
    private:
      ros::Publisher ActionPub;
      std::shared_ptr<RL::GetNewTopic<RL::STATE_1_TYPE>> state_1;
      std::shared_ptr<RL::GetNewTopic<RL::STATE_2_TYPE>> state_2;
      std::shared_ptr<RL::GetNewTopic<sensor_msgs::LaserScanConstPtr>> laser_scan;
      
      bool collision_check();
      bool target_check();
      float  getRobotState();
      float  getRobotState2();
      
      float collision_th;
      tf::TransformListener tf_listener;
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
          const float sleeping_time=0);
  };
}

#endif
