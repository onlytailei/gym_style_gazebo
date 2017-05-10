/*************************************************************************
  > File Name: rl_util.cpp
  > Author: TAI Lei
  > Mail: lei.tai@my.cityu.edu.hk
  > Created Time: Di 09 Mai 2017 18:44:43 CEST
 ************************************************************************/

#include <iostream>
#include <memory>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include "gazebo_env_io.h"

namespace RL {

  template<typename topicType>
  class GetNewTopic{

      public: std::vector<topicType> StateVector;
      private: ros::Subscriber StateSub;
               void StateCallback(const topicType& msg_);

      public: 
               GetNewTopic(ros::NodeHandlePtr rosNode_,
                   const std::string topic_name);
  };

  class TaskEnvIO : public GazeboEnvIO{

    private:
      ros::Publisher ActionPub;
      std::shared_ptr< RL::GetNewTopic<sensor_msgs::ImageConstPtr> >  state_1;
      std::shared_ptr< RL::GetNewTopic<gazebo_msgs::ModelStates> > state_2;
      std::shared_ptr<RL::GetNewTopic<sensor_msgs::LaserScanConstPtr>> laser_scan;

      const float sleeping_time_;
      
      virtual bool ServiceCallback(gym_style_gazebo::PytorchRL::Request&,
            gym_style_gazebo::PytorchRL::Response&);
      
      virtual float rewardCalculate() const;
      virtual bool terminalCheck() const;
      virtual bool reset() const;

    public: 
      TaskEnvIO(
          const std::string service_name="pytorch_io_service",
          const std::string node_name="gazebo_env_io",
          const float sleeping_time=0);
  };
}

