/*************************************************************************
  > File Name: gazebo_env_io.h
  > Author: TAI Lei
  > Mail: lei.tai@my.cityu.edu.hk
  > Created Time: Mo 08 Mai 2017 16:33:39 CEST
 ************************************************************************/

#ifndef _GAZEBO_ENV_IO_H
#define _GAZEBO_ENV_IO_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <gym_style_gazebo/PytorchRL.h>
#include <iostream>
#include <vector>
#include <string>
#include <iterator>

namespace RL{
  template<class StateType, class ActionType>
    class GazeboEnvIO{

      private:
        ros::NodeHandle n_;
        ros::Subscriber LaserSub; 
        ros::Subscriber StateSub;
        ros::Publisher ActionPub;
        ros::ServiceServer PytorchService;

        std::vector<StateType> StatePtrVector;
        std::vector<sensor_msgs::LaserScan::ConstPtr> LaserPtrVector;

        void StateCallback(const StateType& );
        void LaserCallback(const sensor_msgs::LaserScan::ConstPtr&);

        bool ServiceCallback(gym_style_gazebo::PytorchRL::Request&,
            gym_style_gazebo::PytorchRL::Response&);

        float collision_th_;
        bool CollisionCheck();
      
      public:
        GazeboEnvIO(
            const std::string state_topic_name="/camera/depth/image_raw",
            const std::string action_topic_name="/mobile",
            const std::string laser_topic_name="/scan",
            const std::string service_name="pytorch_service",
            const std::string node_name="node name",
            const float sleeping_time=0.01);
    };
}
#endif
