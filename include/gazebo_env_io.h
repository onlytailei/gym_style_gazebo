/*************************************************************************
	> File Name: rl_utils.h
	> Author: TAI Lei
	> Mail: lei.tai@my.cityu.edu.hk
	> Created Time: Di 09 Mai 2017 18:39:58 CEST
 ************************************************************************/

#ifndef _GAZEBO_ENV_IO_H
#define _GAZEBO_ENV_IO_H

#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <gym_style_gazebo/PytorchRL.h>

// TODO build a base class
// with two functions
// rewardCal
// terminalCheck

// TODO build a inherit class to get that

namespace RL{
  
  class GazeboEnvIO{
       
    public:
      ros::NodeHandlePtr rosNode;
      ros::ServiceServer PytorchService;
      
      GazeboEnvIO( 
          const std::string node_name="gazebo_env_io");
      
      virtual bool ServiceCallback(gym_style_gazebo::PytorchRL::Request&,
            gym_style_gazebo::PytorchRL::Response&);

      virtual float rewardCalculate() const =0; 
      
      virtual bool terminalCheck() const =0;
      
      virtual bool reset() const =0;
  };
}

RL::GazeboEnvIO::GazeboEnvIO( 
          const std::string node_name):
  rosNode(new ros::NodeHandle(node_name)){
}

#endif
