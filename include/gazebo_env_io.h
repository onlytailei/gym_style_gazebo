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
#include <gym_style_gazebo/SocialForce.h>

namespace RL{
  
  class GazeboEnvIO{
       
    public:
      ros::NodeHandlePtr rosNode_pr;
      ros::ServiceServer PytorchService;
      
      GazeboEnvIO(const std::string node_name="gazebo_env_io")
        :rosNode_pr(new ros::NodeHandle(node_name)){};
      
      virtual bool ServiceCallback(
          gym_style_gazebo::SocialForce::Request&,
          gym_style_gazebo::SocialForce::Response&) =0;

      virtual float rewardCalculate() =0; 
      
      virtual bool terminalCheck()  =0;
      
      virtual bool reset()  =0;
  };
}

#endif


