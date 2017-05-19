/*************************************************************************
	> File Name: main.cpp
	> Author: TAI Lei
	> Mail: lei.tai@my.cityu.edu.hk
	> Created Time: Di 09 Mai 2017 18:45:03 CEST
 ************************************************************************/

#include <iostream>
#include <string>
#include <ros/ros.h>
#include "task_env_io.h"
#include "gazebo_env_io.h"

int main(int argc, char **argv){
  
  std::string node_name = "gazebo_env_io";
  ros::init(argc, argv, node_name);
  
  RL::TaskEnvIO taskenv(
          "pytorch_io_service",
          "gazebo_env_io",
          0);
  
  //ros::spin();
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  
  return 0;
}

