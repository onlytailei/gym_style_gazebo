/*************************************************************************
	> File Name: main.cpp
	> Author: TAI Lei
	> Mail: lei.tai@my.cityu.edu.hk
	> Created Time: Di 09 Mai 2017 18:45:03 CEST
 ************************************************************************/

#include<iostream>
#include<string>

#include "gazebo_env_io.h"
#include "rl_utils_child.h"



int main(int argc, char **argv){
  std::string node_name = "gazebo_env_io";
  using action_type = geometry_msgs::Twist; 
  using state_type = sensor_msgs::Image; 
  
  ros::init(argc, argv, node_name);
  
  RL::DynamicNavi_RL_Utils my_rl_utils;
  
  // Change the topic type to your tasks related type
  RL::GazeboEnvIO<state_type, action_type>  ge_IO(
            &my_rl_utils, 
            "/camera/depth/image_raw",
            "/mobile_bas/moving_command/velocity",
            "pytorch_service",
            node_name);
  
  ros::spin();
  return 0;
}


























int main(){

  // TODO initialize the ros node
  
  // 

}
