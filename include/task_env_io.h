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
#include <geometry_msgs/Quaternion.h>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ignition/math.hh>
#include "gazebo_env_io.h"

namespace RL {
  // Template class for subscriber 
  template<typename topicType>
    class GetNewTopic{

      public: std::vector<topicType> StateVector;
      private: ros::Subscriber StateSub;
      private: void StateCallback(const topicType& );

      public: GetNewTopic(ros::NodeHandlePtr, const std::string);
    };

  // typedef
  using STATE_1_TYPE = sensor_msgs::ImageConstPtr;
  using STATE_2_TYPE = gazebo_msgs::ModelStates;
  using ACTION_TYPE = geometry_msgs::Twist;
  // ang_velocity, lin_velocity
  //using ROBOT_STATE = std::array<float, 6>; 
  const std::string ROBOT_NAME = "turtlebot3_burger";
  //const std::string TARGET_NAME = "Construction_Barrel";
  const std::string ACTOR_NAME_BASE= "actor";

  // target pose structure
  struct Pose2 {
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
      float collisionReward;
      float time_discount;
      float max_lin_vel;
      float max_ang_vel;
      int action_sleep_time;
      bool enable_collision_terminal;
      bool enable_continuous_control;
      bool enable_ped;
      float robot_x_start;
      float robot_x_end;
      float robot_y_start;
      float robot_y_end;
      float robot_yaw_start;
      float robot_yaw_end;
      float hard_ped_th;
      float neighbor_range;
      float depth_fov;
      float actor_number;
      float desired_force_factor;
      float social_force_factor;
  };

  // Main class inherit from gazeboenvio
  class TaskEnvIO : public RL::GazeboEnvIO{
    private:
      ros::Publisher ActionPub;
      std::shared_ptr<RL::GetNewTopic<RL::STATE_1_TYPE>> state_1;
      std::shared_ptr<RL::GetNewTopic<RL::STATE_2_TYPE>> state_2;
      std::shared_ptr<RL::GetNewTopic<sensor_msgs::LaserScanConstPtr>> laser_scan;
      ros::ServiceClient SetModelPositionClient;
      ros::ServiceClient GetModelPositionClient;
      ros::ServiceClient SetActorTargetClient;
      
      const std::shared_ptr<ParamLoad> paramlist;

      bool setModelPosition(const float, const float, const geometry_msgs::Quaternion, const std::string=RL::ROBOT_NAME);
      bool setActorTarget(const float, const float);
      bool CollisionCheck(const ignition::math::Pose3d) const;
      bool TargetCheck(const ignition::math::Pose3d);
      geometry_msgs::Pose findPosebyName(const std::string) const;
      ignition::math::Pose3d gazePose2IgnPose(const geometry_msgs::Pose) const;
      void actionPub(const float, const float);
      
      //void updatePedStates(
          //const geometry_msgs::Pose, 
          //const gazebo_msgs::ModelStates, 
          //const std::vector<std::string>);
      
      float getQuaternionYaw(const geometry_msgs::Quaternion &) const; 
      
      bool terminal_flag;
      ignition::math::Vector3d target_pose;
      ignition::math::Pose3d robot_ignition_state;

      gazebo_msgs::ModelState robot_model_state;
      gazebo_msgs::ModelStates newStates;

      // randomizatoin
      std::mt19937 random_engine;
      std::uniform_real_distribution<> dis;
      std::uniform_real_distribution<> target_gen;

      const float sleeping_time_;

      virtual bool ServiceCallback(
          gym_style_gazebo::SocialForce::Request&,
          gym_style_gazebo::SocialForce::Response&);

      float rewardCalculate(const ignition::math::Pose3d);
      virtual float rewardCalculate();
      virtual bool terminalCheck();
      virtual bool reset();

    public: 
      TaskEnvIO(
          const std::string service_name="pytorch_io_service",
          const std::string node_name="gazebo_env_io",
          const float sleeping_time=0.2);
  };
  ignition::math::Pose3d gazePose2IgnPose(const geometry_msgs::Pose);
  ignition::math::Quaterniond yaw2Quaterniond(const ignition::math::Angle);
}

#endif
