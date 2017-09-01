#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/MotorStateList.h>
#include <dynamixel_msgs/MotorState.h>
#include <stdio.h>
#include <iostream>
#include<dynamixel_controllers/SetSpeed.h>
using namespace  std;
 class Dynamixel
    {
    public:
         Dynamixel();

      void moveMotor(double position);
      void MsgCallback(const dynamixel_msgs::MotorStateList &msg);
      void GoalCycle(const ros::TimerEvent &);
      void GoalSingle(const ros::TimerEvent &);
      void GoalCycleContinue(const ros::TimerEvent &);
      void GoalSingleContinue(const ros::TimerEvent &);
      ~Dynamixel();
    private:
      ros::NodeHandle  n;
      ros::Publisher      pub;
      ros::Subscriber   sub;
      tf::TransformBroadcaster tfbc;
      ros::Timer timer  ;
      int backcount;
      double motor_speed;
      double cycle_time;
      int goal;
      int goalrang;
    };


