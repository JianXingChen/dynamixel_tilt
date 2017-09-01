#include <ros/ros.h>
#include"dynamixel_tilt/motor_tilt_node.h"
#include "stdio.h"


Dynamixel::Dynamixel():goal(0),goalrang(20)
{

  sub = n.subscribe("/motor_states/pan_tilt_port",  1, &Dynamixel::MsgCallback, this);
  ROS_INFO("Subscribe: /motor_states/pan_tilt_port");

  pub = n.advertise< std_msgs::Float64 >("/tilt_controller/command", 1);
   ROS_INFO("Advertise: /motor_states/pan_tilt_port");

   if (!n.getParam("tilt_cycle/motor_speed", motor_speed))
   {
     ROS_ERROR("Failed to get  speed param 'motor_speed', set  'motor_speed'to defult value ");
     n.param<double>("tilt_cycle/motor_speed", motor_speed, double(1.0));
   }
   ros::service::waitForService("/tilt_controller/set_speed");
   ros::ServiceClient client = n.serviceClient<dynamixel_controllers::SetSpeed>("/tilt_controller/set_speed");
   dynamixel_controllers::SetSpeed srv;
   srv.request.speed=motor_speed*3,1415;
   if (!client.call(srv))
   {
     ROS_ERROR("Failed to call service set speed");
   }

  cycle_time=1.0/motor_speed;
  timer =  n.createTimer(ros::Duration(40.0/180.0f) ,&Dynamixel::GoalCycle,this);//1000ms执行一次GoalCycle回调
}

void Dynamixel::moveMotor(double position)
{
std_msgs::Float64 aux;
aux.data=position;
pub.publish(aux);
}
void Dynamixel::GoalSingleContinue(const ros::TimerEvent &)
{

static  float cnt=0;
int goal=-80;

   if((cnt>=0)&&(cnt<=abs(goal)))
   {
   moveMotor(((goal>0)?cnt:-cnt)*3.14f/180.0f);
   cnt++;
   }
   else {
   cnt=goal;}

moveMotor(goal*3.14f/180.0f);

}

void Dynamixel::GoalSingle(const ros::TimerEvent &)
{

static int goal=-70;
moveMotor(goal*3.14f/180.0f);
}
void Dynamixel::GoalCycle(const ros::TimerEvent &)
{
  //  ROS_INFO(" motor send frequency : %d",backcount);
    //backcount=0;


goalrang=-goalrang;

goal=goalrang-50;
//  ROS_INFO(" motor send frequency : %d",backcount);
ROS_INFO("goal : %d",goal);
//ROS_INFO("goalrang : %d",goalrang);
moveMotor((float)goal*3.1415f/180.0f);
}

void Dynamixel::GoalCycleContinue(const ros::TimerEvent &)
{

    static  float cnt=0;
    int initposition=0;
    int goal=90;
    if((cnt>=0)&&(cnt<=2*goal))
    {
    moveMotor((cnt-goal-initposition)*3.14/180);
    }
    else if((cnt>2*goal)&&((cnt)<=4*goal))
    {
        moveMotor((3*goal-cnt-initposition)*3.14/180);
    }else
    {
    cnt=0;
    }
    cnt++;

}


void Dynamixel::MsgCallback(const dynamixel_msgs::MotorStateList &msg)//motor send back frequency about 20hz
{
   backcount++;
    dynamixel_msgs::MotorState ms;

    ms=msg.motor_states[0];
    double lidar_roll=-(ms.position-2230.0)*(360.0f/4096.0f);//zhongdian


  //cout<<ros::Time(ms.timestamp)<<endl;;

   //ROS_INFO("MotorPosition : %f", lidar_roll);
    lidar_roll=(lidar_roll/180.0)*3.14159265;
    tfbc.sendTransform(
            tf::StampedTransform(  tf::Transform(tf::Quaternion(lidar_roll,0,0),tf::Vector3(0,0,0)),
                                                   ros::Time::now(),"base_tilt" , "base_laser" ) );
}
Dynamixel::~Dynamixel()
{

   cout<<"Reset motor"<<endl;

  }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_tilt_node");
  Dynamixel motor;
   ros::spin();

}
