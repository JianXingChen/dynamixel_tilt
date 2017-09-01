#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include<sensor_msgs/PointCloud2.h>

using namespace laser_assembler;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "call_assemble");
    ros::NodeHandle   n;
    ros::Time               last_time=ros::Time::now();
    double                   call_period;

    if (!n.getParam("call_assemble/call_period", call_period))
    {
      ROS_ERROR("Failed to get period param 'call_assemble/call_period', set  'call_assemble/call_period'to defult value ");
      n.param<double>("call_assemble/call_period", call_period, double(1));
    }

    ros::service::waitForService("assemble_scans");
    ros::ServiceClient client = n.serviceClient<AssembleScans2>("assemble_scans2");
    ros::Publisher        _pub = n.advertise<sensor_msgs::PointCloud2>("assember_scan2", 1000);
    AssembleScans2   srv;
    ros::Rate    loop_rate(1/call_period);//loop rate unit is hz
    ROS_INFO("call period : %.2f",call_period);
    while (ros::ok())
    {
      srv.request.begin =last_time;
      srv.request.end   =ros::Time::now();
      last_time=ros::Time::now();
      if (client.call(srv)){
         ROS_INFO("Got cloud with : %u points\n",srv.response.cloud.width);
        _pub.publish(srv.response.cloud);
        loop_rate.sleep();
        }
      else{
        printf("Service call failed\n");
        }
    }
    return 0;
}
