#include <dynamixel_tilt/tflaserscan2basetilt.h>



scan2point::scan2point()
{
        if (!n.getParam("tfScan2Base/point_scan_tilt", point_scan_tilt))//topic name
        {
          ROS_ERROR("Failed to get  param 'tfScan2Base/point_scan_tilt', set  'tfScan2Base/point_scan_tilt'to defult value ");
          n.param<std::string>("tfScan2Base/point_scan_tilt", point_scan_tilt, std::string("point_scan_tilt"));
        }

        if (!n.getParam("tfScan2Base/point_scan_base", point_scan_base))//topic name
        {
          ROS_ERROR("Failed to get  param 'tfScan2Base/point_scan_base', set  'tfScan2Base/point_scan_base'to defult value ");
          n.param<std::string>("tfScan2Base/point_scan_base", point_scan_base, std::string("point_scan_base"));
        }

        if (!n.getParam("tfScan2Base/point_tilt_frame", point_tilt_frame))//cloud frame_id
        {
          ROS_ERROR("Failed to get  param 'tfScan2Base/point_tilt_frame', set  'tfScan2Base/point_tilt_frame'to defult value ");
          n.param<std::string>("tfScan2Base/point_tilt_frame", point_tilt_frame, std::string(" base_tilt"));
        }

        if (!n.getParam("tfScan2Base/point_base_frame", point_base_frame))//cloud frame_id
        {
          ROS_ERROR("Failed to get  param 'tfScan2Base/point_base_frame', set  'tfScan2Base/point_base_frame'to defult value ");
          n.param<std::string>("tfScan2Base/point_base_frame", point_base_frame, std::string(" base_scan"));
        }

      sub = n.subscribe("/scan",  100, &scan2point::scanCallback,this);
       //    sub = n.subscribe("sync_scan_cloud_filtered",  2, &scan2point::pointCallback,this);
      pubtilt = n.advertise<sensor_msgs::PointCloud2>(point_scan_tilt, 10);
      pubbase = n.advertise<sensor_msgs::PointCloud2>(point_scan_base, 40);
      timer =  n.createTimer(ros::Duration(1) ,&scan2point::counter,this);//1000ms执行一次GoalCycle回调
}
void scan2point::pointCallback (const sensor_msgs::PointCloud2ConstPtr& point_in)
{
 // ROS_INFO("COME IN");
   coutlaser++;
}


void scan2point::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
   coutlaser++;
  if(!listener.waitForTransform(
       scan_in->header.frame_id,
        "/base_tilt",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0/40))){
     return;
  }
  //projector_.transformLaserScanToPointCloud(  "/base_tilt",*scan_in,cloud,listener);
    projector_.transformLaserScanToPointCloud(  "/base_laser",*scan_in,cloud_tilt,listener);
    projector_.projectLaser( *scan_in,cloud_base);
    cloud_base.header.frame_id=point_base_frame;
    cloud_tilt.header.frame_id=point_tilt_frame;
    pubtilt.publish(cloud_tilt);
    pubtilt.publish(cloud_base);

}
void  scan2point::counter(const ros::TimerEvent &){
  ROS_INFO("point frequency :%d ",coutlaser);
 coutlaser=0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tfLaserScan2BaseTilt");
  scan2point s2p;
   ros::spin();

 return 0;
}
