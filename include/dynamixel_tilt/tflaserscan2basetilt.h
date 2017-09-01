#ifndef TFLASERSCAN2BASETILT_H
#define TFLASERSCAN2BASETILT_H


#include <ros/ros.h>
#include<laser_geometry/laser_geometry.h>
#include<laser_filters/range_filter.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud2.h>
#include<tf/transform_listener.h>
#include<tf_conversions/tf_eigen.h>
#include<Eigen/Eigen>

using namespace std;
using namespace sensor_msgs;
 class scan2point
    {
    public:

         scan2point();
         void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
         void pointCallback (const PointCloud2ConstPtr &point_in);
         void  laserscanCallback(const LaserScan::ConstPtr &lasermsg);
         void  counter(const ros::TimerEvent &);

    private:
      ros::NodeHandle            n;
      ros::Publisher                pubtilt;
      ros::Publisher                pubbase;
      ros::Subscriber             sub;
      ros::Timer                     timer ;
      tf::TransformListener  listener;
      tf::StampedTransform  transform;
      tf::Quaternion              tq;
      tf:: Vector3                  te;
      Eigen::Vector3d            ee;
      Eigen::Quaterniond       eq;
      laser_geometry::LaserProjection        projector_;
      sensor_msgs::PointCloud2                  cloud_tilt;
      sensor_msgs::PointCloud2                  cloud_base;
      ros::Time                                            error;
      laser_filters::LaserScanRangeFilter  lf;
      std::string                                                 point_scan_tilt;
      std::string                                                 point_scan_base;
      std::string        point_tilt_frame;
      std::string        point_base_frame;
    int coutlaser;
    };




















#endif // TFLASERSCAN2BASETILT_H
