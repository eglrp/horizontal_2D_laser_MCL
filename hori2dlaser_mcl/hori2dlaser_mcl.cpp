
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#include "conf.h"

#include "src/MCL.h"

#include <sstream>

using namespace UROBOT;

bool g_flag_laser = false;
bool g_flag_odom = false;
sensor_msgs::LaserScan g_laser;
nav_msgs::Odometry g_odom;

MCL_aglrotihm g_mcl;

void callback_occupancymap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg);
void callback_odom(const nav_msgs::Odometry::ConstPtr& msg);
void callback_initialpose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "hori2dlaser_mcl");
    ros::NodeHandle n;

    ros::Subscriber sub1 = n.subscribe("/occupancygridmap",1000, callback_occupancymap);
    ros::Subscriber sub2 = n.subscribe("/scan",1000, callback_laser);               // hokuyo
    ros::Subscriber sub3 = n.subscribe("/RosAria/pose",1000, callback_odom);        // odometry from pioneer
    ros::Subscriber sub4 = n.subscribe("/initialpose",1000, callback_initialpose);  // rviz initialpose

    ros::Publisher pub_mclpose = n.advertise<nav_msgs::Odometry>("/hjk_pb/mclpose", 1);

    // map load
    nav_msgs::OccupancyGrid gridmap;
    if(!loadOccupancymap("ocuupancymap_all",gridmap))
      std::cout << "Warning : no initial map" <<std::endl;

    Eigen::Matrix4f T_l2r = Eigen::Matrix4f::Identity();
    T_l2r(0,3) = 0.21;

    // initalization of mcl
    g_mcl.setGridmap(gridmap);
    g_mcl.setcrtPose(Eigen::Matrix4f::Identity());
    g_mcl.setParam_Tlaser2rt(T_l2r);      // transformation laser 2 robot
    g_mcl.setParam_mapuncertainty(21);    // size of Gaussian filter for map
    g_mcl.setParam_robotmodel(0.1,0.01,0.1,0.001,0.01,0.01);  // robot noise model


    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        if(g_flag_laser && g_flag_odom)
        {
          g_mcl.setodompose(CVT_geoPose2eigen(g_odom.pose.pose),g_laser);
          g_flag_laser = false;
          g_flag_odom = false;
          if(g_mcl.spinOnce(2))
          {
            nav_msgs::Odometry mclpose;
            mclpose = g_odom;
            mclpose.header.frame_id = "map";
            mclpose.pose.pose = CVT_mat2geoPose(g_mcl.getBestpose());
            pub_mclpose.publish(mclpose);
          }
        }

        loop_rate.sleep();
    }


    return 0;
}

void callback_occupancymap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  g_mcl.setGridmap(*msg);
}

void callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  g_laser= *msg;
  g_flag_laser = true;
}

void callback_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
  g_odom = *msg;
  g_flag_odom = true;
}

void callback_initialpose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  g_mcl.setcrtPose(CVT_geoPose2eigen(msg->pose.pose));
}
