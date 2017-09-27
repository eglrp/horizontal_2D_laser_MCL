
#ifndef CONF_H
#define CONF_H

#include <vector>
#include <iostream>
#include <fstream>
#include <map>
#include <math.h>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <stdio.h>


#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/file_io.h>

#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
//#include "utm.h"
//#include "param.h"

#include "ros/ros.h"
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>

#include "cv_bridge/cv_bridge.h"
#include "nav_msgs/OccupancyGrid.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define MSG_IUMATCHER_NOJOB       0
#define MSG_IUMATCHER_ICPREQUEST  1
#define MSG_IUMATCHER_ICPRESULT   2

//# 0 : odom cons, 1 : icp cons
#define MSG_CONS_ODOM  0
#define MSG_CONS_MATCH   1

#define WORLD_FRAME "map"

namespace
{
    class __GET_TICK_COUNT
    {
    public:
        __GET_TICK_COUNT()
        {
        if (gettimeofday(&tv_, NULL) != 0)
            throw 0;
        }
        timeval tv_;
    };
    __GET_TICK_COUNT timeStart;
}
using namespace std;

namespace UROBOT
{

    inline Eigen::Matrix4f CVT_mat2eigen(cv::Mat mat);
    inline cv::Mat CVT_eigen2mat(Eigen::Matrix4f mat);
    inline void CVT_mat2xyzrpy(cv::Mat mat, float *x, float *y, float *z, float *roll, float *pitch, float *yaw);
    inline cv::Mat CVT_xyzrpy2mat(float x, float y, float z, float roll, float pitch, float yaw);
    inline cv::Vec3b heightcolor(double h);

    inline unsigned long GetTickCount(int option = 0)   //0 : ms / 1:ns
    {
        static time_t   secStart    = timeStart.tv_.tv_sec;
        static time_t   usecStart   = timeStart.tv_.tv_usec;
                    timeval tv;
        gettimeofday(&tv, NULL);
        if(option == 0)
        return (tv.tv_sec - secStart) * 1000 + (tv.tv_usec - usecStart) / 1000;
        else
        return (tv.tv_sec - secStart) * 1000000 + (tv.tv_usec - usecStart);
    }

    static unsigned long tictoctime = 0;
    inline void tic()
    {
      tictoctime = GetTickCount(1);
    }

    inline void toc()
    {
      unsigned long difftime = GetTickCount(1) - tictoctime;
      if(difftime < 1000)
        std::cout << "Tictoc : " << difftime << "ns" << std::endl;
      else if(difftime < 1000000)
      {
        double tmp = (double)difftime/1000;
        std::cout << "Tictoc : " << tmp << "ms" << std::endl;
      }
      else if(difftime < 1000000000)
      {
        double tmp = (double)difftime/1000000;
        std::cout << "Tictoc : " << tmp << "s" << std::endl;
      }
    }

    inline std_msgs::String CVT_str2msgStr(std::string str)
    {
      std_msgs::String msgStr;
      msgStr.data = str;
      return msgStr;
    }

    inline std::string CVT_msgStr2str(std_msgs::String msgStr)
    {
      return msgStr.data;
    }

    // Eigen matrix
    inline geometry_msgs::Pose CVT_eigen2geoPose(Eigen::Matrix4f pose)
    {
        geometry_msgs::Pose geoPose;


        tf::Matrix3x3 m;
        m.setValue((double)pose(0,0),
                (double)pose(0,1),
                (double)pose(0,2),
                (double)pose(1,0),
                (double)pose(1,1),
                (double)pose(1,2),
                (double)pose(2,0),
                (double)pose(2,1),
                (double)pose(2,2));

        tf::Quaternion q;
        m.getRotation(q);
        geoPose.orientation.x = q.getX();
        geoPose.orientation.y = q.getY();
        geoPose.orientation.z = q.getZ();
        geoPose.orientation.w = q.getW();

        geoPose.position.x = pose(0,3);
        geoPose.position.y = pose(1,3);
        geoPose.position.z = pose(2,3);

        return geoPose;
    }

    inline Eigen::Matrix4d CVT_eigenf2eigend(Eigen::Matrix4f pose)
    {
      Eigen::Matrix4d result;
      for(int y = 0; y < 4; y++)
      {
        for(int x = 0; x < 4; x ++)
        {
          result(y,x) = pose(y,x);
        }
      }
      return result;
    }

    inline Eigen::Matrix4f CVT_eigend2eigenf(Eigen::Matrix4d pose)
    {
      Eigen::Matrix4f result;
      for(int y = 0; y < 4; y++)
      {
        for(int x = 0; x < 4; x ++)
        {
          result(y,x) = pose(y,x);
        }
      }
      return result;
    }

    inline Eigen::Matrix4f CVT_geoPose2eigen(geometry_msgs::Pose geoPose)
    {
        Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
        tf::Quaternion q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w);
        tf::Matrix3x3 m(q);
        result(0,0) = m[0][0];
        result(0,1) = m[0][1];
        result(0,2) = m[0][2];
        result(1,0) = m[1][0];
        result(1,1) = m[1][1];
        result(1,2) = m[1][2];
        result(2,0) = m[2][0];
        result(2,1) = m[2][1];
        result(2,2) = m[2][2];
        result(3,3) = 1;

        result(0,3) = geoPose.position.x;
        result(1,3) = geoPose.position.y;
        result(2,3) = geoPose.position.z;

        return result;
    }


    inline Eigen::VectorXf CVT_eigen2xyzrpy(Eigen::Matrix4f mat)
    {
        Eigen::VectorXf result(6);


        CVT_mat2xyzrpy(CVT_eigen2mat(mat), &result[0], &result[1], &result[2], &result[3], &result[4], &result[5]);


//        result[0] = mat(0,3);
//        result[1] = mat(1,3);
//        result[2] = mat(2,3);

//        Eigen::Matrix3f rotmat = mat.block<3,3>(0,0);
//        Eigen::Vector3f vecEuler = rotmat.eulerAngles(2,1,0);
//        result[3] = vecEuler[2]; // roll
//        result[4] = vecEuler[1]; // pitch
//        result[5] = vecEuler[0]; // yaw

        return result;
    }


    inline Eigen::Matrix4f CVT_xyzrpy2eigen(float x, float y, float z, float roll, float pitch, float yaw)
    {
        Eigen::Matrix4f result =  CVT_mat2eigen(CVT_xyzrpy2mat(x,y,z,roll,pitch,yaw));



//        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
//        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
//        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
//        Eigen::Matrix3f rotmat = yawAngle.matrix() * pitchAngle.matrix() * rollAngle.matrix();

//        result.block<3,3>(0,0) = rotmat;
//        result(0,3) = x;
//        result(1,3) = y;
//        result(2,3) = z;

        return result;
    }

    inline Eigen::Matrix4f CVT_mat2eigen(cv::Mat mat)
    {
        Eigen::Matrix4f result =  Eigen::Matrix4f::Identity();

        result(0,0) = mat.at<float>(0,0);
        result(0,1) = mat.at<float>(0,1);
        result(0,2) = mat.at<float>(0,2);
        result(0,3) = mat.at<float>(0,3);

        result(1,0) = mat.at<float>(1,0);
        result(1,1) = mat.at<float>(1,1);
        result(1,2) = mat.at<float>(1,2);
        result(1,3) = mat.at<float>(1,3);

        result(2,0) = mat.at<float>(2,0);
        result(2,1) = mat.at<float>(2,1);
        result(2,2) = mat.at<float>(2,2);
        result(2,3) = mat.at<float>(2,3);

        result(3,0) = mat.at<float>(3,0);
        result(3,1) = mat.at<float>(3,1);
        result(3,2) = mat.at<float>(3,2);
        result(3,3) = mat.at<float>(3,3);

        return result;
    }

    inline cv::Mat CVT_eigen2mat(Eigen::Matrix4f mat)
    {
        cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);
        result.at<float>(0,0) = mat(0,0);
        result.at<float>(0,1) = mat(0,1);
        result.at<float>(0,2) = mat(0,2);
        result.at<float>(0,3) = mat(0,3);

        result.at<float>(1,0) = mat(1,0);
        result.at<float>(1,1) = mat(1,1);
        result.at<float>(1,2) = mat(1,2);
        result.at<float>(1,3) = mat(1,3);

        result.at<float>(2,0) = mat(2,0);
        result.at<float>(2,1) = mat(2,1);
        result.at<float>(2,2) = mat(2,2);
        result.at<float>(2,3) = mat(2,3);

        result.at<float>(3,0) = mat(3,0);
        result.at<float>(3,1) = mat(3,1);
        result.at<float>(3,2) = mat(3,2);
        result.at<float>(3,3) = mat(3,3);

        return result;
    }






    inline geometry_msgs::Pose CVT_mat2geoPose(cv::Mat pose)
    {
        geometry_msgs::Pose geoPose;


        tf::Matrix3x3 m;
        m.setValue((double)pose.at<float>(0,0),
                (double)pose.at<float>(0,1),
                (double)pose.at<float>(0,2),
                (double)pose.at<float>(1,0),
                (double)pose.at<float>(1,1),
                (double)pose.at<float>(1,2),
                (double)pose.at<float>(2,0),
                (double)pose.at<float>(2,1),
                (double)pose.at<float>(2,2));

        tf::Quaternion q;
        m.getRotation(q);
        geoPose.orientation.x = q.getX();
        geoPose.orientation.y = q.getY();
        geoPose.orientation.z = q.getZ();
        geoPose.orientation.w = q.getW();

        geoPose.position.x = pose.at<float>(0,3);
        geoPose.position.y = pose.at<float>(1,3);
        geoPose.position.z = pose.at<float>(2,3);

        return geoPose;
    }

    inline cv::Mat CVT_geoPose2mat(geometry_msgs::Pose geoPose)
    {
        cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);
        tf::Quaternion q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w);
        tf::Matrix3x3 m(q);
        result.at<float>(0,0) = m[0][0];
        result.at<float>(0,1) = m[0][1];
        result.at<float>(0,2) = m[0][2];
        result.at<float>(1,0) = m[1][0];
        result.at<float>(1,1) = m[1][1];
        result.at<float>(1,2) = m[1][2];
        result.at<float>(2,0) = m[2][0];
        result.at<float>(2,1) = m[2][1];
        result.at<float>(2,2) = m[2][2];
        result.at<float>(3,3) = 1;

        result.at<float>(0,3) = geoPose.position.x;
        result.at<float>(1,3) = geoPose.position.y;
        result.at<float>(2,3) = geoPose.position.z;

        return result;
    }



    inline void CVT_mat2q(cv::Mat pose, double &qX, double &qY, double &qZ, double &qW)
    {
        tf::Matrix3x3 m;
        m.setValue((double)pose.at<float>(0,0),
                (double)pose.at<float>(0,1),
                (double)pose.at<float>(0,2),
                (double)pose.at<float>(1,0),
                (double)pose.at<float>(1,1),
                (double)pose.at<float>(1,2),
                (double)pose.at<float>(2,0),
                (double)pose.at<float>(2,1),
                (double)pose.at<float>(2,2));

        tf::Quaternion q;
        m.getRotation(q);

        qX = q.getX();
        qY = q.getY();
        qZ = q.getZ();
        qW = q.getW();
    }

    inline cv::Mat CVT_q2mat(double qX, double qY, double qZ, double qW)
    {

        cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);
        tf::Quaternion q(qX, qY, qZ, qW);
        tf::Matrix3x3 m(q);
        result.at<float>(0,0) = m[0][0];
        result.at<float>(0,1) = m[0][1];
        result.at<float>(0,2) = m[0][2];
        result.at<float>(1,0) = m[1][0];
        result.at<float>(1,1) = m[1][1];
        result.at<float>(1,2) = m[1][2];
        result.at<float>(2,0) = m[2][0];
        result.at<float>(2,1) = m[2][1];
        result.at<float>(2,2) = m[2][2];
        result.at<float>(3,3) = 1;

        return result;
    }

    inline cv::Mat CVT_xyzrpy2mat(float x, float y, float z, float roll, float pitch, float yaw)
    {
            cv::Mat rot_vec = cv::Mat::zeros(3,1,CV_32FC1);
            rot_vec.at<float>(0) = roll;
            rot_vec.at<float>(1) = pitch;
            rot_vec.at<float>(2) = yaw;

            cv::Mat rot_mat;
            cv::Rodrigues(rot_vec,rot_mat);

            cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);

            rot_mat.copyTo(result(cv::Rect(0,0,3,3)));

            result.at<float>(0,3) = x;
            result.at<float>(1,3) = y;
            result.at<float>(2,3) = z;

            result.at<float>(3,3) = 1;

            return result;
    }

    inline void CVT_mat2xyzrpy(cv::Mat mat, float *x, float *y, float *z, float *roll, float *pitch, float *yaw)
    {
        *x = mat.at<float>(0,3);
        *y = mat.at<float>(1,3);
        *z = mat.at<float>(2,3);

        cv::Mat rot_mat = cv::Mat(mat(cv::Rect(0,0,3,3)));

        cv::Mat rot_vec;
        cv::Rodrigues(rot_mat,rot_vec);
        *roll = rot_vec.at<float>(0);
        *pitch = rot_vec.at<float>(1);
        *yaw = rot_vec.at<float>(2);
    }




    inline void CVT_mat2sensorImg(cv::Mat mat, sensor_msgs::Image& sensorImg, std::string frameID)
    {
        cv_bridge::CvImage bridge;
        mat.copyTo(bridge.image);
        bridge.header.frame_id = frameID;
        bridge.header.stamp = ros::Time::now();
        if(mat.type() == CV_8UC1)
        {
            bridge.encoding = sensor_msgs::image_encodings::MONO8;
        }
        else if(mat.type() == CV_8UC3)
        {
            bridge.encoding = sensor_msgs::image_encodings::BGR8;
        }
        else if(mat.type() == CV_32FC1)
        {
            bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        }
        else if(mat.type() == CV_16UC1)
        {
            bridge.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        }
        else
        {
            std::cout <<"Error : mat type" << std::endl;

        }

        bridge.toImageMsg(sensorImg);
    }

    inline cv::Mat CVT_sensorImg2mat(sensor_msgs::Image sensorImg)
    {
        static cv_bridge::CvImagePtr cv_ptr;
        cv::Mat mat;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(sensorImg, sensorImg.encoding);
            mat = cv_ptr->image.clone();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        return mat;
    }

    inline pcl::PointCloud<pcl::PointXYZI>::Ptr CVT_laser2cloud(sensor_msgs::LaserScan laser)
    {
      static laser_geometry::LaserProjection projector;
      sensor_msgs::PointCloud2 cloud_ROS;
      projector.projectLaser(laser, cloud_ROS,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
      static pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
      cloud->clear();
      pcl::fromROSMsg(cloud_ROS, *cloud);



      return cloud;
    }


    inline cv::Point CVT_eigen2cvpt(Eigen::MatrixXf pt)
    {
      return cv::Point(pt(0),pt(1));
    }

    inline double GaussianRand() {
        double u = ((double) rand() / (RAND_MAX)) * 2 - 1;
        double v = ((double) rand() / (RAND_MAX)) * 2 - 1;
        double r = u * u + v * v;
        if (r == 0 || r > 1) return GaussianRand();
        double c = sqrt(-2 * log(r) / r);
        return u * c;
    }

    inline nav_msgs::OccupancyGrid CVT_cvimg2occumap(cv::Mat cvimg, float resolution,cv::Point origin_cvpt)
    {
      nav_msgs::OccupancyGrid m_gridmap;
      m_gridmap.info.resolution = resolution;
      geometry_msgs::Pose origin;
      origin.position.x = -origin_cvpt.x * resolution; origin.position.y = -origin_cvpt.y * resolution;
      origin.orientation.w = 1;
      m_gridmap.info.origin = origin;
      m_gridmap.info.width = cvimg.size().width;
      m_gridmap.info.height = cvimg.size().height;
      ROS_INFO("[CV 2 OCCUGRID CONVERSION] PUT CV DATA");
      for(int i=0;i<m_gridmap.info.width*m_gridmap.info.height;i++) m_gridmap.data.push_back(-1);
      ROS_INFO("[CV 2 OCCUGRID CONVERSION] GRID SIZE : %d",m_gridmap.data.size());
      ROS_INFO("[CV 2 OCCUGRID CONVERSION] GRID ORIGIN : [%d,%d]",origin_cvpt.x,origin_cvpt.y);

      for(int y = 0; y < cvimg.size().height; y++)
      {
        for(int x = 0; x < cvimg.size().width; x++)
        {
          int tmpdata = cvimg.at<unsigned char>(y,x);
          int ttmpdata = -1; //Unknown
          if(tmpdata >= 150) //free
          {
            ttmpdata = (tmpdata - 250) / -2;
            if(ttmpdata > 100)
              ttmpdata = 100;
          }
          else if(tmpdata <= 98)
          {
            ttmpdata = (tmpdata - 200) / -2;
            if(ttmpdata < 0)
              ttmpdata = 0;
          }
          m_gridmap.data.at(x + m_gridmap.info.width * y) = ttmpdata;
        }
      }
      return m_gridmap;
    }

    inline cv::Mat CVT_occumap2cvimg( nav_msgs::OccupancyGrid occumap)
    {
      // unkown = -1 -> 99~149, free : 0:50 ->250:150, occupied :51:100 -> 0:98
      double resolution = occumap.info.resolution;
      cv::Point origin_cvpt(-occumap.info.origin.position.x / resolution,
                            -occumap.info.origin.position.y / resolution);
      cv::Size img_size;
      cv::Mat cvimg = cv::Mat::zeros( occumap.info.height,occumap.info.width,CV_8UC1);
      ROS_INFO("[OCCUGRID 2 CVT CONVERSION] GRID SIZE : %d",occumap.data.size());
      ROS_INFO("[OCCUGRID 2 CVT CONVERSION] CV ORIGIN : [%d,%d]",origin_cvpt.x,origin_cvpt.y);
      for(int pt = 0;pt < occumap.data.size();pt++)
      {
        int pt_y = pt / occumap.info.width;
        int pt_x = pt % occumap.info.width;
        int value = occumap.data.at(pt);
        unsigned char img_value;
        if(value == -1) img_value = 120;
        else if (value <= 50) img_value = 250 - 2 * value;
        else if (value >=51) img_value = 200 - 2 * value;
        cvimg.at<unsigned char>(pt_y,pt_x) = img_value;
      }
      return cvimg;
    }

    inline void saveOccupanymap(std::string filepath, nav_msgs::OccupancyGrid gridmap_in)
    {
      cv::Mat occumat = CVT_occumap2cvimg(gridmap_in);
      std::stringstream strm_png;
      strm_png << filepath << ".png";
      std::stringstream strm_info;
      strm_info << filepath << ".csv";

      cv::imwrite(strm_png.str(),occumat);

      std::ofstream filesave(strm_info.str().c_str());
      if(filesave.is_open())
      {
        filesave << gridmap_in.info.resolution << "\n";
        filesave << gridmap_in.info.origin.position.x << "\n";
        filesave << gridmap_in.info.origin.position.y << "\n";
      }
      filesave.close();
    }

    inline bool loadOccupancymap(std::string filepath, nav_msgs::OccupancyGrid& gridmap_out)
    {
      std::stringstream strm_png;
      strm_png << filepath <<".png";
      std::stringstream strm_info;
      strm_info << filepath << ".csv";
      cv::Mat occumat = cv::imread(strm_png.str(),cv::IMREAD_GRAYSCALE);
      std::ifstream fileload(strm_info.str().c_str());
      float resolution,origin_x,origin_y;
      std::vector<std::string>   result;
      std::string line;
      if(!fileload.is_open())
      {
        ROS_INFO("FILE NOT OPENED");
        return false;
      }
      while(std::getline(fileload,line)) result.push_back(line);
      if(result.size()!=3)
      {
        ROS_INFO("NO SUFFICIENT RESULT or TOO MUCH RESULT");
        return false;
      }
      resolution = std::atof(result.at(0).c_str());
      origin_x = std::atof(result.at(1).c_str());
      origin_y = std::atof(result.at(2).c_str());
      gridmap_out = CVT_cvimg2occumap(occumat, resolution,cv::Point(- origin_x / resolution,-origin_y / resolution));
      gridmap_out.header.frame_id = "map";

      return true;
    }
}

#endif
