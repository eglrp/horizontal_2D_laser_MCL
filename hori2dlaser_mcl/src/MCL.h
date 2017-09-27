#ifndef MCL_H
#define MCL_H

#include "conf.h"
#include <stdio.h>
#include <vector>
#include <numeric>
#include <opencv2/opencv.hpp>
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

namespace UROBOT
{

  class MCL_aglrotihm
  {
  private:
      float m_param_alpha[6];
      cv::Size m_param_gaussianboxsize;

      int m_Numparticle;                           //the number of particles
      cv::Mat m_occugridmap;                       //the occupied map from the low-dynamic class
      cv::Mat m_T_global2pix;                      //transformation the global coordinate to pixel coordinate system
      cv::Mat m_T_laser2odom;                      // transformation laser sensor to robot (odom)
      cv::Mat m_bestpose;                          //estimated pose from the particle filter
      double m_bestposeWeight;                     //the weight of the best particle
      std::vector<cv::Mat> m_particles;            //list of partcles poses
      std::vector<double> m_particles_weight;      //list of particles weights
      cv::Mat m_crtpose;                           //a current pose from graph SLAM
      cv::Mat m_crtodom;                           //a current odometry pose from the pioneer
      cv::Mat m_crtLiDARpts;                       //current LiDAR points
      bool m_flag_initialization;

      void initialParticles(cv::Mat crtPose, double num = -1, cv::Point3f noise = cv::Point3f(0,0,0)); // initalization particles
      void prediction(cv::Mat diffPose);                               // a prediction step for MCL
      void weightnormalize(std::vector<double>& particles_weight);     // normalization of weights
      void weighting(cv::Mat pts);                                     // a weight step for MCL
      void resampling(double Neff);                                    // a resampling step for MCL



  public:
      MCL_aglrotihm(int num = 100);
      ~MCL_aglrotihm();

      void setParam_mapuncertainty(int size = 21); // Gaussian kernel, the number should be positive odd
      void setParam_robotmodel(float alpha1 = 0.1, float alpha2 = 0.01, float alpha3 = 0.1, float alpha4 = 0.001, float alpha5 = 0.01, float alpha6 = 0.01);
      // alphas : errors of rot-rot, trans-rot, trans-trans, rot-trans, dist, and theta.
      void setParam_Tlaser2rt(Eigen::Matrix4f T_laser2rt);  // Transformation matrix laser scanner to robot (odom)

      void setGridmap(nav_msgs::OccupancyGrid map); // set occupied grid map
      void setodompose(Eigen::Matrix4f pose, sensor_msgs::LaserScan laser); // odometry, LiDAR set
      void setcrtPose(Eigen::Matrix4f pose);  // initial pose

      bool spinOnce(float showsize = -1);                        // to perform once of all MCL steps

      inline cv::Mat getBestpose(){return m_bestpose;} // to obtain a estimated pose from MCL
  };

}


#endif

