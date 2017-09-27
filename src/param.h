
#ifndef PARAM_H
#define PARAM_H

#include <vector>
#include <iostream>
#include <fstream>
#include <map>
#include <math.h>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <stdio.h>

#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define PARAM_ICPSEARCH_AREA  2       // unit : meter
#define PARAM_GENSUBMAP_DIST  0.5     // unit : meter
#define PARAM_GENSUBMAP_MAXANGLE  10  // unit : degree


namespace IUSLAM
{
  inline Eigen::Matrix4f GETPARAM_Tlaser2rt()
  {
    Eigen::Matrix4f T_laser2rt = Eigen::Matrix4f::Identity();
    T_laser2rt(0,3) = 0.21;
    return T_laser2rt;
  }
}


#endif
