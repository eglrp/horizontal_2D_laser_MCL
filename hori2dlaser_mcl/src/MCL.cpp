#include "MCL.h"

using namespace UROBOT;

MCL_aglrotihm::MCL_aglrotihm(int num)
{
    m_Numparticle = num;
    m_crtodom = cv::Mat::zeros(4,4,CV_32FC1);
    m_crtpose = cv::Mat::zeros(4,4,CV_32FC1);
    m_crtLiDARpts = cv::Mat::zeros(4,0,CV_32FC1);
    m_occugridmap = cv::Mat::zeros(0,0,CV_8UC1);

    m_T_global2pix = cv::Mat::zeros(4,4,CV_32FC1);
    m_T_global2pix.at<float>(0,0) = 50;
    m_T_global2pix.at<float>(1,1) = -50;
    m_T_global2pix.at<float>(2,2) = -1;
    m_T_global2pix.at<float>(3,3) = 1;

    m_T_global2pix.at<float>(0,3) = 500;
    m_T_global2pix.at<float>(1,3) = 500;

    m_T_laser2odom = cv::Mat::zeros(4,4,CV_32FC1);
    m_T_laser2odom.at<float>(0,0) = 1;
    m_T_laser2odom.at<float>(1,1) = 1;
    m_T_laser2odom.at<float>(2,2) = 1;
    m_T_laser2odom.at<float>(3,3) = 1;
    m_T_laser2odom.at<float>(0,3) = 0.22;

    for(int i= 0; i < m_Numparticle; i++)
    {
        m_particles.push_back(m_crtodom.clone());
        m_particles_weight.push_back(1);
    }
    m_bestposeWeight = 1;

    m_flag_initialization = false;

    m_param_gaussianboxsize = cv::Size(21,21);
    m_param_alpha[0] = 0.1;
    m_param_alpha[1] = 0.01;
    m_param_alpha[2] = 0.1;
    m_param_alpha[3] = 0.001;
    m_param_alpha[4] = 0.001;
    m_param_alpha[5] = 0.001;
}

MCL_aglrotihm::~MCL_aglrotihm()
{

}

void MCL_aglrotihm::setParam_mapuncertainty(int size )
{
  if(size <= 0 && size % 2 == 0)
  {
    std::cout <<"Warning : mapuncertainty size error" << std::endl;
    return;
  }
  m_param_gaussianboxsize = cv::Size(size,size);
}

void MCL_aglrotihm::setParam_robotmodel(float alpha1, float alpha2, float alpha3, float alpha4, float alpha5, float alpha6)
{
  m_param_alpha[0] = alpha1;
  m_param_alpha[1] = alpha2;
  m_param_alpha[2] = alpha3;
  m_param_alpha[3] = alpha4;
  m_param_alpha[4] = alpha5;
  m_param_alpha[5] = alpha6;
}

void MCL_aglrotihm::setParam_Tlaser2rt(Eigen::Matrix4f T_laser2rt)
{
   m_T_laser2odom = CVT_eigen2mat(T_laser2rt);
}

void MCL_aglrotihm::setGridmap(nav_msgs::OccupancyGrid map)
{
  int width = map.info.width;
  int height = map.info.height;
  float resolution = map.info.resolution;

  m_T_global2pix.at<float>(0,0) = 1./resolution;
  m_T_global2pix.at<float>(1,1) = 1./resolution;
  m_T_global2pix.at<float>(2,2) = 0;
  m_T_global2pix.at<float>(0,3) = -map.info.origin.position.x / resolution;
  m_T_global2pix.at<float>(1,3) = -map.info.origin.position.y / resolution;

  m_occugridmap = cv::Mat::zeros(height,width,CV_8UC1);
  for(int y = 0; y < height; y++)
  {
    for(int x = 0; x < width; x++)
    {
      int tmp_data = map.data[y * width + x];
      if(tmp_data == -1)  // not searched
      {
        m_occugridmap.at<unsigned char>(y,x) = 0;
      }
      else if(tmp_data <= 50) // free
      {
        m_occugridmap.at<unsigned char>(y,x) = 0;
      }
      else if(tmp_data > 50)  // ocuupied
      {
        m_occugridmap.at<unsigned char>(y,x) = 255;
      }
    }
  }
  cv::GaussianBlur(m_occugridmap,m_occugridmap,m_param_gaussianboxsize,0);

  std::cout << "Map update : " << width << "," << height << std::endl;
  std::cout << m_T_global2pix << std::endl;

}

void MCL_aglrotihm::setodompose(Eigen::Matrix4f pose, sensor_msgs::LaserScan laser)
{
  m_crtodom = CVT_eigen2mat(pose);
  static pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  laser_cloud->clear();
  laser_cloud = CVT_laser2cloud(laser);
  m_crtLiDARpts = cv::Mat::zeros(4,laser_cloud->size(),CV_32FC1);
  int laser_cnt = -1;
  for(int i = 0; i < laser_cloud->size(); i++)
  {
    float dist = sqrt(pow(laser_cloud->at(i).x,2)+pow(laser_cloud->at(i).y,2));
    if(dist < 0.2)
      continue;
    laser_cnt++;
    m_crtLiDARpts.at<float>(0,laser_cnt) = laser_cloud->at(i).x;
    m_crtLiDARpts.at<float>(1,laser_cnt) = laser_cloud->at(i).y;
    m_crtLiDARpts.at<float>(2,laser_cnt) = laser_cloud->at(i).z;
    m_crtLiDARpts.at<float>(3,laser_cnt) = 1;
  }
  if(laser_cnt == 0)
    m_crtLiDARpts = cv::Mat::zeros(4,0,CV_32FC1);
  else
    m_crtLiDARpts(cv::Rect(0,0,laser_cnt,4)).copyTo(m_crtLiDARpts);

  m_crtLiDARpts = m_T_laser2odom * m_crtLiDARpts;
}

void MCL_aglrotihm::setcrtPose(Eigen::Matrix4f pose)
{
  m_crtpose = CVT_eigen2mat(pose);
  std::cout << "Initial pose : " << m_crtpose << std::endl;
  m_flag_initialization = true;

}


void MCL_aglrotihm::initialParticles(cv::Mat crtPose, double num, cv::Point3f noise)
{
  if(num == -1)
  {
    for(int i = 0; i < m_Numparticle; i++)
    {
      cv::Mat noisemat = CVT_xyzrpy2mat(noise.x * GaussianRand(), noise.y * GaussianRand(), 0,0,0,noise.z * M_PI /180 * GaussianRand());
      m_particles[i] = crtPose * noisemat;
      m_particles_weight[i] = 1;
    }
    m_bestpose = crtPose.clone();
  }
  else
  {
    for(int i = 0; i < num; i++)
    {
      m_particles[i] = crtPose;
      m_particles_weight[i] = m_bestposeWeight/2;
    }

  }
}

void MCL_aglrotihm::prediction(cv::Mat diffPose)
{

    float diff_x, diff_y, diff_z, diff_roll, diff_pitch, diff_yaw;
    CVT_mat2xyzrpy(diffPose,&diff_x,&diff_y,&diff_z,&diff_roll,&diff_pitch,&diff_yaw);

    double delta_trans = sqrt(pow(diff_x,2) + pow(diff_y,2));
    double delta_rot1 = atan2((double)diff_y, (double)diff_x);
    if(delta_trans < 0.001)
        delta_rot1 = 0;
    double delta_rot2 = diff_yaw - delta_rot1;

    ///wrap around
    if(delta_rot1  > CV_PI)
        delta_rot1 -= (2*CV_PI);
    if(delta_rot1  < -CV_PI)
        delta_rot1 += (2*CV_PI);
    if(delta_rot2  > CV_PI)
        delta_rot2 -= (2*CV_PI);
    if(delta_rot2  < -CV_PI)
        delta_rot2 += (2*CV_PI);

    //double alpha[6] = {0.1, 0.01, 0.1, 0.001, 0.01, 0.1};
    //double alpha[6] = {0, 0, 0, 0, 0, 0};
    // rot-rot, trans-rot, trans-trans, rot-trans, x, y

    double delta_trans_err = m_param_alpha[2] * abs(delta_trans) + m_param_alpha[3] * abs(delta_rot1 + delta_rot2);
    double delta_rot1_err = m_param_alpha[0] * abs(delta_rot1) + m_param_alpha[1] * abs(delta_trans);
    double delta_rot2_err = m_param_alpha[0] * abs(delta_rot2) + m_param_alpha[1] * abs(delta_trans);

    double tmp_diff_x = delta_trans * cos(delta_rot1);
    double tmp_diff_y = delta_trans * sin(delta_rot1);
    double tmp_diff_theta = delta_rot1 + delta_rot2;


    if(m_particles.size() == 0)
    {
        std::cout << "Warning : no particles" << std::endl;
        return ;
    }

    //m_particles[0] = m_particles[0] * CVT_xyzrpy2mat(tmp_diff_x,tmp_diff_y,diff_z, diff_roll,diff_pitch,tmp_diff_theta);
    //m_particles[0].at<float>(2,3) = height;
    m_bestpose = m_bestpose * CVT_xyzrpy2mat(tmp_diff_x,tmp_diff_y,diff_z, diff_roll,diff_pitch,tmp_diff_theta);
    //m_bestpose.at<float>(2,3) = height;
    for(int i = 0; i < m_particles.size(); i++)
    {

        double tilde_trans = delta_trans + GaussianRand() * delta_trans_err;
        double tilde_rot1 = delta_rot1 + GaussianRand() * delta_rot1_err;
        double tilde_rot2 = delta_rot2 + GaussianRand() * delta_rot2_err;

        double tmp_diff_x = tilde_trans * cos(tilde_rot1) + GaussianRand() * m_param_alpha[4];
        double tmp_diff_y = tilde_trans * sin(tilde_rot1) + GaussianRand() * m_param_alpha[4];
        double tmp_diff_theta = tilde_rot1 + tilde_rot2 + GaussianRand() * m_param_alpha[5] * CV_PI / 180;

        double tmp_diff_z = diff_z;
        double tmp_diff_roll = diff_roll;
        double tmp_diff_pitch = diff_pitch;
        m_particles[i] = m_particles[i] * CVT_xyzrpy2mat(tmp_diff_x,tmp_diff_y,tmp_diff_z, tmp_diff_roll,tmp_diff_pitch,tmp_diff_theta);

    }
}

void MCL_aglrotihm::weighting(cv::Mat pts)
{

    cv::Mat pts_onGrid;
    static unsigned char* gridmap_data = (unsigned char*) m_occugridmap.data;
    for(int i = 0; i < m_particles.size();i++)
    {

        pts_onGrid = m_T_global2pix * m_particles[i] * pts;
        float* pts_onGrid_data = (float*)pts_onGrid.data;
        float weight = 0;
        for(int j = 0; j < pts_onGrid.size().width; j++)
        {
            //int pix_x = pts_onGrid.at<float>(0,j) + 0.5;
            //int pix_y = pts_onGrid.at<float>(1,j) + 0.5;
            int pix_x = pts_onGrid_data[0 * pts_onGrid.cols + j] + 0.5;
            int pix_y = pts_onGrid_data[1 * pts_onGrid.cols + j] + 0.5;

            if(pix_x < 0 || pix_x >= m_occugridmap.size().width || pix_y < 0 || pix_y >= m_occugridmap.size().height)
                continue;

            weight += ((float)gridmap_data[pix_y * m_occugridmap.cols + pix_x] / 255);

        }
        m_particles_weight[i] *= weight / pts.size().width + 0.00000000001 ;
    }
    weightnormalize(m_particles_weight);

}


void MCL_aglrotihm::weightnormalize(std::vector<double>& particles_weight)
{
    double sum_intens = std::accumulate(particles_weight.begin(),particles_weight.end(),0.0);
    if(sum_intens != 0)
    {
        for(int i = 0; i < particles_weight.size(); i++)
        {
            particles_weight[i] /= sum_intens;
        }
    }
    else
    {
        for(int i = 0; i < particles_weight.size(); i++)
        {
            particles_weight[i] = 1. /particles_weight.size();
        }
    }
}

void MCL_aglrotihm::resampling(double Neff)
{
    // N eff
    double sum_weight = 0;
    for(int i = 0; i < m_particles_weight.size(); i++)
    {
        sum_weight += pow((double)m_particles_weight[i],2);
    }

    //TODO: what is N_eff? variance?
    // A : to check effectiveness of particle.
    //      reference : https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/optreadings/Arulampalam_etal_2002.pdf
    double N_eff = 1./ sum_weight / m_particles_weight.size();

    if(N_eff > Neff)
        return;


    double weightSum = 0;
    std::vector<double>::iterator biggest = std::max_element(m_particles_weight.begin(),m_particles_weight.end());
    int maxIdx = std::distance(m_particles_weight.begin(), biggest);
    m_bestpose = m_particles[maxIdx].clone();
    m_bestposeWeight = *std::max_element(m_particles_weight.begin(), m_particles_weight.end());

    //std::cout << "Weight : " <<  max_weight << std::endl;

    //// roulette
    for(int i = 0; i < m_particles_weight.size(); i++)
    {
        weightSum += m_particles_weight[i];
    }
    std::vector<double> rand_roulette;
    for(int i = 0; i < m_Numparticle; i++)
    {
        rand_roulette.push_back((double) rand() / (RAND_MAX) * weightSum);
    }

    std::sort(rand_roulette.begin(),rand_roulette.end());

    double check_roulette = 0;
    int i_roulette = 0;
    std::vector<cv::Mat> tmp_particles;
    std::vector<double> tmp_weights;
    for(int i = 0; i < m_particles_weight.size(); i++)
    {
        check_roulette += m_particles_weight[i];
        while(rand_roulette[i_roulette] <= check_roulette &&  i_roulette < rand_roulette.size())
        {
            tmp_particles.push_back(m_particles[i]);

            tmp_weights.push_back(m_particles_weight[i]);
            i_roulette++;
        }
    }
    m_particles.clear();
    m_particles_weight.clear();
    for(int i = 0; i < tmp_particles.size(); i++)
    {
        m_particles.push_back(tmp_particles[i].clone());
        m_particles_weight.push_back(1./tmp_particles.size());
    }
}




bool MCL_aglrotihm::spinOnce(float showsize)
{
    if(m_occugridmap.size().width == 0)
    {
        std::cout << "Warning : no grid map " << std::endl;
        return false;
    }
    if(m_crtLiDARpts.size().width == 0)
    {
        std::cout << "Warning : no lidar pts " << std::endl;
        return false;
    }

    static bool firstrun = false;
    static cv::Mat preOdom = m_crtodom.clone();
    if(m_flag_initialization)
    {
      preOdom = m_crtodom.clone();
      m_flag_initialization = false;
      initialParticles(m_crtpose, -1, cv::Point3f(0.1,0.1,3));
    }
    if(!firstrun)
    {
        firstrun = true;
        initialParticles(m_crtpose, -1, cv::Point3f(0.1,0.1,3));
        return false;
    }


    cv::Mat diffPose = preOdom.inv() * m_crtodom;
    preOdom = m_crtodom.clone();
    prediction(diffPose);
    weighting(m_crtLiDARpts);
    //initialParticles(m_crtpose, 1);
    resampling(0.8);
    std::cout << "Best : " << m_bestpose.at<float>(0,3) << "," << m_bestpose.at<float>(1,3) << std::endl;

    if(showsize > 0)
    {

        cv::Mat showmat = cv::Mat(m_occugridmap.size(), CV_8UC3);
        cv::cvtColor(m_occugridmap,showmat, CV_GRAY2BGR);
        cv::Mat pts = m_T_global2pix *  m_bestpose * m_crtLiDARpts;
        for(int i = 0; i < pts.cols; i++)
        {
            cv::Point tmp_xy = cv::Point(pts.at<float>(0,i)+0.5,pts.at<float>(1,i) +0.5);
            cv::circle(showmat,tmp_xy,1,cv::Scalar(0,0,255),1);
        }
        cv::Mat pt_o = cv::Mat::zeros(4,1,CV_32FC1);
        pt_o.at<float>(3) = 1;
        cv::Mat pt_x = cv::Mat::zeros(4,1,CV_32FC1);
        pt_x.at<float>(0) = 0.3;
        pt_x.at<float>(3) = 1;
        for(int i = 0; i < m_particles.size(); i++)
        {
            cv::Mat pt_o_pix = m_T_global2pix * m_particles[i] * pt_o;
            cv::Mat pt_x_pix = m_T_global2pix * m_particles[i] * pt_x;
            cv::Point cvPt_o = cv::Point(pt_o_pix.at<float>(0) + 0.5,pt_o_pix.at<float>(1) + 0.5);
            cv::Point cvPt_x = cv::Point(pt_x_pix.at<float>(0) + 0.5,pt_x_pix.at<float>(1) + 0.5);
            cv::circle(showmat, cvPt_o, 2, cv::Scalar(0,255,0),2);
            cv::line(showmat,cvPt_o, cvPt_x, cv::Scalar(0,255,0),2);

        }
        cv::Mat pt_o_pix = m_T_global2pix * m_bestpose * pt_o;
        cv::Mat pt_x_pix = m_T_global2pix * m_bestpose * pt_x;
        cv::Point cvPt_o = cv::Point(pt_o_pix.at<float>(0) + 0.5,pt_o_pix.at<float>(1) + 0.5);
        cv::Point cvPt_x = cv::Point(pt_x_pix.at<float>(0) + 0.5,pt_x_pix.at<float>(1) + 0.5);
        cv::circle(showmat, cvPt_o, 2, cv::Scalar(0,0,255),2);
        cv::line(showmat,cvPt_o, cvPt_x, cv::Scalar(0,0,255),2);

        cv::Mat showmat_resize = cv::Mat::zeros(showmat.size().height * showsize, showmat.size().width * showsize, CV_8UC3);

        cv::resize(showmat,showmat_resize,showmat_resize.size());
        cv::imshow("MCL show",showmat_resize);
        //cv::imshow("MCL show",showmat);
        cv::waitKey(1);
    }

    return true;
}
