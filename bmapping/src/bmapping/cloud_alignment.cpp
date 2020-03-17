/// \file
/// \brief A wrapper library for point cloud library scan matcher algorithm (ICP)

#include <iostream>
#include <iterator>

#include <eigen3/Eigen/Core>
// #include <Eigen/Dense>
// #include <Eigen/Core>

// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/registration/icp.h>

#include "bmapping/cloud_alignment.hpp"


namespace bmapping
{

ScanAlignment::ScanAlignment(const LaserProperties &props, const Transform2D &Trs)
                              : max_iter_(100),
                                max_correspondence_dist_(0.5), // 0.05 = 5 cm
                                transform_epsilon_(1e-8),
                                fitness_epsilon_(1e-6), // between two steps of icp
                                Trs_(Trs),
                                beam_min_(props.beam_min),
                                beam_max_(props.beam_max),
                                beam_delta_(props.beam_delta),
                                range_min_(props.range_min),
                                range_max_(props.range_max),
                                first_scan_recieved(false)
{
}


bool ScanAlignment::pclICPWrapper(Transform2D &T, const Transform2D &T_init,
                                      const std::vector<float> &beam_length)
{
  // TODO: design this so I am not creating the target cloud twice

  // more than one scan has been recieved
  if (first_scan_recieved)
  {

    // creat point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclTarget(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclSource(new pcl::PointCloud<pcl::PointXYZ>);

    createPointCloud(pclTarget, old_scan);
    createPointCloud(pclSource, beam_length);


    // ICP
    if (!pclICP(T, T_init, pclTarget, pclSource))
    {
      return false;
    }

    // save new scan
    old_scan = beam_length;
  }


  if (!first_scan_recieved)
  {
    old_scan = beam_length;
    first_scan_recieved = true;
  }

  return true;
}



void ScanAlignment::createPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                       const std::vector<float> &beam_length) const
{
  // TODO: RANSAC, max correspondences
  // TODO: check for inf or nan in raw measurements


  // number of raw measurements with limits of range finder
  unsigned int num_meas = beam_length.size();

  // number of valid measurements
  unsigned int valid_meas = 0;


  // iterate and find number of valid measurements
  std::vector<float>::const_iterator it;
  for(it = beam_length.begin(); it != beam_length.end(); ++it)
  {
    if (*it >= range_min_ and *it < range_max_)
      valid_meas++;
  }


  // std::cout << "Valid meas: " << valid_meas << " "
  //           << "Raw meas: " << num_meas << std::endl;


  // allocate memory for cloud using number of valid measurements
  cloud->width = valid_meas;
  cloud->height = 1;
  cloud->is_dense = true; // all points are finite, do not contain inf or nan
  cloud->points.resize(cloud->width * cloud->height);

  // end point
  Vector2D point;

  // convert ranges from polar to cartesian
  auto range = 0.0;
  auto beam_angle = beam_min_;

  auto ctr = 0;             // assign valid measurements to pcl
  for(unsigned int i = 0; i < num_meas; i++)
  {
    range = beam_length.at(i);

    if (range >= range_min_ and range < range_max_)
    {
      point.x = range * std::cos(beam_angle);
      point.y = range * std::sin(beam_angle);

      // cartesian coordinates in frame of robot
      // transform from frame of sensor to frame of robot
      // pr = Trs * ps
      point = Trs_(point);


      cloud->points[ctr].x = point.x;
      cloud->points[ctr].y = point.y;
      cloud->points[ctr].z = 1.0;
      ctr++;
    }


    // update beam angle
    beam_angle += beam_delta_;


    // max angle is negative
    if (beam_max_ < 0.0 and beam_angle <= beam_max_)
    {
      beam_angle = beam_min_;
    }

    // max angle is positive
    else if (beam_max_ >= 0.0 and beam_angle >= beam_max_)
    {
      beam_angle = beam_min_;
    }

  } // end loop

}


bool ScanAlignment::pclICP(Transform2D &T, const Transform2D &T_init,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr source) const
{

  // ICP object
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  // source cloud after transform is applied
  pcl::PointCloud<pcl::PointXYZ> Final;

  // initial guess
  TransformData2D Tinit = T_init.displacement();
  float ctheta = std::cos(Tinit.theta);
  float stheta = std::sin(Tinit.theta);

  Eigen::Matrix<float, 4, 4> T_guess;

  // homogeneous transform, rotation about z-axis
  T_guess << ctheta, -stheta, 0, Tinit.x,
             stheta,  ctheta, 0, Tinit.y,
             0,       0,      1,       0,
             0,       0,      0,       1;



  icp.setMaximumIterations(max_iter_);
  icp.setMaxCorrespondenceDistance(max_correspondence_dist_);
  icp.setTransformationEpsilon(transform_epsilon_);
  icp.setEuclideanFitnessEpsilon(fitness_epsilon_);
  icp.setRANSACOutlierRejectionThreshold (0.05);


  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.align(Final, T_guess);

  // std::cout << " score: " << icp.getFitnessScore() << std::endl;


  if (!icp.hasConverged())
  {
    std::cout << "ICP FAILED TO CONVERGED!" << std::endl;
    return false;
  }


  // ICP success
  // retrieve transform
  Eigen::Matrix<float, 4, 4> Tpcl = icp.getFinalTransformation();

  // rotation between clouds
  double theta = std::atan2(Tpcl(1,0), Tpcl(0,0));

  // translation between clouds
  Vector2D trans;
  trans.x = Tpcl(0,3);
  trans.y = Tpcl(1,3);

  Transform2D Tnew(trans, theta);
  T = Tnew;

  return true;
}

} // end namespace
// end file
