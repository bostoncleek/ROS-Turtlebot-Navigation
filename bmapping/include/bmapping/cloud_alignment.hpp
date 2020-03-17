#ifndef CLOUD_ALIGNMENT_GUARD_HPP
#define CLOUD_ALIGNMENT_GUARD_HPP
/// \file
/// \brief A wrapper library for point cloud library scan matcher algorithm (ICP)


#include <cmath>
#include <iosfwd>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>


#include <rigid2d/rigid2d.hpp>
#include "bmapping/sensor_model.hpp"



namespace bmapping
{
  using rigid2d::Vector2D;
  using rigid2d::Transform2D;
  using rigid2d::TransformData2D;

  /// \brief compose transform between two point clouds
  class ScanAlignment
  {
  public:
    /// \brief creates a scan matcher
    /// param beam_min - start angle of scan
    /// param beam_max - end angle of scan
    /// param beam_delta - increment scan angle
    /// param range_min - min range limit for laser
    /// param range_max - max range limit for laser
    // ScanAlignment(float beam_min, float beam_max, float beam_delta,
    //                               float range_min, float range_max);
    ScanAlignment(const LaserProperties &props, const Transform2D &Trs);

    /// \brief Creates point cloud and call pclICP from the current laser scan
    /// \parma T_init - initial guess
    /// T [out] - 2D transform between clouds
    /// \param beam_length - a vector of raw laser range measurements
    bool pclICPWrapper(Transform2D &T, const Transform2D &T_init,
                          const std::vector<float> &beam_length);

  private:

    /// \brief creates a 3D point cloud (height of 1) from raw
    ///        2D laser range measurements
    /// \param cloud - ptr to pcl corresponding to range measurements
    /// \param beam_length - a vector of raw laser range measurements
    void createPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                               const std::vector<float> &beam_length) const;

    /// \brief uses pcl ICP API to align two point clouds
    /// \parma T_init - initial guess
    /// \param target - corresponds to the target scan
    /// \param source - corresponds to the source scan
    /// T [out] - 2D transform between clouds
    bool pclICP(Transform2D &T, const Transform2D &T_init,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr source) const;


    double max_iter_;                         // iterations optimization runs for
    double max_correspondence_dist_;          // max correspondence distance
    double transform_epsilon_;                // difference btw transforms
    double fitness_epsilon_;                  // sum of Euclidean squared errors

    Transform2D Trs_;                         // robot to laser scanner
    float beam_min_, beam_max_, beam_delta_;  // start, end, increment scan angles
    float range_min_, range_max_;             // min and max range limit for laser

    bool first_scan_recieved;                 // whether the first scan has been recieved

    std::vector<float> old_scan;              // store the old laser scan

  };

} // end namespace

#endif
