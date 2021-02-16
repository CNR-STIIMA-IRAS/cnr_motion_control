#ifndef CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_POINT__H
#define CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_POINT__H

#include <rosdyn_core/spacevect_algebra.h>
#include <Eigen/Dense>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace cnr
{
namespace control
{


/**
 * @brief The InterpolatorPoint struct
 */
struct InterpolatorPoint
{
  InterpolatorPoint() = default;
  virtual ~InterpolatorPoint() = default;
  InterpolatorPoint(const InterpolatorPoint&) = default;
  InterpolatorPoint& operator=(const InterpolatorPoint&) = default;
  InterpolatorPoint(InterpolatorPoint&&) = default;
  InterpolatorPoint& operator=(InterpolatorPoint&&) = default;
};

typedef std::shared_ptr<InterpolatorPoint> InterpolatorPointPtr;
typedef std::shared_ptr<InterpolatorPoint const > InterpolatorPointConstPtr;


/**
 * @brief The JointInterpolatorPoint struct
 */
struct JointInterpolatorPoint : public InterpolatorPoint
{
  JointInterpolatorPoint() = default;
  virtual ~JointInterpolatorPoint() = default;
  JointInterpolatorPoint(const JointInterpolatorPoint&) = delete;
  JointInterpolatorPoint& operator=(const JointInterpolatorPoint&) = delete;
  JointInterpolatorPoint(JointInterpolatorPoint&&) = delete;
  JointInterpolatorPoint& operator=(JointInterpolatorPoint&&) = delete;

  trajectory_msgs::JointTrajectoryPoint pnt;
};

typedef std::shared_ptr<JointInterpolatorPoint> JointInterpolatorPointPtr;
typedef std::shared_ptr<JointInterpolatorPoint const > JointInterpolatorPointConstPtr;


/**
 * @brief The CartesianInterpolatorPoint struct
 */
struct CartesianInterpolatorPoint : public InterpolatorPoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CartesianInterpolatorPoint() = default;
  virtual ~CartesianInterpolatorPoint() = default;
  CartesianInterpolatorPoint(const CartesianInterpolatorPoint&) = default;
  CartesianInterpolatorPoint& operator=(const CartesianInterpolatorPoint&) = default;

  ros::Duration   time_from_start;
  Eigen::Affine3d x;
  Eigen::Vector6d twist;
  Eigen::Vector6d twistd;
};

typedef std::shared_ptr<CartesianInterpolatorPoint> CartesianInterpolatorPointPtr;
typedef std::shared_ptr<CartesianInterpolatorPoint const > CartesianInterpolatorPointConstPtr;



}  //  namespace control
}  //  namespace cnr

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_POINT__H
