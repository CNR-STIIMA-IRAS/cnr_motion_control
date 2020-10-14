#ifndef CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_POINT__H
#define CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_POINT__H

#include <rosdyn_core/spacevect_algebra.h>
#include <eigen3/Eigen/Dense>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace cnr_interpolator_interface
{

/**
 * @brief The InterpolationPoint struct
 */
struct InterpolationPoint
{
  InterpolationPoint() = default;
  virtual ~InterpolationPoint() = default;
  InterpolationPoint(const InterpolationPoint&) = default;
  InterpolationPoint& operator=(const InterpolationPoint&) = default;
  InterpolationPoint(InterpolationPoint&&) = default;
  InterpolationPoint& operator=(InterpolationPoint&&) = default;
};

typedef std::shared_ptr<InterpolationPoint> InterpolationPointPtr;
typedef std::shared_ptr<InterpolationPoint const > InterpolationPointConstPtr;


/**
 * @brief The JointPoint struct
 */
struct JointPoint : public cnr_interpolator_interface::InterpolationPoint
{
  JointPoint() = default;
  virtual ~JointPoint() = default;
  JointPoint(const JointPoint&) = delete;
  JointPoint& operator=(const JointPoint&) = delete;
  JointPoint(JointPoint&&) = delete;
  JointPoint& operator=(JointPoint&&) = delete;

  trajectory_msgs::JointTrajectoryPoint pnt;
};

typedef std::shared_ptr<JointPoint> JointPointPtr;
typedef std::shared_ptr<JointPoint const > JointPointConstPtr;


/**
 * @brief The CartesianPoint struct
 */
struct CartesianPoint : public cnr_interpolator_interface::InterpolationPoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CartesianPoint() = default;
  virtual ~CartesianPoint() = default;
  CartesianPoint(const CartesianPoint&) = default;
  CartesianPoint& operator=(const CartesianPoint&) = default;

  ros::Duration   time_from_start;
  Eigen::Affine3d x;
  Eigen::Vector6d twist;
  Eigen::Vector6d twistd;
};

typedef std::shared_ptr<CartesianPoint> CartesianPointPtr;
typedef std::shared_ptr<CartesianPoint const > CartesianPointConstPtr;



}  //  namespace cnr_interpolator_interface

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_POINT__H
