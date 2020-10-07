#ifndef CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INPUTS__H
#define CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INPUTS__H

#include <cnr_logger/cnr_logger.h>
#include <eigen3/Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>

namespace cnr_interpolator_interface
{


/**
 * @brief The InterpolationInput struct
 */
struct InterpolationInput
{
  InterpolationInput() : time(0.0), override(1.0) {}
  virtual ~InterpolationInput() = default;
  InterpolationInput(const InterpolationInput&) = delete;
  InterpolationInput& operator=(const InterpolationInput&) = delete;
  InterpolationInput(InterpolationInput&&) = delete;
  InterpolationInput& operator=(InterpolationInput&&) = delete;

  ros::Duration time;
  double override;
};
typedef std::shared_ptr<InterpolationInput> InterpolationInputPtr;
typedef const std::shared_ptr<InterpolationInput const > InterpolationInputConstPtr;



/**
 * @brief The JointInput struct
 */
struct JointInput : public cnr_interpolator_interface::InterpolationInput
{
  JointInput() = default;
  virtual ~JointInput() = default;
  JointInput(const JointInput&) = delete;
  JointInput& operator=(const JointInput&) = delete;
  JointInput(JointInput&&) = delete;
  JointInput& operator=(JointInput&&) = delete;

  trajectory_msgs::JointTrajectoryPoint pnt;
};
typedef std::shared_ptr<JointInput> JointInputPtr;
typedef const std::shared_ptr<JointInput const > JointInputConstPtr;



/**
 * @brief The CartesianInput struct
 */
struct CartesianInput : public cnr_interpolator_interface::InterpolationInput
{
  CartesianInput() = default;
  virtual ~CartesianInput() = default;
  CartesianInput(const CartesianInput&) = delete;
  CartesianInput& operator=(const CartesianInput&) = delete;
  CartesianInput(CartesianInput&&) = delete;
  CartesianInput& operator=(CartesianInput&&) = delete;

};
typedef std::shared_ptr<CartesianInput> CartesianInputPtr;
typedef const std::shared_ptr<CartesianInput const > CartesianInputConstPtr;


}  // namespace cnr_interpolator_interface

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INPUTS__H
