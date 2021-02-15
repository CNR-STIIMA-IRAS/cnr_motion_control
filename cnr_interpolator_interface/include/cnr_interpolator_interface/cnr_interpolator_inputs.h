#ifndef CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INPUTS__H
#define CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INPUTS__H

#include <cnr_logger/cnr_logger.h>
#include <Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>

namespace cnr
{
namespace control
{

/**
 * @brief The InterpolationInput struct
 */
class InterpolationInput
{
private:
  ros::Duration time_;
  double override_;
public:
  InterpolationInput() : time_(0.0), override_(1.0) {}
  virtual ~InterpolationInput() = default;
  InterpolationInput(const InterpolationInput&) = delete;
  InterpolationInput& operator=(const InterpolationInput&) = delete;
  InterpolationInput(InterpolationInput&&) = delete;
  InterpolationInput& operator=(InterpolationInput&&) = delete;

  const ros::Duration& time() const {return time_; }
  const double& override() const {return override_; }
  ros::Duration& time() {return time_; }
  double& override() {return override_; }

};
typedef std::shared_ptr<InterpolationInput> InterpolationInputPtr;
typedef std::shared_ptr<InterpolationInput const> InterpolationInputConstPtr;



/**
 * @brief The JointInterpolatorInput struct
 */
class JointInterpolatorInput : public InterpolationInput
{
private:
  trajectory_msgs::JointTrajectoryPoint pnt_;
public:
  JointInterpolatorInput() = default;
  virtual ~JointInterpolatorInput() = default;
  JointInterpolatorInput(const JointInterpolatorInput&) = delete;
  JointInterpolatorInput& operator=(const JointInterpolatorInput&) = delete;
  JointInterpolatorInput(JointInterpolatorInput&&) = delete;
  JointInterpolatorInput& operator=(JointInterpolatorInput&&) = delete;

  const trajectory_msgs::JointTrajectoryPoint& pnt() const { return pnt_; };
  trajectory_msgs::JointTrajectoryPoint& pnt() { return pnt_; };
};
typedef std::shared_ptr<JointInterpolatorInput> JointInterpolatorInputPtr;
typedef std::shared_ptr<JointInterpolatorInput const > JointInterpolatorInputConstPtr;



/**
 * @brief The CartesianInterpolatorInput struct
 */
struct CartesianInterpolatorInput : public InterpolationInput
{
  CartesianInterpolatorInput() = default;
  virtual ~CartesianInterpolatorInput() = default;
  CartesianInterpolatorInput(const CartesianInterpolatorInput&) = delete;
  CartesianInterpolatorInput& operator=(const CartesianInterpolatorInput&) = delete;
  CartesianInterpolatorInput(CartesianInterpolatorInput&&) = delete;
  CartesianInterpolatorInput& operator=(CartesianInterpolatorInput&&) = delete;

};
typedef std::shared_ptr<CartesianInterpolatorInput> CartesianInterpolatorInputPtr;
typedef std::shared_ptr<CartesianInterpolatorInput const > CartesianInterpolatorInputConstPtr;


}  // namespace control
}  // cnr

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INPUTS__H
