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
 * @brief The InterpolatorInput struct
 */
class InterpolatorInput
{
private:
  ros::Duration time_;
  double override_;
public:
  InterpolatorInput() : time_(0.0), override_(1.0) {}
  virtual ~InterpolatorInput() = default;
  InterpolatorInput(const InterpolatorInput&) = delete;
  InterpolatorInput& operator=(const InterpolatorInput&) = delete;
  InterpolatorInput(InterpolatorInput&&) = delete;
  InterpolatorInput& operator=(InterpolatorInput&&) = delete;

  const ros::Duration& time() const {return time_; }
  const double& override() const {return override_; }
  ros::Duration& time() {return time_; }
  double& override() {return override_; }

};
typedef std::shared_ptr<InterpolatorInput> InterpolatorInputPtr;
typedef std::shared_ptr<InterpolatorInput const> InterpolatorInputConstPtr;



/**
 * @brief The JointInterpolatorInput struct
 */
class JointInterpolatorInput : public InterpolatorInput
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
struct CartesianInterpolatorInput : public InterpolatorInput
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
