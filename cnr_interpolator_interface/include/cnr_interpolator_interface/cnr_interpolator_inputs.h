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
 * @brief The JointInput struct
 */
class JointInput : public InterpolationInput
{
private:
  trajectory_msgs::JointTrajectoryPoint pnt_;
public:
  JointInput() = default;
  virtual ~JointInput() = default;
  JointInput(const JointInput&) = delete;
  JointInput& operator=(const JointInput&) = delete;
  JointInput(JointInput&&) = delete;
  JointInput& operator=(JointInput&&) = delete;

  const trajectory_msgs::JointTrajectoryPoint& pnt() const { return pnt_; };
  trajectory_msgs::JointTrajectoryPoint& pnt() { return pnt_; };
};
typedef std::shared_ptr<JointInput> JointInputPtr;
typedef std::shared_ptr<JointInput const > JointInputConstPtr;



/**
 * @brief The CartesianInput struct
 */
struct CartesianInput : public InterpolationInput
{
  CartesianInput() = default;
  virtual ~CartesianInput() = default;
  CartesianInput(const CartesianInput&) = delete;
  CartesianInput& operator=(const CartesianInput&) = delete;
  CartesianInput(CartesianInput&&) = delete;
  CartesianInput& operator=(CartesianInput&&) = delete;

};
typedef std::shared_ptr<CartesianInput> CartesianInputPtr;
typedef std::shared_ptr<CartesianInput const > CartesianInputConstPtr;


}  // namespace control
}  // cnr

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INPUTS__H
