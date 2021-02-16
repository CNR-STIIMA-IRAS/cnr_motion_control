#ifndef CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_OUTPUTS__H
#define CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_OUTPUTS__H

#include <cnr_logger/cnr_logger.h>
#include <Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>

namespace cnr
{
namespace control
{

/**
 * @brief The InterpolatorOutput struct
 */
struct InterpolatorOutput
{
  InterpolatorOutput() = default;
  virtual ~InterpolatorOutput() = default;
  InterpolatorOutput(const InterpolatorOutput&) = delete;
  InterpolatorOutput& operator=(const InterpolatorOutput&) = delete;
  InterpolatorOutput(InterpolatorOutput&&) = delete;
  InterpolatorOutput& operator=(InterpolatorOutput&&) = delete;
};

typedef std::shared_ptr<InterpolatorOutput> InterpolatorOutputPtr;
typedef std::shared_ptr<InterpolatorOutput const > InterpolatorOutputConstPtr;


/**
 * @brief The JointInterpolatorOutput struct
 */
struct JointInterpolatorOutput : public InterpolatorOutput
{
  JointInterpolatorOutput() = default;
  virtual ~JointInterpolatorOutput() = default;
  JointInterpolatorOutput(const JointInterpolatorOutput&) = delete;
  JointInterpolatorOutput& operator=(const JointInterpolatorOutput&) = delete;
  JointInterpolatorOutput(JointInterpolatorOutput&&) = delete;
  JointInterpolatorOutput& operator=(JointInterpolatorOutput&&) = delete;

  trajectory_msgs::JointTrajectoryPoint pnt;
};

typedef std::shared_ptr<JointInterpolatorOutput> JointInterpolatorOutputPtr;
typedef std::shared_ptr<JointInterpolatorOutput const > JointInterpolatorOutputConstPtr;




/**
 * @brief The CartesianInterpolatorOutput struct
 */
struct CartesianInterpolatorOutput : public InterpolatorOutput
{
  CartesianInterpolatorOutput() = default;
  virtual ~CartesianInterpolatorOutput() = default;
  CartesianInterpolatorOutput(const CartesianInterpolatorOutput&) = delete;
  CartesianInterpolatorOutput& operator=(const CartesianInterpolatorOutput&) = delete;
  CartesianInterpolatorOutput(CartesianInterpolatorOutput&&) = delete;
  CartesianInterpolatorOutput& operator=(CartesianInterpolatorOutput&&) = delete;

  CartesianInterpolatorPoint pnt;
};

typedef std::shared_ptr<CartesianInterpolatorOutput> CartesianInterpolatorOutputPtr;
typedef std::shared_ptr<CartesianInterpolatorOutput const > CartesianInterpolatorOutputConstPtr;



} // namespace control
} // namespace cnr

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_OUTPUTS__H
