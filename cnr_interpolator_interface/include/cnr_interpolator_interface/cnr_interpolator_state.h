#ifndef CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_STATES__H
#define CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_STATES__H

#include <cnr_logger/cnr_logger.h>
#include <Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>


namespace cnr
{
namespace control
{

/**
 * @brief The InterpolatorState struct
 */
struct InterpolatorState
{
  InterpolatorState() = default;
  virtual ~InterpolatorState() = default;
  InterpolatorState(const InterpolatorState&) = delete;
  InterpolatorState& operator=(const InterpolatorState&) = delete;
  InterpolatorState(InterpolatorState&&) = delete;
  InterpolatorState& operator=(InterpolatorState&&) = delete;
};
typedef std::shared_ptr<InterpolatorState> InterpolatorStatePtr;
typedef std::shared_ptr<InterpolatorState const > InterpolatorStateConstPtr;



/**
 * @brief The JointInterpolatorState struct
 */
struct JointInterpolatorState : public InterpolatorState
{
  JointInterpolatorState() = default;
  virtual ~JointInterpolatorState() = default;
  JointInterpolatorState(const JointInterpolatorState&) = delete;
  JointInterpolatorState& operator=(const JointInterpolatorState&) = delete;
  JointInterpolatorState(JointInterpolatorState&&) = delete;
  JointInterpolatorState& operator=(JointInterpolatorState&&) = delete;
};
typedef std::shared_ptr<JointInterpolatorState> JointInterpolatorStatePtr;
typedef std::shared_ptr<JointInterpolatorState const > JointInterpolatorStateConstPtr;



/**
 * @brief The CartesianInterpolatorState struct
 */
struct CartesianInterpolatorState : public InterpolatorState
{
  CartesianInterpolatorState() = default;
  virtual ~CartesianInterpolatorState() = default;
  CartesianInterpolatorState(const CartesianInterpolatorState&) = delete;
  CartesianInterpolatorState& operator=(const CartesianInterpolatorState&) = delete;
  CartesianInterpolatorState(CartesianInterpolatorState&&) = delete;
  CartesianInterpolatorState& operator=(CartesianInterpolatorState&&) = delete;

};
typedef std::shared_ptr<CartesianInterpolatorState> CartesianInterpolatorStatePtr;
typedef std::shared_ptr<CartesianInterpolatorState const > CartesianInterpolatorStateConstPtr;


}  // namespace control
}  // namespace cnr

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_STATES__H
