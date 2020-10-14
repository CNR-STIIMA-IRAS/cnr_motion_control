#ifndef CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_STATES__H
#define CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_STATES__H

#include <cnr_logger/cnr_logger.h>
#include <eigen3/Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>

namespace cnr_interpolator_interface
{


/**
 * @brief The InterpolationState struct
 */
struct InterpolationState
{
  InterpolationState() = default;
  virtual ~InterpolationState() = default;
  InterpolationState(const InterpolationState&) = delete;
  InterpolationState& operator=(const InterpolationState&) = delete;
  InterpolationState(InterpolationState&&) = delete;
  InterpolationState& operator=(InterpolationState&&) = delete;
};
typedef std::shared_ptr<InterpolationState> InterpolationStatePtr;
typedef std::shared_ptr<InterpolationState const > InterpolationStateConstPtr;



/**
 * @brief The JointState struct
 */
struct JointState : public cnr_interpolator_interface::InterpolationState
{
  JointState() = default;
  virtual ~JointState() = default;
  JointState(const JointState&) = delete;
  JointState& operator=(const JointState&) = delete;
  JointState(JointState&&) = delete;
  JointState& operator=(JointState&&) = delete;
};
typedef std::shared_ptr<JointState> JointStatePtr;
typedef std::shared_ptr<JointState const > JointStateConstPtr;



/**
 * @brief The CartesianState struct
 */
struct CartesianState : public cnr_interpolator_interface::InterpolationState
{
  CartesianState() = default;
  virtual ~CartesianState() = default;
  CartesianState(const CartesianState&) = delete;
  CartesianState& operator=(const CartesianState&) = delete;
  CartesianState(CartesianState&&) = delete;
  CartesianState& operator=(CartesianState&&) = delete;

};
typedef std::shared_ptr<CartesianState> CartesianStatePtr;
typedef std::shared_ptr<CartesianState const > CartesianStateConstPtr;


}  // namespace cnr_interpolator_interface

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_STATES__H
