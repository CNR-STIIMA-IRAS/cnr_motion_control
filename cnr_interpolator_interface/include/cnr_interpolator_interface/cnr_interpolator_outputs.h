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
 * @brief The InterpolationOutput struct
 */
struct InterpolationOutput
{
  InterpolationOutput() = default;
  virtual ~InterpolationOutput() = default;
  InterpolationOutput(const InterpolationOutput&) = delete;
  InterpolationOutput& operator=(const InterpolationOutput&) = delete;
  InterpolationOutput(InterpolationOutput&&) = delete;
  InterpolationOutput& operator=(InterpolationOutput&&) = delete;
};

typedef std::shared_ptr<InterpolationOutput> InterpolationOutputPtr;
typedef std::shared_ptr<InterpolationOutput const > InterpolationOutputConstPtr;


/**
 * @brief The JointOutput struct
 */
struct JointOutput : public InterpolationOutput
{
  JointOutput() = default;
  virtual ~JointOutput() = default;
  JointOutput(const JointOutput&) = delete;
  JointOutput& operator=(const JointOutput&) = delete;
  JointOutput(JointOutput&&) = delete;
  JointOutput& operator=(JointOutput&&) = delete;

  trajectory_msgs::JointTrajectoryPoint pnt;
};

typedef std::shared_ptr<JointOutput> JointOutputPtr;
typedef std::shared_ptr<JointOutput const > JointOutputConstPtr;




/**
 * @brief The CartesianOutput struct
 */
struct CartesianOutput : public InterpolationOutput
{
  CartesianOutput() = default;
  virtual ~CartesianOutput() = default;
  CartesianOutput(const CartesianOutput&) = delete;
  CartesianOutput& operator=(const CartesianOutput&) = delete;
  CartesianOutput(CartesianOutput&&) = delete;
  CartesianOutput& operator=(CartesianOutput&&) = delete;

  CartesianPoint pnt;
};

typedef std::shared_ptr<CartesianOutput> CartesianOutputPtr;
typedef std::shared_ptr<CartesianOutput const > CartesianOutputConstPtr;



} // namespace control
} // namespace cnr

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_OUTPUTS__H
