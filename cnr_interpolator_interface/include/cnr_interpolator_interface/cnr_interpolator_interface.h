#ifndef CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INTERFACE__H
#define CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INTERFACE__H

#include <cnr_logger/cnr_logger.h>
#include <eigen3/Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/node_handle.h>
#include <cnr_interpolator_interface/cnr_interpolator_state.h>
#include <cnr_interpolator_interface/cnr_interpolator_inputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_outputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_point.h>
#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>

namespace cnr_interpolator_interface
{


/**
 * @brief The InterpolatorInterface class
 */
class InterpolatorInterface
{
private:
  ros::NodeHandle  m_controller_nh;

protected:
  cnr_logger::TraceLoggerPtr m_logger;

  ros::NodeHandle& getControllerNh() { return m_controller_nh;}

public:
  InterpolatorInterface() = default;
  virtual ~InterpolatorInterface() = default;
  InterpolatorInterface(const InterpolatorInterface&) = delete;
  InterpolatorInterface& operator=(const InterpolatorInterface&) = delete;
  InterpolatorInterface(InterpolatorInterface&&) = delete;
  InterpolatorInterface& operator=(InterpolatorInterface&&) = delete;

  virtual bool initialize(cnr_logger::TraceLoggerPtr logger,
                          ros::NodeHandle&            controller_nh,
                          InterpolationTrajectoryPtr  trj = nullptr);

  virtual bool setTrajectory(InterpolationTrajectoryPtr /*trj*/)
  {
    return false;
  }
  virtual bool appendToTrajectory(InterpolationPointConstPtr /*point*/)
  {
    return false;
  }
  virtual const ros::Duration& trjTime() const
  {
    static ros::Duration ret = ros::Duration(0);
    return ret;
  }
  virtual bool interpolate(const ros::Duration& /*time*/, InterpolationOutputPtr /*output*/)
  {
    return true;
  }
  virtual bool interpolate(const ros::Duration& /*time*/, InterpolationOutputPtr /*output*/, InterpolationInputConstPtr /*input*/)
  {
    return true;
  }
  virtual InterpolationPointConstPtr getLastInterpolatedPoint()
  {
    return nullptr;
  }
  virtual InterpolationTrajectoryConstPtr getTrajectory()
  {
    return nullptr;
  }
};

typedef std::shared_ptr<cnr_interpolator_interface::InterpolatorInterface> InterpolatorInterfacePtr;
typedef const std::shared_ptr<cnr_interpolator_interface::InterpolatorInterface const> InterpolatorInterfaceConstPtr;

}  // namespace cnr_interpolator_interface

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INTERFACE__H
