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
  ros::Duration    m_starting_duration;
  ros::Duration    m_interpolator_time;
  bool             m_new_trajectory_interpolation_started;
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

  virtual bool setTrajectory(InterpolationTrajectoryPtr trj)
  {
    if(!trj)
    {
      CNR_RETURN_FALSE(*m_logger, "Trajectory is not set");
    }
    m_starting_duration = ros::Duration(0);
    m_new_trajectory_interpolation_started = false;
    CNR_RETURN_TRUE(*m_logger);
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

  virtual bool interpolate(InterpolationInputConstPtr input, InterpolationOutputPtr /*output*/)
  {
    if(m_new_trajectory_interpolation_started)
    {
      m_new_trajectory_interpolation_started = true;
      m_starting_duration = input->time;
    }
    m_interpolator_time = input->time - m_starting_duration;
    return true;

  }
  virtual InterpolationPointConstPtr getLastInterpolatedPoint() const
  {
    return nullptr;
  }

  virtual InterpolationTrajectoryConstPtr getTrajectory() const
  {
    return nullptr;
  }

  virtual const ros::Duration& interpolatorTime() const
  {
    return m_interpolator_time;
  }
};

typedef std::shared_ptr<cnr_interpolator_interface::InterpolatorInterface> InterpolatorInterfacePtr;
typedef const std::shared_ptr<cnr_interpolator_interface::InterpolatorInterface const> InterpolatorInterfaceConstPtr;

}  // namespace cnr_interpolator_interface

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INTERFACE__H
