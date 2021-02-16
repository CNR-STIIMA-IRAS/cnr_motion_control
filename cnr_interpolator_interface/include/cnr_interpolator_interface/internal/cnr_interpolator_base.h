#pragma once

#ifndef CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_BASE__H
#define CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_BASE__H

#include <Eigen/Dense>

#include <trajectory_msgs/JointTrajectory.h>
#include <ros/node_handle.h>

#include <cnr_logger/cnr_logger.h>
#include <cnr_interpolator_interface/cnr_interpolator_state.h>
#include <cnr_interpolator_interface/cnr_interpolator_inputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_outputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_point.h>
#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>

namespace cnr
{
namespace control
{

/**
 * @brief The InterpolatorBase class
 */
class InterpolatorBase
{
protected:
  ros::NodeHandle  m_controller_nh;
  ros::Duration    m_starting_duration;
  ros::Duration    m_interpolator_time;
  bool             m_new_trajectory_interpolator_started;

  InterpolatorTrajectoryPtr  m_trj;
  InterpolatorInputConstPtr   m_in;
  InterpolatorOutputPtr       m_out;

  cnr_logger::TraceLoggerPtr  m_logger;

  ros::NodeHandle& getControllerNh() { return m_controller_nh;}

public:
  InterpolatorBase() = default;
  virtual ~InterpolatorBase() = default;
  InterpolatorBase(const InterpolatorBase&) = delete;
  InterpolatorBase& operator=(const InterpolatorBase&) = delete;
  InterpolatorBase(InterpolatorBase&&) = delete;
  InterpolatorBase& operator=(InterpolatorBase&&) = delete;

  virtual bool initialize(cnr_logger::TraceLoggerPtr logger, ros::NodeHandle& controller_nh,
                          InterpolatorTrajectoryPtr  trj = nullptr);

  virtual bool setTrajectory(InterpolatorTrajectoryPtr trj);
  virtual bool appendToTrajectory(InterpolatorPointConstPtr /*point*/);
  virtual const ros::Duration& trjTime() const;
  virtual bool interpolate(InterpolatorInputConstPtr input, InterpolatorOutputPtr /*output*/);
  virtual InterpolatorPointConstPtr getLastInterpolatedPoint() const;
  
  virtual InterpolatorTrajectoryConstPtr getTrajectory() const;
  virtual const ros::Duration& interpolatorTime() const;
  cnr_logger::TraceLoggerPtr& logger() {return m_logger; }
};

typedef std::shared_ptr<InterpolatorBase> InterpolatorBasePtr;
typedef std::shared_ptr<InterpolatorBase const> InterpolatorBaseConstPtr;

}  // namespace control
}  // namespace cnr

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INTERFACE__H
