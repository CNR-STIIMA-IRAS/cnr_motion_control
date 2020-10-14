#ifndef CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_BASE__H
#define CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_BASE__H

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
 * @brief The InterpolatorBase class
 */
class InterpolatorBase
{
protected:
  ros::NodeHandle  m_controller_nh;
  ros::Duration    m_starting_duration;
  ros::Duration    m_interpolator_time;
  bool             m_new_trajectory_interpolation_started;

  InterpolationTrajectoryPtr  m_trj;
  InterpolationInputConstPtr  m_in;
  InterpolationOutputPtr      m_out;

  cnr_logger::TraceLoggerPtr m_logger;

  ros::NodeHandle& getControllerNh() { return m_controller_nh;}

public:
  InterpolatorBase() = default;
  virtual ~InterpolatorBase() = default;
  InterpolatorBase(const InterpolatorBase&) = delete;
  InterpolatorBase& operator=(const InterpolatorBase&) = delete;
  InterpolatorBase(InterpolatorBase&&) = delete;
  InterpolatorBase& operator=(InterpolatorBase&&) = delete;

  virtual bool initialize(cnr_logger::TraceLoggerPtr logger, ros::NodeHandle& controller_nh,
                          InterpolationTrajectoryPtr  trj = nullptr);

  virtual bool setTrajectory(InterpolationTrajectoryPtr trj);
  virtual bool appendToTrajectory(InterpolationPointConstPtr /*point*/);
  virtual const ros::Duration& trjTime() const;
  virtual bool interpolate(InterpolationInputConstPtr input, InterpolationOutputPtr /*output*/);
  virtual InterpolationPointConstPtr getLastInterpolatedPoint() const;
  
  virtual InterpolationTrajectoryConstPtr getTrajectory() const;
  virtual const ros::Duration& interpolatorTime() const;
  cnr_logger::TraceLoggerPtr& logger() {return m_logger; }
};

typedef std::shared_ptr<cnr_interpolator_interface::InterpolatorBase> InterpolatorBasePtr;
typedef std::shared_ptr<cnr_interpolator_interface::InterpolatorBase const> InterpolatorBaseConstPtr;

}  // namespace cnr_interpolator_interface

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INTERFACE__H
