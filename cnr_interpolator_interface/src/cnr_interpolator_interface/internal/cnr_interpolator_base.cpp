#include <cnr_interpolator_interface/internal/cnr_interpolator_base.h>

namespace cnr
{
namespace control
{

bool InterpolatorBase::initialize(cnr_logger::TraceLoggerPtr logger,
                                  ros::NodeHandle&           controller_nh,
                                  InterpolationTrajectoryPtr trj )
{
  m_controller_nh = controller_nh;
  if(logger)
  {
    m_logger = logger;
  }
  else
  {
    ROS_ERROR("The interpolator has none logger set!");
    return false;
  }
  CNR_TRACE_START(m_logger);
  m_starting_duration = ros::Duration(0);
  m_new_trajectory_interpolation_started = false;
  bool ret = true;
  if(trj)
  {
    ret = setTrajectory(trj);
  }
  CNR_RETURN_BOOL(m_logger, ret);
}

bool InterpolatorBase::setTrajectory(InterpolationTrajectoryPtr trj)
{
  if(!trj)
  {
    CNR_RETURN_FALSE(m_logger, "Trajectory is not set");
  }

  if(m_trj)
  {
    m_trj.reset();
  }
  m_trj = trj;
  m_starting_duration = ros::Duration(0);
  m_new_trajectory_interpolation_started = false;
  CNR_RETURN_TRUE(m_logger);
}

bool InterpolatorBase::appendToTrajectory(InterpolationPointConstPtr point)
{
  CNR_TRACE_START(m_logger);
  if(!point)
  {
    CNR_RETURN_FALSE(this->logger(), "Null pointer as input.");
  }
  CNR_RETURN_TRUE(m_logger);
}

const ros::Duration& InterpolatorBase::trjTime() const
{
  static ros::Duration default_duration(0);
  if(m_trj)
  {
    return m_trj->trjTime();
  }
  else
  {
    return default_duration;
  }
}

bool InterpolatorBase::interpolate(InterpolationInputConstPtr input, InterpolationOutputPtr output)
{
  if(!input || !output)
  {
    CNR_RETURN_FALSE(this->logger(), "Null pointer as input.");
  }

  if(m_new_trajectory_interpolation_started)
  {
    m_new_trajectory_interpolation_started = true;
    m_starting_duration = input->time();
  }
  m_interpolator_time = input->time() - m_starting_duration;
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());

}

InterpolationPointConstPtr InterpolatorBase::getLastInterpolatedPoint() const
{
  return nullptr;
}

InterpolationTrajectoryConstPtr InterpolatorBase::getTrajectory() const
{
  return m_trj;
}

const ros::Duration& InterpolatorBase::interpolatorTime() const
{
  return m_interpolator_time;
}


}  // namespace control
}  // namespace cnr
