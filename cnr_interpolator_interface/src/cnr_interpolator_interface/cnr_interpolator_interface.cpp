#include <cnr_interpolator_interface/cnr_interpolator_interface.h>


namespace cnr_interpolator_interface
{


bool InterpolatorInterface::initialize(cnr_logger::TraceLoggerPtr logger,
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
  CNR_TRACE_START(*m_logger);
  m_starting_duration = ros::Duration(0);
  m_new_trajectory_interpolation_started = false;
  bool ret = true;
  if(trj)
  {
    ret = setTrajectory(trj);
  }
  CNR_RETURN_BOOL(*m_logger, ret);
}



}  // namespace cnr_interpolator_interface
