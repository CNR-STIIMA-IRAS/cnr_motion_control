#ifndef CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INTERFACE_IMPL__H
#define CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INTERFACE_IMPL__H

#include <cnr_logger/cnr_logger.h>
#include <eigen3/Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/node_handle.h>
#include <cnr_interpolator_interface/cnr_interpolator_interface.h>

namespace cnr_interpolator_interface
{

/**
 * @brief The InterpolatorInterface class
 */
template<class TRJ, class PNT, class IN, class OUT>
bool InterpolatorInterface<TRJ,PNT,IN,OUT>::initialize(cnr_logger::TraceLoggerPtr logger, ros::NodeHandle&  controller_nh, InterpolationTrajectoryPtr trj)
{
  if(!cnr_interpolator_interface::InterpolatorBase::initialize(logger, controller_nh, trj))
  {
    return false;
  }

  CNR_TRACE_START(this->logger());
  if(trj)
  {
    if(!std::dynamic_pointer_cast<TRJ const>(trj) )
    {
      CNR_RETURN_FALSE(this->logger(), "The input trajectory mismatch with the expected type. ABort.");
    }
  }

  CNR_RETURN_TRUE(this->logger());
}

template<class TRJ, class PNT, class IN, class OUT>
bool InterpolatorInterface<TRJ,PNT,IN,OUT>::setTrajectory(InterpolationTrajectoryPtr trj)
{
  CNR_TRACE_START(this->logger());
  if(!trj)
  {
    CNR_RETURN_FALSE(this->logger(), "Null pointer as input.");
  }
  
  if(!std::dynamic_pointer_cast<TRJ const>(trj) )
  {
    CNR_RETURN_FALSE(this->logger(), "The input trajectory mismatch with the expected type. ABort.");
  }

  if(!InterpolatorBase::setTrajectory(trj))
  {
    CNR_RETURN_FALSE(this->logger(), "Error in storing the pointer.");
  }
  CNR_RETURN_TRUE(this->logger());
}

template<class TRJ, class PNT, class IN, class OUT>
bool InterpolatorInterface<TRJ,PNT,IN,OUT>::appendToTrajectory(InterpolationPointConstPtr point)
{
  CNR_TRACE_START(this->logger());
  if(!InterpolatorBase::appendToTrajectory(point))
  {
    CNR_RETURN_FALSE(this->logger(), "Error in storing the pointer.");
  }
  
  if(!std::dynamic_pointer_cast<PNT const>(point) )
  {
    CNR_RETURN_FALSE(this->logger(), "The input trajectory mismatch with the expected type. ABort.");
  }
  this->m_trj->append(point);
  CNR_RETURN_TRUE(this->logger());
}

template<class TRJ, class PNT, class IN, class OUT>
const ros::Duration& InterpolatorInterface<TRJ,PNT,IN,OUT>::trjTime() const
{
  return InterpolatorBase::trjTime( );
}

template<class TRJ, class PNT, class IN, class OUT>
bool InterpolatorInterface<TRJ,PNT,IN,OUT>::interpolate(InterpolationInputConstPtr input, InterpolationOutputPtr output)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  if(!InterpolatorBase::interpolate(input,output))
  {
    CNR_RETURN_FALSE(this->logger(), "Error in storing the pointer.");
  }
  if(!std::dynamic_pointer_cast<IN const>(input) )
  {
    CNR_RETURN_FALSE(this->logger(), "The input mismatch with the expected type. Abort.");
  }
  if(!std::dynamic_pointer_cast<OUT>(output) )
  {
    CNR_RETURN_FALSE(this->logger(), "The output  mismatch with the expected type. Abort.");
  }
  this->m_in  = input;
  this->m_out = output;
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

template<class TRJ, class PNT, class IN, class OUT>
InterpolationPointConstPtr InterpolatorInterface<TRJ,PNT,IN,OUT>::getLastInterpolatedPoint() const
{
  assert(0);
}


}  // namespace cnr_interpolator_interface

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INTERFACE_IMPL__H
