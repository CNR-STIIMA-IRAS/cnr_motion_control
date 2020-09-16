#include <thor_prefilter/thor_prefilter.h>
#include <pluginlib/class_list_macros.h> // header for PLUGINLIB_EXPORT_CLASS. NOTE IT SHOULD STAY IN THE CPP FILE NOTE

PLUGINLIB_EXPORT_CLASS(thor::ThorPrefilter, cnr_interpolator_interface::InterpolatorInterface)

using cnr_interpolator_interface::InterpolatorInterface;
using cnr_interpolator_interface::InterpolationTrajectoryPtr;

using cnr_interpolator_interface::JointPoint;
using cnr_interpolator_interface::JointInput;
using cnr_interpolator_interface::JointOutput;
using cnr_interpolator_interface::JointTrajectory;

using cnr_interpolator_interface::JointPointPtr;
using cnr_interpolator_interface::JointInputPtr;
using cnr_interpolator_interface::JointOutputPtr;
using cnr_interpolator_interface::JointTrajectoryPtr;

using cnr_interpolator_interface::JointPointConstPtr;
using cnr_interpolator_interface::JointInputConstPtr;
using cnr_interpolator_interface::JointOutputConstPtr;
using cnr_interpolator_interface::JointTrajectoryConstPtr;

namespace thor
{

/**
 * @brief ThorPrefilter::initialize
 * @param logger
 * @param nh
 * @param trj
 * @return
 */
bool ThorPrefilter::initialize(cnr_logger::TraceLoggerPtr logger, ros::NodeHandle& nh, InterpolationTrajectoryPtr trj)
{
  if(!InterpolatorInterface::initialize(logger, nh, trj) )
  {
    return false;
  }

  CNR_TRACE_START(*m_logger);
  int spline_order = 0;
  if( !nh.getParam("spline_order", spline_order ))
  {
    CNR_WARN(*m_logger, "The param 'spline_order' is missing. Default value superimposed to 0.");
    spline_order = 0;
  }
  setSplineOrder(spline_order);
  CNR_RETURN_TRUE(*m_logger);

}

void ThorPrefilter::setSplineOrder(const unsigned int& order)
{
  if(order < 5)
  {
    m_order = order;
  }
  else
  {
    CNR_WARN(*m_logger, "Interpolation order " << order << "is out of limits, it should be less or equal to 4");
  }
}

bool ThorPrefilter::setTrajectory(InterpolationTrajectoryPtr trj)
{
  CNR_TRACE_START(*m_logger);
  if(!trj)
  {
    CNR_RETURN_FALSE(*m_logger, "Trajectory is not set");
  }
  if(m_trj)
    m_trj.reset();

  m_trj = std::dynamic_pointer_cast<JointTrajectory>(trj);
  if(m_trj)
  {
    CNR_RETURN_FALSE(*m_logger, "Error in casting the pointer. Abort.");
  }
  bool ret = !(m_trj->isEmpty());
  if(ret)
  {
    m_last_interpolated_point.reset( new JointPoint() );
    m_last_interpolated_point->pnt =m_trj->trj->points.front();
  }
  CNR_RETURN_BOOL(*m_logger, ret);
}



bool ThorPrefilter::appendToTrajectory(cnr_interpolator_interface::InterpolationPointConstPtr point)
{
  CNR_TRACE_START(*m_logger);
  bool ret = m_trj->append(point);
  CNR_RETURN_BOOL(*m_logger, ret);
}

const ros::Duration& ThorPrefilter::trjTime() const
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

bool ThorPrefilter::interpolate(const ros::Duration&                               time,
                                cnr_interpolator_interface::InterpolationOutputPtr output)
{
  JointOutputPtr     out     = std::dynamic_pointer_cast<JointOutput>     ( output );
  if (!out)
  {
    CNR_RETURN_FALSE(*m_logger, "The dynamic cast failed, the function parameter seems to be not a JointOutputPtr");
  }

  if (!m_trj)
  {
    CNR_RETURN_FALSE(*m_logger, "The dynamic cast failed, the function parameter seems to be not a JointTrajectoryPtr");
  }

  trajectory_msgs::JointTrajectoryPtr trj = m_trj->trj;
  if (!trj)
  {
    CNR_ERROR(*m_logger, "Trajectory is not set");
    return false;
  }


  if (trj->points.size() == 0)
    return false;

  if ((time - trj->points.at(0).time_from_start).toSec() < 0)
  {
    out->pnt = trj->points.at(0);
    out->pnt.effort.resize(trj->points.at(0).positions.size(), 0);
    return false;
  }

  if ((time - trj->points.back().time_from_start).toSec() >= 0)
  {
    out->pnt = trj->points.back();
    out->pnt.effort.resize(trj->points.back().positions.size(), 0);
    return true;
  }

  for (unsigned int iPnt = 1; iPnt < trj->points.size(); iPnt++)
  {
    if (((time - trj->points.at(iPnt).time_from_start).toSec() < 0) && ((time - trj->points.at(iPnt-1).time_from_start).toSec() >= 0))
    {
      unsigned int nAx = trj->points.at(iPnt).positions.size();
      out->pnt.positions.resize(nAx, 0);
      out->pnt.velocities.resize(nAx, 0);
      out->pnt.accelerations.resize(nAx, 0);
      out->pnt.effort.resize(nAx, 0);
      out->pnt.time_from_start = time;
      double delta_time = std::max(1.0e-6, (trj->points.at(iPnt).time_from_start - trj->points.at(iPnt-1).time_from_start).toSec());
      double t = (time - trj->points.at(iPnt-1).time_from_start).toSec();
      double ratio = t / delta_time;
      for (unsigned int iAx = 0; iAx < nAx; iAx++)
      {
        //spline
        if (m_order == 0)
        {
          out->pnt.positions.at(iAx)  = trj->points.at(iPnt-1).positions.at(iAx)
                                      + ratio * (trj->points.at(iPnt).positions.at(iAx) - trj->points.at(iPnt-1).positions.at(iAx));
          out->pnt.velocities.at(iAx) = (trj->points.at(iPnt).positions.at(iAx) - trj->points.at(iPnt-1).positions.at(iAx)) / delta_time;
        }
        else if (m_order == 1)
        {
          double& p0_1 = trj->points.at(iPnt-1).positions.at(iAx);
          double& p0_2 = trj->points.at(iPnt-1).velocities.at(iAx);
          double& pf_1 = trj->points.at(iPnt).positions.at(iAx);
          double& pf_2 = trj->points.at(iPnt).velocities.at(iAx);

          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = -1.0 / (delta_time * delta_time) * (p0_1 * 3.0 - pf_1 * 3.0 + delta_time * p0_2 * 2.0 + delta_time * pf_2);
          double c4 = 1.0 / (delta_time * delta_time * delta_time) * (p0_1 * 2.0 - pf_1 * 2.0 + delta_time * p0_2 + delta_time * pf_2);

          out->pnt.positions.at(iAx)     = c1 + c2 * t + c3 * (t * t) + c4 * (t * t * t);
          out->pnt.velocities.at(iAx)    = c2 + c3 * t * 2.0 + c4 * (t * t) * 3.0;
          out->pnt.accelerations.at(iAx) = c3 * 2.0 + c4 * t * 6.0;

        }
        else if (m_order == 2)
        {
          double& p0_1 = trj->points.at(iPnt-1).positions.at(iAx);
          double& p0_2 = trj->points.at(iPnt-1).velocities.at(iAx);
          double& p0_3 = trj->points.at(iPnt-1).accelerations.at(iAx);
          double& pf_1 = trj->points.at(iPnt).positions.at(iAx);
          double& pf_2 = trj->points.at(iPnt).velocities.at(iAx);
          double& pf_3 = trj->points.at(iPnt).accelerations.at(iAx);

          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = p0_3 * (1.0 / 2.0);
          double c4 = 1.0 / (delta_time * delta_time * delta_time) * (p0_1 * 2.0E1 - pf_1 * 2.0E1 + delta_time * p0_2 * 1.2E1 + delta_time * pf_2 * 8.0 + (delta_time * delta_time) * p0_3 * 3.0 - (delta_time * delta_time) * pf_3) * (-1.0 / 2.0);
          double c5 = 1.0 / (delta_time * delta_time * delta_time * delta_time) * (p0_1 * 3.0E1 - pf_1 * 3.0E1 + delta_time * p0_2 * 1.6E1 + delta_time * pf_2 * 1.4E1 + (delta_time * delta_time) * p0_3 * 3.0 - (delta_time * delta_time) * pf_3 * 2.0) * (1.0 / 2.0);
          double c6 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time) * (p0_1 * 1.2E1 - pf_1 * 1.2E1 + delta_time * p0_2 * 6.0 + delta_time * pf_2 * 6.0 + (delta_time * delta_time) * p0_3 - (delta_time * delta_time) * pf_3) * (-1.0 / 2.0);


          out->pnt.positions.at(iAx)     = c1 + c2 * t + c3 * (t * t) + c4 * (t * t * t) + c5 * (t * t * t * t) + c6 * (t * t * t * t * t);
          out->pnt.velocities.at(iAx)    = c2 + c3 * t * 2.0 + c4 * (t * t) * 3.0 + c5 * (t * t * t) * 4.0 + c6 * (t * t * t * t) * 5.0;
          out->pnt.accelerations.at(iAx) = c3 * 2.0 + c4 * t * 6.0 + c5 * (t * t) * 1.2E1 + c6 * (t * t * t) * 2.0E1;
        }
        else if (m_order == 3)
        {
          double& p0_1 = trj->points.at(iPnt-1).positions.at(iAx);
          double& p0_2 = trj->points.at(iPnt-1).velocities.at(iAx);
          double& p0_3 = trj->points.at(iPnt-1).accelerations.at(iAx);
          double& pf_1 = trj->points.at(iPnt).positions.at(iAx);
          double& pf_2 = trj->points.at(iPnt).velocities.at(iAx);
          double& pf_3 = trj->points.at(iPnt).accelerations.at(iAx);

          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = p0_3 * (1.0 / 2.0);
          double c4 = 0.0;
          double c5 = 1.0 / (delta_time * delta_time * delta_time * delta_time) * (p0_1 * 1.4E1 - pf_1 * 1.4E1 + delta_time * p0_2 * 8.0 + delta_time * pf_2 * 6.0 + (delta_time * delta_time) * p0_3 * 2.0 - (delta_time * delta_time) * pf_3) * (-5.0 / 2.0);
          double c6 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time) * (p0_1 * 8.4E1 - pf_1 * 8.4E1 + delta_time * p0_2 * 4.5E1 + delta_time * pf_2 * 3.9E1 + (delta_time * delta_time) * p0_3 * 1.0E1 - (delta_time * delta_time) * pf_3 * 7.0);
          double c7 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time * delta_time) * (p0_1 * 1.4E2 - pf_1 * 1.4E2 + delta_time * p0_2 * 7.2E1 + delta_time * pf_2 * 6.8E1 + (delta_time * delta_time) * p0_3 * 1.5E1 - (delta_time * delta_time) * pf_3 * 1.3E1) * (-1.0 / 2.0);
          double c8 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time * delta_time * delta_time) * (p0_1 * 1.0E1 - pf_1 * 1.0E1 + delta_time * p0_2 * 5.0 + delta_time * pf_2 * 5.0 + (delta_time * delta_time) * p0_3 - (delta_time * delta_time) * pf_3) * 2.0;


          out->pnt.positions.at(iAx)     = c1 + c2 * t + c3 * (t * t) + c4 * (t * t * t) + c5 * (t * t * t * t) + c6 * (t * t * t * t * t) + c7 * (t * t * t * t * t * t) + c8 * (t * t * t * t * t * t * t);
          out->pnt.velocities.at(iAx)    = c2 + c3 * t * 2.0 + c4 * (t * t) * 3.0 + c5 * (t * t * t) * 4.0 + c6 * (t * t * t * t) * 5.0 + c7 * (t * t * t * t * t) * 6.0 + c8 * (t * t * t * t * t * t) * 7.0;
          out->pnt.accelerations.at(iAx) = c3 * 2.0 + c4 * t * 6.0 + c5 * (t * t) * 1.2E1 + c6 * (t * t * t) * 2.0E1 + c7 * (t * t * t * t) * 3.0E1 + c8 * (t * t * t * t * t) * 4.2E1;
        }
        else if (m_order == 4)
        {
          double& p0_1 = trj->points.at(iPnt-1).positions.at(iAx);
          double& p0_2 = trj->points.at(iPnt-1).velocities.at(iAx);
          double& p0_3 = trj->points.at(iPnt-1).accelerations.at(iAx);
          double& pf_1 = trj->points.at(iPnt).positions.at(iAx);
          double& pf_2 = trj->points.at(iPnt).velocities.at(iAx);
          double& pf_3 = trj->points.at(iPnt).accelerations.at(iAx);

          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = p0_3 * (1.0 / 2.0);
          double c4 = 0.0;
          double c5 = 0.0;
          double c6 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time) * (p0_1 * 3.6E1 - pf_1 * 3.6E1 + delta_time * p0_2 * 2.0E1 + delta_time * pf_2 * 1.6E1 + (delta_time * delta_time) * p0_3 * 5.0 - (delta_time * delta_time) * pf_3 * 3.0) * (-7.0 / 2.0);
          double c7 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time * delta_time) * (p0_1 * 1.2E2 - pf_1 * 1.2E2 + delta_time * p0_2 * 6.4E1 + delta_time * pf_2 * 5.6E1 + (delta_time * delta_time) * p0_3 * 1.5E1 - (delta_time * delta_time) * pf_3 * 1.1E1) * (7.0 / 2.0);
          double c8 = -1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time * delta_time * delta_time) * (p0_1 * 5.4E2 - pf_1 * 5.4E2 + delta_time * p0_2 * 2.8E2 + delta_time * pf_2 * 2.6E2 + (delta_time * delta_time) * p0_3 * 6.3E1 - (delta_time * delta_time) * pf_3 * 5.3E1);
          double c9 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time * delta_time * delta_time * delta_time) * (p0_1 * 1.26E2 - pf_1 * 1.26E2 + delta_time * p0_2 * 6.4E1 + delta_time * pf_2 * 6.2E1 + (delta_time * delta_time) * p0_3 * 1.4E1 - (delta_time * delta_time) * pf_3 * 1.3E1) * (5.0 / 2.0);
          double c10 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time * delta_time * delta_time * delta_time * delta_time) * (p0_1 * 2.8E1 - pf_1 * 2.8E1 + delta_time * p0_2 * 1.4E1 + delta_time * pf_2 * 1.4E1 + (delta_time * delta_time) * p0_3 * 3.0 - (delta_time * delta_time) * pf_3 * 3.0) * (-5.0 / 2.0);


          out->pnt.positions.at(iAx)     = c1 + c2 * t + c3 * (t * t) + c4 * (t * t * t) + c5 * (t * t * t * t) + c6 * (t * t * t * t * t) + c7 * (t * t * t * t * t * t) + c8 * (t * t * t * t * t * t * t) + c9 * (t * t * t * t * t * t * t * t) + c10 * (t * t * t * t * t * t * t * t * t);
          out->pnt.velocities.at(iAx)    = c2 + c3 * t * 2.0 + c4 * (t * t) * 3.0 + c5 * (t * t * t) * 4.0 + c6 * (t * t * t * t) * 5.0 + c7 * (t * t * t * t * t) * 6.0 + c8 * (t * t * t * t * t * t) * 7.0 + c9 * (t * t * t * t * t * t * t) * 8.0 + c10 * (t * t * t * t * t * t * t * t) * 9.0;
          out->pnt.accelerations.at(iAx) = c3 * 2.0 + c4 * t * 6.0 + c5 * (t * t) * 1.2E1 + c6 * (t * t * t) * 2.0E1 + c7 * (t * t * t * t) * 3.0E1 + c8 * (t * t * t * t * t) * 4.2E1 + c9 * (t * t * t * t * t * t) * 5.6E1 + c10 * (t * t * t * t * t * t * t) * 7.2E1;
        }
      }
      m_last_interpolated_point->pnt = out->pnt;
      break;
    }
  }
  return true;
}

cnr_interpolator_interface::InterpolationPointConstPtr ThorPrefilter::getLastInterpolatedPoint()
{
  return m_last_interpolated_point;
}
cnr_interpolator_interface::InterpolationTrajectoryConstPtr ThorPrefilter::getTrajectory()
{
  return m_trj;
}


}
