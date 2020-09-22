#include <lookahead_prefilter/cartesian_lookahead_prefilter.h>
#include <pluginlib/class_list_macros.h> // header for PLUGINLIB_EXPORT_CLASS. NOTE IT SHOULD STAY IN THE CPP FILE NOTE

PLUGINLIB_EXPORT_CLASS(thor::CartesianLookaheadPrefilter, cnr_interpolator_interface::InterpolatorInterface)

using cnr_interpolator_interface::InterpolatorInterface;

using cnr_interpolator_interface::CartesianPoint;
using cnr_interpolator_interface::CartesianInput;
using cnr_interpolator_interface::CartesianOutput;
using cnr_interpolator_interface::CartesianTrajectory;

using cnr_interpolator_interface::CartesianPointConstPtr;
using cnr_interpolator_interface::CartesianInputConstPtr;
using cnr_interpolator_interface::CartesianOutputPtr;
using cnr_interpolator_interface::CartesianTrajectoryPtr;

namespace thor
{

bool CartesianLookaheadPrefilter::initialize(cnr_logger::TraceLoggerPtr logger,
                                             ros::NodeHandle& nh,
                                             cnr_interpolator_interface::InterpolationTrajectoryPtr trj)
{
  if(!InterpolatorInterface::initialize(logger, nh, trj))
  {
    return false;
  }
  int spline_order = 0;
  if( !nh.getParam("spline_order", spline_order ))
  {
    CNR_ERROR(*m_logger, "The param 'spline_order' is missing. Default value superimposed to 0.");
    spline_order = 0;
  }
  return true;
}

bool CartesianLookaheadPrefilter::interpolate(cnr_interpolator_interface::InterpolationInputConstPtr input,
                                              cnr_interpolator_interface::InterpolationOutputPtr output)
{
  CartesianInputConstPtr in   = std::dynamic_pointer_cast<CartesianInput const>( input );
  CartesianOutputPtr     out  = std::dynamic_pointer_cast<CartesianOutput>     ( output );
  CartesianTrajectoryPtr ctrj = std::dynamic_pointer_cast<CartesianTrajectory> ( m_trj );

  if (!ctrj)
  {
    CNR_RETURN_FALSE(*m_logger, "Trajectory is not set");
  }

  if (!out)
  {
    CNR_RETURN_FALSE(*m_logger, "The dynamic cast failed, the function parameter seems to be not a CartesianOutputPtr");
  }

  std::vector< CartesianPoint >& trj = ctrj->trj;

  if (trj.size() == 0)
    return false;

  if ((in->time - trj.at(0).time_from_start).toSec() < 0)
  {
    out->pnt.x = trj.at(0).x;
    out->pnt.twist = Eigen::Vector6d::Zero();
    out->pnt.twistd = Eigen::Vector6d::Zero();
    return false;
  }

  if ((in->time - trj.back().time_from_start).toSec() >= 0)
  {
    out->pnt = trj.back();
    return true;
  }

  for (unsigned int iPnt = 1; iPnt < trj.size(); iPnt++)
  {
    if (((in->time - trj.at(iPnt).time_from_start).toSec() < 0) && ((in->time - trj.at(iPnt - 1).time_from_start).toSec() >= 0))
    {
      out->pnt.time_from_start = in->time;
      double delta_time = std::max(1.0e-6, (trj.at(iPnt).time_from_start - trj.at(iPnt - 1).time_from_start).toSec());
      double t          = (in->time - trj.at(iPnt - 1).time_from_start).toSec();
      double ratio      = t / delta_time;

      Eigen::Affine3d T_0_1 =trj.at(iPnt - 1).x;
      Eigen::Affine3d T_0_2 =trj.at(iPnt).x;

      // T_0_2 = Q_0 T_0_1 --> Q_0 = T_0_2 * T_0_1.inverse()
      Eigen::Affine3d Q_from_1_to_2_in_0 = T_0_2 * T_0_1.inverse();
      Eigen::Vector3d Dx_from_1_to_2_in_0 = Q_from_1_to_2_in_0.translation();
      Eigen::AngleAxisd aa_from_1_to_2_in_0( Q_from_1_to_2_in_0.linear() );

      Eigen::Vector3d Dx_from_1_to_t_in_0 = ratio * Dx_from_1_to_2_in_0;
      Eigen::AngleAxisd aa_from_1_to_t_in_0 = Eigen::AngleAxisd( ratio * aa_from_1_to_2_in_0.angle(), aa_from_1_to_2_in_0.axis());

      Eigen::Affine3d Q_from_1_to_t_in_0;
      Q_from_1_to_t_in_0.translation() = Dx_from_1_to_t_in_0;
      Q_from_1_to_t_in_0.linear() = aa_from_1_to_t_in_0.toRotationMatrix();

      out->pnt.x = Q_from_1_to_t_in_0 * T_0_1;
      out->pnt.twist.head(3) = in->override * Dx_from_1_to_2_in_0 / delta_time;
      out->pnt.twist.tail(3) = in->override * aa_from_1_to_2_in_0.axis() * aa_from_1_to_2_in_0.angle() / delta_time;
      out->pnt.twistd = Eigen::Vector6d::Zero();

      *m_last_interpolated_point = out->pnt;
      return true;
    }
  }
  return false;
}


bool CartesianLookaheadPrefilter::setTrajectory(cnr_interpolator_interface::InterpolationTrajectoryPtr trj)
{
  CNR_TRACE_START(*m_logger);
  if(!trj)
  {
    CNR_RETURN_FALSE(*m_logger, "Trajectory is not set");
  }
  if(m_trj)
    m_trj.reset();

  m_trj = std::dynamic_pointer_cast<CartesianTrajectory>(trj);
  if(m_trj)
  {
    CNR_RETURN_FALSE(*m_logger, "Error in casting the pointer. Abort.");
  }
  bool ret = !(m_trj->isEmpty());
  if(ret)
  {
    m_last_interpolated_point.reset( new CartesianPoint() );
    *m_last_interpolated_point =m_trj->trj.front();
  }
  CNR_RETURN_BOOL(*m_logger, ret);
}



bool CartesianLookaheadPrefilter::appendToTrajectory(cnr_interpolator_interface::InterpolationPointConstPtr point)
{
  CNR_TRACE_START(*m_logger);
  bool ret = m_trj->append(point);
  CNR_RETURN_BOOL(*m_logger, ret);
}

const ros::Duration& CartesianLookaheadPrefilter::trjTime() const
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


cnr_interpolator_interface::InterpolationPointConstPtr CartesianLookaheadPrefilter::getLastInterpolatedPoint()
{
  return m_last_interpolated_point;
}
cnr_interpolator_interface::InterpolationTrajectoryConstPtr CartesianLookaheadPrefilter::getTrajectory()
{
  return m_trj;
}


}
