#include <vector>
#include <cnr_logger/cnr_logger.h>
#include <cnr_interpolator_interface/cnr_interpolator_interface.h>
#include <lookahead_prefilter/cartesian_lookahead_prefilter.h>
#include <pluginlib/class_list_macros.h> // header for PLUGINLIB_EXPORT_CLASS. NOTE IT SHOULD STAY IN THE CPP FILE NOTE

PLUGINLIB_EXPORT_CLASS(cnr::control::CartesianLookaheadPrefilter, cnr::control::InterpolatorBase)


namespace cnr
{

namespace control
{

bool CartesianLookaheadPrefilter::initialize(cnr_logger::TraceLoggerPtr logger,
                                             ros::NodeHandle& nh,
                                             InterpolatorTrajectoryPtr trj)
{
  if(!CartesianInterpolatorInterface::initialize(logger, nh, trj))
  {
    return false;
  }
  int spline_order = 0;
  if( !nh.getParam("spline_order", spline_order ))
  {
    CNR_ERROR(m_logger, "The param 'spline_order' is missing. Default value superimposed to 0.");
    spline_order = 0;
  }
  return true;
}

bool CartesianLookaheadPrefilter::interpolate(InterpolatorInputConstPtr input,
                                              InterpolatorOutputPtr output)
{
  CNR_TRACE_START_THROTTLE(this->logger(), 5.0);
  if(!CartesianInterpolatorInterface::interpolate(input,output))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  std::vector< CartesianInterpolatorPoint >& points = trj()->trj;

  if(points.size() == 0)
  {
    CNR_RETURN_FALSE(this->logger());
  }

  if((in()->time() - points.at(0).time_from_start).toSec() < 0)
  {
    out()->pnt.x = points.at(0).x;
    out()->pnt.twist = Eigen::Vector6d::Zero();
    out()->pnt.twistd = Eigen::Vector6d::Zero();
    CNR_RETURN_FALSE(this->logger());
  }

  if((in()->time() - points.back().time_from_start).toSec() >= 0)
  {
    out()->pnt = points.back();
  }

  for(unsigned int iPnt = 1; iPnt < points.size(); iPnt++)
  {
    if(((in()->time() - points.at(iPnt).time_from_start).toSec() < 0)
    && ((in()->time() - points.at(iPnt - 1).time_from_start).toSec() >= 0))
    {
      out()->pnt.time_from_start = in()->time();
      double delta_time = std::max(1.0e-6,(points.at(iPnt).time_from_start - points.at(iPnt - 1).time_from_start).toSec());
      double t          =(in()->time() - points.at(iPnt - 1).time_from_start).toSec();
      double ratio      = t / delta_time;

      Eigen::Affine3d T_0_1 =points.at(iPnt - 1).x;
      Eigen::Affine3d T_0_2 =points.at(iPnt).x;

      // T_0_2 = Q_0 T_0_1 --> Q_0 = T_0_2 * T_0_1.inverse()
      Eigen::Affine3d Q_from_1_to_2_in_0 = T_0_2 * T_0_1.inverse();
      Eigen::Vector3d Dx_from_1_to_2_in_0 = Q_from_1_to_2_in_0.translation();
      Eigen::AngleAxisd aa_from_1_to_2_in_0( Q_from_1_to_2_in_0.linear() );

      Eigen::Vector3d Dx_from_1_to_t_in_0 = ratio * Dx_from_1_to_2_in_0;
      Eigen::AngleAxisd aa_from_1_to_t_in_0 = Eigen::AngleAxisd( ratio * aa_from_1_to_2_in_0.angle(), aa_from_1_to_2_in_0.axis());

      Eigen::Affine3d Q_from_1_to_t_in_0;
      Q_from_1_to_t_in_0.translation() = Dx_from_1_to_t_in_0;
      Q_from_1_to_t_in_0.linear() = aa_from_1_to_t_in_0.toRotationMatrix();

      out()->pnt.x = Q_from_1_to_t_in_0 * T_0_1;
      out()->pnt.twist.head(3) = in()->override() * Dx_from_1_to_2_in_0 / delta_time;
      out()->pnt.twist.tail(3) = in()->override() * aa_from_1_to_2_in_0.axis() * aa_from_1_to_2_in_0.angle() / delta_time;
      out()->pnt.twistd = Eigen::Vector6d::Zero();

      *m_last_interpolated_point = out()->pnt;
      CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
    }
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}


bool CartesianLookaheadPrefilter::setTrajectory(InterpolatorTrajectoryPtr _trj)
{
  CNR_TRACE_START(m_logger);
  if(!CartesianInterpolatorInterface::setTrajectory(_trj))
  {
    CNR_RETURN_FALSE(m_logger, "Trajectory is not set");
  }

  m_last_interpolated_point.reset( new CartesianInterpolatorPoint() );
  *m_last_interpolated_point =trj()->trj.front();
  CNR_RETURN_TRUE(m_logger);
}


bool CartesianLookaheadPrefilter::appendToTrajectory(InterpolatorPointConstPtr point)
{
  CNR_TRACE_START(m_logger);
  if(!CartesianInterpolatorInterface::appendToTrajectory(point))
  {
    CNR_RETURN_FALSE(m_logger, "Error in appending the points");
  }
  CNR_RETURN_TRUE(m_logger);
}

InterpolatorPointConstPtr CartesianLookaheadPrefilter::getLastInterpolatedPoint() const
{
  return m_last_interpolated_point;
}


}  // namespace control
}  // namespace cnr
