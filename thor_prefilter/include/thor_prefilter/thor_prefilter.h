#ifndef THOR_PREFILTER__THOR_PREFILTER__H
#define THOR_PREFILTER__THOR_PREFILTER__H

#include <eigen3/Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/node_handle.h>
#include <cnr_interpolator_interface/cnr_interpolator_interface.h>

namespace thor
{




class ThorPrefilter : public cnr_interpolator_interface::InterpolatorInterface
{
protected:
  cnr_interpolator_interface::JointTrajectoryPtr m_trj;
  cnr_interpolator_interface::JointPointPtr m_last_interpolated_point;
  cnr_interpolator_interface::JointState m_state;
  unsigned int m_order;

public:

  ThorPrefilter() = default;
  virtual ~ThorPrefilter() = default;
  ThorPrefilter(const ThorPrefilter&) = delete;
  ThorPrefilter& operator=(const ThorPrefilter&) = delete;
  ThorPrefilter(ThorPrefilter&&) = delete;
  ThorPrefilter& operator=(ThorPrefilter&&) = delete;

  virtual bool initialize(cnr_logger::TraceLoggerPtr logger, ros::NodeHandle& nh,
                          cnr_interpolator_interface::InterpolationTrajectoryPtr trj = nullptr);

  virtual bool interpolate(const ros::Duration& time, cnr_interpolator_interface::InterpolationOutputPtr output);

  virtual bool setTrajectory(cnr_interpolator_interface::InterpolationTrajectoryPtr trj);
  virtual bool appendToTrajectory(cnr_interpolator_interface::InterpolationPointConstPtr point);
  virtual const ros::Duration& trjTime() const;
  virtual cnr_interpolator_interface::InterpolationPointConstPtr getLastInterpolatedPoint();
  virtual cnr_interpolator_interface::InterpolationTrajectoryConstPtr getTrajectory();

  void setSplineOrder(const unsigned int& order);
};

typedef std::shared_ptr<thor::ThorPrefilter> ThorPrefilterPtr;
typedef const std::shared_ptr<thor::ThorPrefilter const > ThorPrefilterConstPtr;

}  // namespace thor

#endif  // THOR_PREFILTER__H
