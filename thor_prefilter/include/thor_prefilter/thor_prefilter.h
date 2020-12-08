#ifndef THOR_PREFILTER__THOR_PREFILTER__H
#define THOR_PREFILTER__THOR_PREFILTER__H

#include <Eigen/Dense>
#include <ros/node_handle.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <cnr_interpolator_interface/cnr_interpolator_interface.h>

namespace thor
{




class ThorPrefilter : public cnr_interpolator_interface::JointInterpolatorInterface
{
protected:
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
                          cnr_interpolator_interface::InterpolationTrajectoryPtr trj = nullptr) override;

  virtual bool interpolate(cnr_interpolator_interface::InterpolationInputConstPtr input,
                           cnr_interpolator_interface::InterpolationOutputPtr     output) override;

  virtual bool setTrajectory(cnr_interpolator_interface::InterpolationTrajectoryPtr trj) override;
  virtual cnr_interpolator_interface::InterpolationPointConstPtr getLastInterpolatedPoint() const override;

  void setSplineOrder(const unsigned int& order);
};

typedef std::shared_ptr<thor::ThorPrefilter> ThorPrefilterPtr;
typedef std::shared_ptr<thor::ThorPrefilter const > ThorPrefilterConstPtr;

}  // namespace thor

#endif  // THOR_PREFILTER__H
