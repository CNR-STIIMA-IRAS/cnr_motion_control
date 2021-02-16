#ifndef THOR_PREFILTER__THOR_PREFILTER__H
#define THOR_PREFILTER__THOR_PREFILTER__H

#include <Eigen/Dense>
#include <ros/node_handle.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <cnr_interpolator_interface/cnr_interpolator_interface.h>

namespace cnr
{
namespace control
{


class ThorPrefilter : public JointInterpolatorInterface
{
protected:
  JointInterpolatorPointPtr m_last_interpolated_point;
  JointInterpolatorState m_state;
  unsigned int m_order;

public:

  ThorPrefilter() = default;
  virtual ~ThorPrefilter() = default;
  ThorPrefilter(const ThorPrefilter&) = delete;
  ThorPrefilter& operator=(const ThorPrefilter&) = delete;
  ThorPrefilter(ThorPrefilter&&) = delete;
  ThorPrefilter& operator=(ThorPrefilter&&) = delete;

  virtual bool initialize(cnr_logger::TraceLoggerPtr logger, ros::NodeHandle& nh,
                          InterpolatorTrajectoryPtr trj = nullptr) override;

  virtual bool interpolate(InterpolatorInputConstPtr input,
                           InterpolatorOutputPtr     output) override;

  virtual bool setTrajectory(InterpolatorTrajectoryPtr trj) override;
  virtual InterpolatorPointConstPtr getLastInterpolatedPoint() const override;

  void setSplineOrder(const unsigned int& order);
};

typedef std::shared_ptr<cnr::control::ThorPrefilter> ThorPrefilterPtr;
typedef std::shared_ptr<cnr::control::ThorPrefilter const > ThorPrefilterConstPtr;

}  // namespace control
}  // namespace cnr

#endif  // THOR_PREFILTER__H
