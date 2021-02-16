#ifndef LOOKAHEAD_PREFILTER__LOOKAHEAD_PREFILTER__H
#define LOOKAHEAD_PREFILTER__LOOKAHEAD_PREFILTER__H


#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <ros/node_handle.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <rosdyn_core/spacevect_algebra.h>

#include <cnr_logger/cnr_logger.h>
#include <cnr_interpolator_interface/cnr_interpolator_interface.h>

namespace cnr
{
namespace control
{

class CartesianLookaheadPrefilter : public CartesianInterpolatorInterface
{
public:

  CartesianLookaheadPrefilter() = default;
  virtual ~CartesianLookaheadPrefilter() = default;
  CartesianLookaheadPrefilter(const CartesianLookaheadPrefilter&) = delete;
  CartesianLookaheadPrefilter& operator=(const CartesianLookaheadPrefilter&) = delete;
  CartesianLookaheadPrefilter(CartesianLookaheadPrefilter&&) = delete;
  CartesianLookaheadPrefilter& operator=(CartesianLookaheadPrefilter&&) = delete;

  virtual bool initialize(cnr_logger::TraceLoggerPtr logger, 
                          ros::NodeHandle&           controller_nh,
                          InterpolatorTrajectoryPtr  trj = nullptr) override;

  virtual bool setTrajectory(InterpolatorTrajectoryPtr trj) override;
  virtual bool appendToTrajectory(InterpolatorPointConstPtr point) override;
  virtual bool interpolate(InterpolatorInputConstPtr input,
                           InterpolatorOutputPtr output) override;
  virtual InterpolatorPointConstPtr getLastInterpolatedPoint() const override;

private:
  CartesianInterpolatorPointPtr m_last_interpolated_point;

};

typedef std::shared_ptr<CartesianLookaheadPrefilter> CartesianLookaheadPrefilterPtr;

}  // namespace control
}  // namespace cnr

#endif  // LOOKAHEAD_PREFILTER__LOOKAHEAD_PREFILTER__H
