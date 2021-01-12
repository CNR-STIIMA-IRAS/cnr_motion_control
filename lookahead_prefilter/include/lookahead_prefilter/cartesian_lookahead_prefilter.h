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
                          InterpolationTrajectoryPtr  trj = nullptr) override;

  virtual bool setTrajectory(InterpolationTrajectoryPtr trj) override;
  virtual bool appendToTrajectory(InterpolationPointConstPtr point) override;
  virtual bool interpolate(InterpolationInputConstPtr input,
                           InterpolationOutputPtr output) override;
  virtual InterpolationPointConstPtr getLastInterpolatedPoint() const override;

private:
  CartesianPointPtr m_last_interpolated_point;

};

typedef std::shared_ptr<CartesianLookaheadPrefilter> CartesianLookaheadPrefilterPtr;

}  // namespace control
}  // namespace cnr

#endif  // LOOKAHEAD_PREFILTER__LOOKAHEAD_PREFILTER__H
