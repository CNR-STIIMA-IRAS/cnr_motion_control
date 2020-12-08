#ifndef LOOKAHEAD_PREFILTER__LOOKAHEAD_PREFILTER__H
#define LOOKAHEAD_PREFILTER__LOOKAHEAD_PREFILTER__H


#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <ros/node_handle.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <rosdyn_core/spacevect_algebra.h>

#include <cnr_logger/cnr_logger.h>
#include <cnr_interpolator_interface/cnr_interpolator_interface.h>


namespace thor
{

class CartesianLookaheadPrefilter : public cnr_interpolator_interface::CartesianInterpolatorInterface
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
                          cnr_interpolator_interface::InterpolationTrajectoryPtr  trj = nullptr) override;

  virtual bool setTrajectory(cnr_interpolator_interface::InterpolationTrajectoryPtr trj) override;
  virtual bool appendToTrajectory(cnr_interpolator_interface::InterpolationPointConstPtr point) override;
  virtual bool interpolate(cnr_interpolator_interface::InterpolationInputConstPtr input, 
                           cnr_interpolator_interface::InterpolationOutputPtr output) override;
  virtual cnr_interpolator_interface::InterpolationPointConstPtr getLastInterpolatedPoint() const override;

private:
  cnr_interpolator_interface::CartesianPointPtr m_last_interpolated_point;

};

typedef std::shared_ptr<thor::CartesianLookaheadPrefilter> CartesianLookaheadPrefilterPtr;

}  // namespace thor

#endif  // LOOKAHEAD_PREFILTER__LOOKAHEAD_PREFILTER__H
