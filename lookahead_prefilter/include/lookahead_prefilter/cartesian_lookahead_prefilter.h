#ifndef LOOKAHEAD_PREFILTER__LOOKAHEAD_PREFILTER__H
#define LOOKAHEAD_PREFILTER__LOOKAHEAD_PREFILTER__H

#include <cnr_logger/cnr_logger.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <rosdyn_core/spacevect_algebra.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/node_handle.h>
#include <cnr_interpolator_interface/cnr_interpolator_interface.h>


namespace thor
{


class CartesianLookaheadPrefilter : public cnr_interpolator_interface::InterpolatorInterface
{
public:

  CartesianLookaheadPrefilter() = default;
  virtual ~CartesianLookaheadPrefilter() = default;
  CartesianLookaheadPrefilter(const CartesianLookaheadPrefilter&) = delete;
  CartesianLookaheadPrefilter& operator=(const CartesianLookaheadPrefilter&) = delete;
  CartesianLookaheadPrefilter(CartesianLookaheadPrefilter&&) = delete;
  CartesianLookaheadPrefilter& operator=(CartesianLookaheadPrefilter&&) = delete;

  virtual bool initialize(cnr_logger::TraceLoggerPtr logger,
                          ros::NodeHandle& nh,
                          cnr_interpolator_interface::InterpolationTrajectoryPtr trj = nullptr);

  virtual bool setTrajectory(cnr_interpolator_interface::InterpolationTrajectoryPtr trj);
  virtual bool appendToTrajectory(cnr_interpolator_interface::InterpolationPointConstPtr point);
  virtual bool interpolate(cnr_interpolator_interface::InterpolationInputConstPtr input,
                           cnr_interpolator_interface::InterpolationOutputPtr     output);

  virtual const ros::Duration& trjTime() const;
  virtual cnr_interpolator_interface::InterpolationPointConstPtr getLastInterpolatedPoint();
  virtual cnr_interpolator_interface::InterpolationTrajectoryConstPtr getTrajectory();

private:
  cnr_interpolator_interface::CartesianTrajectoryPtr m_trj;
  cnr_interpolator_interface::CartesianPointPtr m_last_interpolated_point;

};

typedef std::shared_ptr<thor::CartesianLookaheadPrefilter> CartesianLookaheadPrefilterPtr;

}  // namespace thor

#endif  // LOOKAHEAD_PREFILTER__LOOKAHEAD_PREFILTER__H
