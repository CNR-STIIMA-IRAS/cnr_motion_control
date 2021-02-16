#ifndef CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INTERFACE__H
#define CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INTERFACE__H

#include <cnr_logger/cnr_logger.h>
#include <Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/node_handle.h>
#include <cnr_interpolator_interface/cnr_interpolator_state.h>
#include <cnr_interpolator_interface/cnr_interpolator_inputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_outputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_point.h>
#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>
#include <cnr_interpolator_interface/internal/cnr_interpolator_base.h>

namespace cnr
{
namespace control
{
/**
 * @brief The InterpolatorInterface class
 */
template<class TRJ, class PNT, class IN, class OUT>
class InterpolatorInterface : public InterpolatorBase
{
protected:

  std::shared_ptr<TRJ const>  trj( ) const { return std::dynamic_pointer_cast<TRJ const>(m_trj); }
  std::shared_ptr<TRJ      >  trj( )       { return std::dynamic_pointer_cast<TRJ      >(m_trj); }
  std::shared_ptr<IN  const>  in ( ) const { return std::dynamic_pointer_cast<IN  const>(m_in ); }
  //std::shared_ptr<IN       >  in ( )       { return std::dynamic_pointer_cast<IN       >(m_in ); }
  std::shared_ptr<OUT const>  out( ) const { return std::dynamic_pointer_cast<OUT const>(m_out); }
  std::shared_ptr<OUT      >  out( )       { return std::dynamic_pointer_cast<OUT      >(m_out); }

public:
  InterpolatorInterface() = default;
  virtual ~InterpolatorInterface() = default;
  InterpolatorInterface(const InterpolatorInterface&) = delete;
  InterpolatorInterface& operator=(const InterpolatorInterface&) = delete;
  InterpolatorInterface(InterpolatorInterface&&) = delete;
  InterpolatorInterface& operator=(InterpolatorInterface&&) = delete;

  virtual bool initialize(cnr_logger::TraceLoggerPtr logger, 
                          ros::NodeHandle&           controller_nh,
                          InterpolatorTrajectoryPtr  trj = nullptr) override;

  virtual bool setTrajectory(InterpolatorTrajectoryPtr trj) override;
  virtual bool appendToTrajectory(InterpolatorPointConstPtr point) override;
  virtual const ros::Duration& trjTime() const override;
  virtual bool interpolate(InterpolatorInputConstPtr input, InterpolatorOutputPtr output) override;
  virtual InterpolatorPointConstPtr getLastInterpolatedPoint() const override;
};

typedef InterpolatorInterface<JointTrajectory,
                              JointInterpolatorPoint,
                              JointInterpolatorInput,
                              JointInterpolatorOutput >  JointInterpolatorInterface;

typedef std::shared_ptr<JointInterpolatorInterface> JointInterpolatorInterfacePtr;
typedef std::shared_ptr<JointInterpolatorInterface const> JointInterpolatorInterfaceConstPtr;

typedef InterpolatorInterface<CartesianTrajectory,
                              CartesianInterpolatorPoint,
                              CartesianInterpolatorInput,
                              CartesianInterpolatorOutput >  CartesianInterpolatorInterface;

typedef std::shared_ptr<CartesianInterpolatorInterface> CartesianInterpolatorBasePtr;
typedef std::shared_ptr<CartesianInterpolatorInterface const> CartesianInterpolatorInterfaceConstPtr;

}  // namespace cnr_interpolator_interface
}

#include <cnr_interpolator_interface/internal/cnr_interpolator_interface_impl.h>

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INTERFACE__H
