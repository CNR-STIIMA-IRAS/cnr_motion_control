#ifndef CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INTERFACE__H
#define CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INTERFACE__H

#include <cnr_logger/cnr_logger.h>
#include <eigen3/Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/node_handle.h>
#include <cnr_interpolator_interface/cnr_interpolator_state.h>
#include <cnr_interpolator_interface/cnr_interpolator_inputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_outputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_point.h>
#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>
#include <cnr_interpolator_interface/internal/cnr_interpolator_base.h>

namespace cnr_interpolator_interface
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
                          ros::NodeHandle&            controller_nh,
                          InterpolationTrajectoryPtr  trj = nullptr) override;

  virtual bool setTrajectory(InterpolationTrajectoryPtr trj) override;
  virtual bool appendToTrajectory(InterpolationPointConstPtr point) override;
  virtual const ros::Duration& trjTime() const override;
  virtual bool interpolate(InterpolationInputConstPtr input, InterpolationOutputPtr output) override;
  virtual InterpolationPointConstPtr getLastInterpolatedPoint() const override;
};

typedef InterpolatorInterface<cnr_interpolator_interface::JointTrajectory,
                              cnr_interpolator_interface::JointPoint,
                              cnr_interpolator_interface::JointInput,
                              cnr_interpolator_interface::JointOutput >  JointInterpolatorInterface;

typedef std::shared_ptr<cnr_interpolator_interface::JointInterpolatorInterface> JointInterpolatorBasePtr;
typedef std::shared_ptr<cnr_interpolator_interface::JointInterpolatorInterface const> JointInterpolatorInterfaceConstPtr;

typedef InterpolatorInterface<cnr_interpolator_interface::CartesianTrajectory,
                              cnr_interpolator_interface::CartesianPoint,
                              cnr_interpolator_interface::CartesianInput,
                              cnr_interpolator_interface::CartesianOutput >  CartesianInterpolatorInterface;

typedef std::shared_ptr<cnr_interpolator_interface::CartesianInterpolatorInterface> CartesianInterpolatorBasePtr;
typedef std::shared_ptr<cnr_interpolator_interface::CartesianInterpolatorInterface const> CartesianInterpolatorInterfaceConstPtr;

}  // namespace cnr_interpolator_interface

#include <cnr_interpolator_interface/internal/cnr_interpolator_interface_impl.h>

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INTERFACE__H
