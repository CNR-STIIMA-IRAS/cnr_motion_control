#ifndef CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_TRAJECTORY__H
#define CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_TRAJECTORY__H

#include <cnr_logger/cnr_logger.h>
#include <Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <cnr_interpolator_interface/cnr_interpolator_point.h>

namespace cnr
{
namespace control
{


/**
 * @brief The InterpolatorTrajectory struct
 */
struct InterpolatorTrajectory
{
  InterpolatorTrajectory() = default;
  virtual ~InterpolatorTrajectory() = default;
  InterpolatorTrajectory(const InterpolatorTrajectory&) = delete;
  InterpolatorTrajectory& operator=(const InterpolatorTrajectory&) = delete;
  InterpolatorTrajectory(InterpolatorTrajectory&&) = delete;
  InterpolatorTrajectory& operator=(InterpolatorTrajectory&&) = delete;

  virtual bool isEmpty() const = 0;
  virtual bool append(InterpolatorPointConstPtr point) = 0;
  virtual const ros::Duration& trjTime() const
  {
    static ros::Duration default_duration(0);
    return default_duration;
  }
};
typedef std::shared_ptr<InterpolatorTrajectory> InterpolatorTrajectoryPtr;
typedef std::shared_ptr<InterpolatorTrajectory const > InterpolatorTrajectoryConstPtr;

/**
 * @brief The JointTrajectory struct
 */
struct JointTrajectory : public InterpolatorTrajectory
{
  JointTrajectory() { trj.reset(new trajectory_msgs::JointTrajectory()); }
  virtual ~JointTrajectory() = default;
  JointTrajectory(const JointTrajectory&) = delete;
  JointTrajectory& operator=(const JointTrajectory&) = delete;
  JointTrajectory(JointTrajectory&&) = delete;
  JointTrajectory& operator=(JointTrajectory&&) = delete;
  JointTrajectory(const std::vector<trajectory_msgs::JointTrajectoryPoint>& pnts)
  {
    this->trj.reset( new trajectory_msgs::JointTrajectory());
    for(auto const p : pnts) this->trj->points.push_back(p);
  }

  virtual bool isEmpty() const
  {
    return trj ? trj->points.size() == 0 : false;
  }

  virtual bool append(InterpolatorPointConstPtr point)
  {
    if(trj && point)
    {
      JointInterpolatorPointConstPtr pnt = std::dynamic_pointer_cast<JointInterpolatorPoint const>(point);
      trj->points.push_back(pnt->pnt);
    }
    return isEmpty();
  }

  virtual const ros::Duration& trjTime() const
  {
    return trj ? trj->points.back().time_from_start : InterpolatorTrajectory::trjTime();
  }

  trajectory_msgs::JointTrajectoryPtr trj;
};

typedef std::shared_ptr<JointTrajectory> JointTrajectoryPtr;
typedef std::shared_ptr<JointTrajectory const > JointTrajectoryConstPtr;




/**
 * @brief The CartesianTrajectory struct
 */
struct CartesianTrajectory : public InterpolatorTrajectory
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CartesianTrajectory() { trj.clear(); }
  virtual ~CartesianTrajectory() = default;
  CartesianTrajectory(const CartesianTrajectory&) = delete;
  CartesianTrajectory& operator=(const CartesianTrajectory&) = delete;
  CartesianTrajectory(CartesianTrajectory&&) = delete;
  CartesianTrajectory& operator=(CartesianTrajectory&&) = delete;

  virtual bool isEmpty() const { return trj.size() > 0; }
  virtual bool append( InterpolatorPointConstPtr point)
  {
    if(point)
    {
      CartesianInterpolatorPointConstPtr pnt = std::dynamic_pointer_cast<CartesianInterpolatorPoint const>(point);
      trj.push_back( *pnt );
    }
    return isEmpty();
  }

  virtual const ros::Duration& trjTime() const
  {
    return trj.size()>0 ? trj.back().time_from_start : InterpolatorTrajectory::trjTime();
  }

  std::vector<CartesianInterpolatorPoint> trj;
};
typedef std::shared_ptr<CartesianTrajectory> CartesianTrajectoryPtr;
typedef std::shared_ptr<CartesianTrajectory const > CartesianTrajectoryConstPtr;



}  // namespace control
}  // namespace cnr

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INPUTS__H
