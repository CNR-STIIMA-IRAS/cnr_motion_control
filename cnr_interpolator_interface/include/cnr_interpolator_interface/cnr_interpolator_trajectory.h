#ifndef CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_TRAJECTORY__H
#define CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_TRAJECTORY__H

#include <cnr_logger/cnr_logger.h>
#include <eigen3/Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <cnr_interpolator_interface/cnr_interpolator_point.h>

namespace cnr_interpolator_interface
{

/**
 * @brief The InterpolationTrajectory struct
 */
struct InterpolationTrajectory
{
  InterpolationTrajectory() = default;
  virtual ~InterpolationTrajectory() = default;
  InterpolationTrajectory(const InterpolationTrajectory&) = delete;
  InterpolationTrajectory& operator=(const InterpolationTrajectory&) = delete;
  InterpolationTrajectory(InterpolationTrajectory&&) = delete;
  InterpolationTrajectory& operator=(InterpolationTrajectory&&) = delete;

  virtual bool isEmpty() const = 0;
  virtual bool append( InterpolationPointConstPtr point) = 0;
  virtual const ros::Duration& trjTime() const
  {
    static ros::Duration default_duration(0);
    return default_duration;
  }
};
typedef std::shared_ptr<InterpolationTrajectory> InterpolationTrajectoryPtr;
typedef std::shared_ptr<InterpolationTrajectory const > InterpolationTrajectoryConstPtr;

/**
 * @brief The JointTrajectory struct
 */
struct JointTrajectory : public cnr_interpolator_interface::InterpolationTrajectory
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

  virtual bool append(cnr_interpolator_interface::InterpolationPointConstPtr point)
  {
    if(trj && point)
    {
      JointPointConstPtr pnt = std::dynamic_pointer_cast< JointPoint const >(point);
      trj->points.push_back(pnt->pnt);
    }
    return isEmpty();
  }

  virtual const ros::Duration& trjTime() const
  {
    return trj ? trj->points.back().time_from_start : InterpolationTrajectory::trjTime();
  }

  trajectory_msgs::JointTrajectoryPtr trj;
};

typedef std::shared_ptr<JointTrajectory> JointTrajectoryPtr;
typedef std::shared_ptr<JointTrajectory const > JointTrajectoryConstPtr;




/**
 * @brief The CartesianTrajectory struct
 */
struct CartesianTrajectory : public cnr_interpolator_interface::InterpolationTrajectory
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CartesianTrajectory() { trj.clear(); }
  virtual ~CartesianTrajectory() = default;
  CartesianTrajectory(const CartesianTrajectory&) = delete;
  CartesianTrajectory& operator=(const CartesianTrajectory&) = delete;
  CartesianTrajectory(CartesianTrajectory&&) = delete;
  CartesianTrajectory& operator=(CartesianTrajectory&&) = delete;

  virtual bool isEmpty() const { return trj.size() > 0; }
  virtual bool append( cnr_interpolator_interface::InterpolationPointConstPtr point)
  {
    if(point)
    {
      CartesianPointConstPtr pnt = std::dynamic_pointer_cast< CartesianPoint const >(point);
      trj.push_back( *pnt );
    }
    return isEmpty();
  }

  virtual const ros::Duration& trjTime() const
  {
    return trj.size()>0 ? trj.back().time_from_start : InterpolationTrajectory::trjTime();
  }

  std::vector< CartesianPoint > trj;
};
typedef std::shared_ptr<CartesianTrajectory> CartesianTrajectoryPtr;
typedef std::shared_ptr<CartesianTrajectory const > CartesianTrajectoryConstPtr;




}  // namespace cnr_interpolator_interface

#endif  // CNR_INTERPOLATOR_INTERFACE__CNR_INTERPOLATOR_INPUTS__H
