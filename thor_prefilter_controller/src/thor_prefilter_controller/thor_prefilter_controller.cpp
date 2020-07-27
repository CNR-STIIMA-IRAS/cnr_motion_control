#include <thor_prefilter_controller/thor_prefilter_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(thor::PrefilterPosController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(thor::PrefilterPosVelEffController, controller_interface::ControllerBase)

namespace thor
{


/* ===============================================================================
 * ===============================================================================
 * ===============================================================================
 * ========================== PrefilterPosVelEffController =======================
 * ===============================================================================
 * ===============================================================================
 * =============================================================================== */

bool PrefilterPosVelEffController::doInit()
{
  CNR_TRACE_START(*m_logger);
  m_ctrl.reset(new thor::PrefilterController< hardware_interface::PosVelEffJointInterface > (this));
  CNR_RETURN_TRUE(*m_logger);
}

bool PrefilterPosVelEffController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  std::vector<double> qini(m_nAx);
  std::vector<double> Dqini(m_nAx, 0);
  for (unsigned int idx = 0; idx < m_nAx; idx++)
  {
    qini.at(idx) = m_hw->getHandle(m_joint_names.at(idx)).getPosition();
    Dqini.at(idx) = m_hw->getHandle(m_joint_names.at(idx)).getVelocity();
  }
  m_ctrl->starting(time, qini, Dqini);
  CNR_RETURN_TRUE(*m_logger);
}

bool PrefilterPosVelEffController::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  m_ctrl->stopping(time);
  CNR_RETURN_TRUE(*m_logger);
}

bool PrefilterPosVelEffController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  //CNR_TRACE_START_THROTTLE(*m_logger, 10.0);
  ros::Time st = ros::Time::now();
  ros::Time et = ros::Time::now();
  if (m_ctrl->update(time, period))
  {
    try
    {
      for (unsigned int iDim = 0; iDim < m_nAx; iDim++)
      {
        m_hw->getHandle(m_joint_names.at(iDim)).setCommandPosition(m_ctrl->getTrajectoryPoint().positions.at(iDim));
        m_hw->getHandle(m_joint_names.at(iDim)).setCommandVelocity(m_ctrl->getTrajectoryPoint().velocities.at(iDim));
        if (m_ctrl->getTrajectoryPoint().effort.size() == m_nAx)
        {
          m_hw->getHandle(m_joint_names.at(iDim)).setCommandEffort(m_ctrl->getTrajectoryPoint().effort.at(iDim));
        }
      }
    }
    catch (std::exception& e)
    {
      CNR_RETURN_FALSE(*m_logger, "[ " + getControllerNamespace() + " ] Update failure: exception: " + std::string(e.what()));
    }
  }
  else
  {
    CNR_WARN_THROTTLE(*m_logger, 1.0, "[ " + getControllerNamespace() + " ] PLANNER NOT CONFIGURED");
  }
  return true;
//  CNR_RETURN_TRUE_THROTTLE(*m_logger, 10.0);
}




/* ===============================================================================
 * ===============================================================================
 * ===============================================================================
 * ============================= PrefilterPosController ==========================
 * ===============================================================================
 * ===============================================================================
 * =============================================================================== */

bool PrefilterPosController::doInit()
{
  CNR_TRACE_START(*m_logger);
  m_ctrl.reset(new thor::PrefilterController< hardware_interface::PositionJointInterface > (this));
  CNR_RETURN_TRUE(*m_logger);
}

bool PrefilterPosController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  std::vector<double> qini(m_nAx);
  std::vector<double> Dqini(m_nAx, 0);
  for (unsigned int idx = 0; idx < m_nAx; idx++)
  {
    qini.at(idx) = m_hw->getHandle(m_joint_names.at(idx)).getPosition();
    Dqini.at(idx) = m_hw->getHandle(m_joint_names.at(idx)).getVelocity();
  }
  m_ctrl->starting(time, qini, Dqini);
  CNR_RETURN_TRUE(*m_logger);
}

bool PrefilterPosController::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  m_ctrl->stopping(time);
  CNR_RETURN_TRUE(*m_logger);
}

bool PrefilterPosController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
//  CNR_TRACE_START_THROTTLE(*m_logger, 10.0);
  if (m_ctrl->update(time, period))
  {
    try
    {
      for (unsigned int iDim = 0; iDim < m_nAx; iDim++)
      {
        m_hw->getHandle(m_joint_names.at(iDim)).setCommand(m_ctrl->getTrajectoryPoint().positions.at(iDim));
      }
    }
    catch (std::exception& e)
    {
      CNR_RETURN_FALSE(*m_logger, "[ " + getControllerNamespace() + " ] Update failure: exception: " + std::string(e.what()));
    }
  }
  else
  {
    CNR_WARN_THROTTLE(*m_logger, 1.0, "[ " + getControllerNamespace() + " ] PLANNER NOT CONFIGURED");
  }
  return true;
//  CNR_RETURN_TRUE_THROTTLE(*m_logger, 10.0);

}








}
