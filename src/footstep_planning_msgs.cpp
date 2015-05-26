#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>

namespace vigir_footstep_planning
{
// Extension to ErrorStatus message
msgs::ErrorStatus operator+(const msgs::ErrorStatus& lhs, const msgs::ErrorStatus& rhs)
{
  msgs::ErrorStatus result;

  result.error = lhs.error | rhs.error;
  result.error_msg = lhs.error_msg;
  if (result.error_msg.size() && rhs.error_msg.size())
    result.error_msg += "\n";
  result.error_msg += rhs.error_msg;

  result.warning = lhs.warning | rhs.warning;
  result.warning_msg = lhs.warning_msg;
  if (result.warning_msg.size() && rhs.warning_msg.size())
    result.warning_msg += "\n";
  result.warning_msg += rhs.warning_msg;

  return result;
}

msgs::ErrorStatus operator+=(msgs::ErrorStatus& lhs, const msgs::ErrorStatus& rhs)
{
  lhs = operator+(lhs, rhs);
  return lhs;
}

msgs::ErrorStatus isConsistent(const msgs::StepPlan& step_plan)
{
  // an empty plan is always consistent
  if (step_plan.steps.empty())
    return msgs::ErrorStatus();

  msgs::ErrorStatus status;

  // check if steps are ordered and closed
  unsigned int step_index = step_plan.steps.front().step_index;
  for (std::vector<msgs::Step>::const_iterator itr = step_plan.steps.begin(); itr != step_plan.steps.end(); itr++)
  {
    if (itr->header.frame_id != step_plan.header.frame_id)
      status += ErrorStatusError(msgs::ErrorStatus::ERR_INCONSISTENT_STEP_PLAN, "isConsistent", "Frame id mismatch! Plan: " + step_plan.header.frame_id + " vs. step: " + itr->header.frame_id);

    if (itr->step_index != step_index++)
      status += ErrorStatusWarning(msgs::ErrorStatus::WARN_INCOMPLETE_STEP_PLAN, "isConsistent", "Wrong step index: Expected " + boost::lexical_cast<std::string>(step_index-1) + " but got " + boost::lexical_cast<std::string>(itr->step_index) + "!");

    if (!itr->valid)
      status += ErrorStatusWarning(msgs::ErrorStatus::WARN_INVALID_STEP_PLAN, "isConsistent", "Step " + boost::lexical_cast<std::string>(itr->step_index) + " invalid!");
  }

  return status;
}

std::string ErrorStatusCodeToString(unsigned int error)
{
  switch (error)
  {
    case msgs::ErrorStatus::NO_ERROR: return "NO_ERROR";
    case msgs::ErrorStatus::ERR_UNKNOWN: return "ERR_UNKNOWN";
    case msgs::ErrorStatus::ERR_NO_SOLUTION: return "ERR_NO_SOLUTION";
    case msgs::ErrorStatus::ERR_INVALID_START: return "ERR_INVALID_START";
    case msgs::ErrorStatus::ERR_INVALID_GOAL: return "ERR_INVALID_GOAL";
    case msgs::ErrorStatus::ERR_INVALID_GRID_MAP: return "ERR_INVALID_GRID_MAP";
    case msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL: return "ERR_INVALID_TERRAIN_MODEL";
    case msgs::ErrorStatus::ERR_INVALID_STEP: return "ERR_INVALID_STEP";
    case msgs::ErrorStatus::ERR_INCONSISTENT_STEP_PLAN: return "ERR_INCONSISTENT_STEP_PLAN";
    case msgs::ErrorStatus::ERR_INVALID_PARAMETERS: return "ERR_INVALID_PARAMETERS";
    case msgs::ErrorStatus::ERR_NO_PLUGIN_AVAILABLE: return "ERR_NO_PLUGIN_AVAILABLE";
    case msgs::ErrorStatus::ERR_INCONSISTENT_REQUEST: return "ERR_INCONSISTENT_REQUEST";
    default: return "ERR_UNKNOWN";
  }
}

std::string WarningStatusCodeToString(unsigned int warning)
{
  switch (warning)
  {
    case msgs::ErrorStatus::NO_WARNING: return "NO_WARNING";
    case msgs::ErrorStatus::WARN_UNKNOWN: return "WARN_UNKNOWN";
    case msgs::ErrorStatus::WARN_INCOMPLETE_STEP_PLAN: return "WARN_INCOMPLETE_STEP_PLAN";
    case msgs::ErrorStatus::WARN_INVALID_STEP_PLAN: return "WARN_INVALID_STEP_PLAN";
    default: return "WARN_UNKNOWN";
  }
}

msgs::ErrorStatus createErrorStatus(const std::string& context, unsigned int error, const std::string& error_msg, unsigned int warning, const std::string& warning_msg, bool output)
{
  if (output)
  {
    if (error)
      ROS_ERROR("[%s][%s] %s", ErrorStatusCodeToString(error).c_str(), context.c_str(), error_msg.c_str());
    if (warning)
      ROS_WARN("[%s][%s] %s", WarningStatusCodeToString(warning).c_str(), context.c_str(), warning_msg.c_str());
  }

  msgs::ErrorStatus status;
  status.error = error;
  status.error_msg = error != msgs::ErrorStatus::NO_ERROR ? "[" + ErrorStatusCodeToString(error) + "][" + context + "] " + error_msg : error_msg;
  status.warning = warning;
  status.warning_msg = warning != msgs::ErrorStatus::NO_WARNING ? "[" + WarningStatusCodeToString(warning) + "][" + context + "] " + warning_msg : warning_msg;
  return status;
}

msgs::ErrorStatus ErrorStatusError(unsigned int error, const std::string& context, const std::string& error_msg, bool output)
{
  return createErrorStatus(context, error, error_msg, msgs::ErrorStatus::NO_WARNING, "", output);
}

msgs::ErrorStatus ErrorStatusWarning(unsigned int warning, const std::string& context, const std::string& warning_msg, bool output)
{
  return createErrorStatus(context, msgs::ErrorStatus::NO_ERROR, "", warning, warning_msg, output);
}

bool hasError(const msgs::ErrorStatus& status)
{
  return status.error != msgs::ErrorStatus::NO_ERROR;
}

bool hasWarning(const msgs::ErrorStatus& status)
{
  return status.warning != msgs::ErrorStatus::NO_WARNING;
}

bool isOk(const msgs::ErrorStatus& status)
{
  return !hasError(status) && !hasWarning(status);
}

std::string toString(const msgs::ErrorStatus& error_status)
{
  std::string msg;

  if (error_status.error_msg.size())
    msg = error_status.error_msg;

  if (error_status.warning_msg.size())
  {
    if (msg.size())
      msg += "\n";
    msg = error_status.warning_msg;
  }

  return msg;
}

std::string toString(const msgs::FootstepExecutionStatus& execution_status)
{
  if (!execution_status.status)
    return "NO_ERROR";

  std::string msg;

  if (execution_status.status & msgs::FootstepExecutionStatus::REACHED_GOAL)
    msg += "REACHED_GOAL ";
  if (execution_status.status & msgs::FootstepExecutionStatus::WAITING_FOR_ATLAS)
    msg += "WAITING_FOR_ATLAS ";
  if (execution_status.status & msgs::FootstepExecutionStatus::EXECUTING_STEP_PLAN)
    msg += "EXECUTING_STEP_PLAN ";
  if (execution_status.status & msgs::FootstepExecutionStatus::PREMATURE_HALT)
    msg += "PREMATURE_HALT ";
  if (execution_status.status & msgs::FootstepExecutionStatus::UPDATED_STEP_PLAN)
    msg += "UPDATED_STEP_PLAN ";
  if (execution_status.status & msgs::FootstepExecutionStatus::EMERGENCY_HALT)
    msg += "EMERGENCY_HALT ";
  if (execution_status.status & msgs::FootstepExecutionStatus::ERR_UNKNOWN)
    msg += "ERR_UNKNOWN ";
  if (execution_status.status & msgs::FootstepExecutionStatus::ERR_EMPTY_PLAN)
    msg += "ERR_EMPTY_PLAN ";
  if (execution_status.status & msgs::FootstepExecutionStatus::ERR_INVALID_BEHAVIOR_MODE)
    msg += "ERR_INVALID_BEHAVIOR_MODE ";
  if (execution_status.status & msgs::FootstepExecutionStatus::ERR_INVALID_PLAN)
    msg += "ERR_INVALID_PLAN ";
  if (execution_status.status & msgs::FootstepExecutionStatus::ERR_INVALID_INCONSISTENT_INDICES)
    msg += "ERR_INVALID_INCONSISTENT_INDICES ";
  if (execution_status.status & msgs::FootstepExecutionStatus::ERR_INCONSISTENT_STEP_PLAN)
    msg += "ERR_INCONSISTENT_STEP_PLAN ";
  if (execution_status.status & msgs::FootstepExecutionStatus::ERR_CONTROLLER_NOT_READY)
    msg += "ERR_CONTROLLER_NOT_READY ";
  if (execution_status.status & msgs::FootstepExecutionStatus::ERR_FALLEN)
    msg += "ERR_FALLEN ";
  if (execution_status.status & msgs::FootstepExecutionStatus::ERR_NON_ZERO_INITIAL_STEP)
    msg += "ERR_NON_ZERO_INITIAL_STEP ";
  if (execution_status.status & msgs::FootstepExecutionStatus::ERR_NEXT_ERROR)
    msg += "ERR_NEXT_ERROR ";

  return msg;
}

std::string toString(const XmlRpc::XmlRpcValue& val)
{
  std::ostringstream ss;
  val.write(ss);
  return ss.str();
}

std::string toString(const XmlRpc::XmlRpcValue::Type& type)
{
  switch (type)
  {
    case XmlRpc::XmlRpcValue::TypeInvalid:  return "TypeInvalid";
    case XmlRpc::XmlRpcValue::TypeBoolean:  return "TypeBoolean";
    case XmlRpc::XmlRpcValue::TypeInt:      return "TypeInt";
    case XmlRpc::XmlRpcValue::TypeDouble:   return "TypeDouble";
    case XmlRpc::XmlRpcValue::TypeString:   return "TypeString";
    case XmlRpc::XmlRpcValue::TypeDateTime: return "TypeDateTime";
    case XmlRpc::XmlRpcValue::TypeBase64:   return "TypeBase64";
    case XmlRpc::XmlRpcValue::TypeArray:    return "TypeArray";
    case XmlRpc::XmlRpcValue::TypeStruct:   return "TypeStruct";
    default: return "Unknown Type!";
  }
}

std::string& strip(std::string& s, const char c)
{
  while (!s.empty() && s[0] == c)
    s = s.substr(1);
  while (!s.empty() && s[s.size()-1] == c)
    s = s.substr(0, s.size()-1);
  return s;
}

std::string strip_const(const std::string& s, const char c)
{
  std::string _s = s;
  return strip(_s, c);
}
}
