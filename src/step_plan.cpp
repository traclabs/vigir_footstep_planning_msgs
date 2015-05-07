#include <vigir_footstep_planning_msgs/step_plan.h>

namespace vigir_footstep_planning
{
StepPlan::StepPlan(const msgs::StepPlan& step_plan)
{
  fromMsg(step_plan);
}

StepPlan::StepPlan()
  : mode(0)
{
}

StepPlan& StepPlan::operator=(const msgs::StepPlan& step_plan)
{
  fromMsg(step_plan);
  return *this;
}

StepPlan& StepPlan::operator+(const msgs::StepPlan& step_plan)
{
  appendStepPlan(step_plan);
  return *this;
}

StepPlan& StepPlan::operator|(const msgs::StepPlan& step_plan)
{
  stitchStepPlan(step_plan);
  return *this;
}

StepPlan& StepPlan::operator+(const msgs::Step& step)
{
  insertStep(step);
  return *this;
}

StepPlan& StepPlan::operator|(const msgs::Step& step)
{
  updateStep(step);
  return *this;
}

StepPlan& StepPlan::operator-(const msgs::Step& step)
{
  removeStep(step.step_index);
  return *this;
}

void StepPlan::clear()
{
  header = std_msgs::Header();
  start = msgs::Feet();
  goal = msgs::Feet();
  steps.clear();
  mode = 0;
  data.clear();
}

msgs::ErrorStatus StepPlan::appendStepPlan(const msgs::StepPlan &step_plan)
{
  // init header
  if (steps.empty())
  {
    header = step_plan.header;
    mode = step_plan.mode;
    data = step_plan.data;
  }

  // check for errors
  msgs::ErrorStatus status = isConsistent(step_plan);
  if (status.error != msgs::ErrorStatus::NO_ERROR)
    return status;
  else if (step_plan.steps.empty())
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "appendStepPlan", "Got empty plan!");
  else if (step_plan.header.frame_id != header.frame_id)
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "appendStepPlan", "Frame id mismatch of plans: " + header.frame_id + " vs. " + step_plan.header.frame_id);
  else if (mode != step_plan.mode)
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "appendStepPlan", "Plans have different modes!");

  // append plan
  unsigned int offset = steps.empty() ? 0u : steps.rbegin()->second.step_index + 1;
  for (size_t i = 0u; i < step_plan.steps.size(); i++)
  {
    msgs::Step step = step_plan.steps[i];
    step.step_index = i + offset; // overwrite step index
    status += insertStep(step);

    if (status.error != msgs::ErrorStatus::NO_ERROR)
      break;
  }

  return status;
}

msgs::ErrorStatus StepPlan::stitchStepPlan(const msgs::StepPlan &step_plan)
{
  // init header
  if (steps.empty())
  {
    header = step_plan.header;
    mode = step_plan.mode;
    data = step_plan.data;
  }

  // check for errors
  msgs::ErrorStatus status = isConsistent(step_plan);
  if (status.error != msgs::ErrorStatus::NO_ERROR)
    return status;
  else if (step_plan.steps.empty())
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "stitchStepPlan", "Got empty plan!");
  else if (step_plan.header.frame_id != header.frame_id)
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "stitchStepPlan", "Frame id mismatch of plans: " + header.frame_id + " vs. " + step_plan.header.frame_id);
  else if (mode != step_plan.mode)
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "stitchStepPlan", "Plans have different modes!");

  // stitch plan
  for (size_t i = 0u; i < step_plan.steps.size(); i++)
  {
    status += insertStep(step_plan.steps[i]);

    if (status.error != msgs::ErrorStatus::NO_ERROR)
      break;
  }

  return status;
}

msgs::ErrorStatus StepPlan::insertStep(const msgs::Step& step)
{
  // check for errors
  if (steps.empty())
    header = step.header;
  else if (step.header.frame_id != header.frame_id)
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "insertStep", "Frame id mismatch! Plan: " + header.frame_id + " vs. step: " + step.header.frame_id);

  // insert step
  this->steps[step.step_index] = step;

  return msgs::ErrorStatus();
}

msgs::ErrorStatus StepPlan::updateStep(const msgs::Step& step)
{
  // check for errors
  if (steps.empty())
    header = step.header;
  else if (step.header.frame_id != header.frame_id)
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "updateStep", "Frame id mismatch! Plan: " + header.frame_id + " vs. step: " + step.header.frame_id);

  // check if step has already existed
  bool modified(hasStep(step.step_index));

  // insert/update step
  this->steps[step.step_index] = step;
  this->steps[step.step_index].modified = modified;

  return msgs::ErrorStatus();
}

void StepPlan::removeStep(unsigned int step_index)
{
  steps.erase(step_index);
}

bool StepPlan::hasStep(unsigned int step_index) const
{
  return steps.find(step_index) != steps.end();
}

msgs::ErrorStatus StepPlan::fromMsg(const msgs::StepPlan &step_plan)
{
  clear();
  msgs::ErrorStatus status = isConsistent(step_plan);

  // check for errors
  if (status.error != msgs::ErrorStatus::NO_ERROR)
    return status;

  // convert step plan
  header = step_plan.header;

  start = step_plan.start;
  goal = step_plan.goal;

  steps.clear();
  for (unsigned int i = 0u; i < step_plan.steps.size(); i++)
  {
    status += insertStep(step_plan.steps[i]);

    if (status.error != msgs::ErrorStatus::NO_ERROR)
      break;
  }

  mode = step_plan.mode;
  data = step_plan.data;

  return status;
}

msgs::ErrorStatus StepPlan::toMsg(msgs::StepPlan& step_plan) const
{
  msgs::ErrorStatus status;

  // convert step plan
  step_plan.header = header;

  step_plan.start = start;
  step_plan.goal = goal;

  step_plan.steps.clear();
  for (std::map<unsigned int, msgs::Step>::const_iterator itr = steps.begin(); itr != steps.end(); itr++)
    step_plan.steps.push_back(itr->second);

  step_plan.mode = mode;
  step_plan.data = data;

  status += isConsistent(step_plan);

  return status;
}
}
