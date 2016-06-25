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
  updateStepPlan(step_plan);
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

int StepPlan::getFirstStepIndex() const
{
  boost::shared_lock<boost::shared_mutex> lock(step_plan_mutex);

  if (steps.empty())
    return -1;
  else
    return steps.begin()->second.step_index;
}

int StepPlan::getLastStepIndex() const
{
  boost::shared_lock<boost::shared_mutex> lock(step_plan_mutex);

  if (steps.empty())
    return -1;
  else
    return steps.rbegin()->second.step_index;
}

msgs::ErrorStatus StepPlan::insertStep(const msgs::Step& step)
{
  boost::unique_lock<boost::shared_mutex> lock(step_plan_mutex);
  return _insertStep(step);
}

msgs::ErrorStatus StepPlan::updateStep(const msgs::Step& step)
{
  boost::unique_lock<boost::shared_mutex> lock(step_plan_mutex);
  return _updateStep(step);
}

bool StepPlan::getStep(msgs::Step& step, const msgs::StepPlan& step_plan, unsigned int step_index)
{
  for (std::vector<msgs::Step>::const_iterator itr = step_plan.steps.begin(); itr != step_plan.steps.end(); itr++)
  {
    const msgs::Step& s = *itr;
    if (s.step_index == step_index)
    {
      step = s;
      return true;
    }
  }
  return false;
}

bool StepPlan::getStepAt(msgs::Step& step, unsigned int position) const
{
  boost::shared_lock<boost::shared_mutex> lock(step_plan_mutex);

  if (steps.empty())
    return false;
  else
  {
    std::map<unsigned int, msgs::Step>::const_iterator itr = steps.begin();
    std::advance(itr, position);
    step = itr->second;
    return true;
  }
}

bool StepPlan::getfirstStep(msgs::Step& step) const
{
  boost::shared_lock<boost::shared_mutex> lock(step_plan_mutex);

  if (steps.empty())
    return false;
  else
  {
    step = steps.begin()->second;
    return true;
  }
}

bool StepPlan::getLastStep(msgs::Step& step) const
{
  boost::shared_lock<boost::shared_mutex> lock(step_plan_mutex);

  if (steps.empty())
    return false;
  else
  {
    step = steps.rbegin()->second;
    return true;
  }
}

bool StepPlan::popStep(msgs::Step& step)
{
  boost::unique_lock<boost::shared_mutex> lock(step_plan_mutex);

  if (steps.empty())
    return false;
  else
  {
    step = steps.begin()->second;
    steps.erase(steps.begin());
    return true;
  }
}

bool StepPlan::popStep()
{
  msgs::Step step;
  popStep(step);
}

void StepPlan::removeStep(unsigned int step_index)
{
  boost::unique_lock<boost::shared_mutex> lock(step_plan_mutex);
  steps.erase(step_index);
}

void StepPlan::removeStepAt(unsigned int position)
{
  boost::unique_lock<boost::shared_mutex> lock(step_plan_mutex);
  steps.erase(steps.find(position));
}

void StepPlan::removeSteps(unsigned int from_step_index, int to_step_index)
{
  boost::unique_lock<boost::shared_mutex> lock(step_plan_mutex);

  if (steps.empty())
    return;

  from_step_index = std::max(from_step_index, steps.begin()->first);
  if (from_step_index > to_step_index)
    return;

  std::map<unsigned int, msgs::Step>::iterator start_itr = steps.find(from_step_index);
  std::map<unsigned int, msgs::Step>::iterator end_itr = to_step_index >= 0 ? steps.find(to_step_index+1) : steps.end();
  steps.erase(start_itr, end_itr);
}

msgs::ErrorStatus StepPlan::appendStepPlan(const msgs::StepPlan& step_plan)
{
  boost::unique_lock<boost::shared_mutex> lock(step_plan_mutex);

  // check for errors
  msgs::ErrorStatus status = isConsistent(step_plan);
  if (status.error != msgs::ErrorStatus::NO_ERROR)
    return status;
  else if (step_plan.steps.empty())
    return ErrorStatusWarning(msgs::ErrorStatus::WARN_INVALID_STEP_PLAN, "appendStepPlan", "Got empty plan!");

  if (!_empty())
  {
    if (step_plan.header.frame_id != header.frame_id)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "appendStepPlan", "Frame id mismatch of plans: " + header.frame_id + " vs. " + step_plan.header.frame_id);
    else if (mode != step_plan.mode)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "appendStepPlan", "Plans have different modes!");
  }
  // append plan
  return _appendStepPlan(step_plan);
}

msgs::ErrorStatus StepPlan::updateStepPlan(const msgs::StepPlan& step_plan)
{
  boost::unique_lock<boost::shared_mutex> lock(step_plan_mutex);

  // check for errors
  if (step_plan.steps.empty())
    return ErrorStatusWarning(msgs::ErrorStatus::WARN_INVALID_STEP_PLAN, "updateStepPlan", "Got empty plan!");

  if (!_empty())
  {
    if (step_plan.header.frame_id != header.frame_id)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "updateStepPlan", "Frame id mismatch of plans: " + header.frame_id + " vs. " + step_plan.header.frame_id);
    else if (mode != step_plan.mode)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "updateStepPlan", "Plans have different modes!");
  }

  // update plan
  return _updateStepPlan(step_plan);
}

msgs::ErrorStatus StepPlan::stitchStepPlan(const msgs::StepPlan& step_plan, int step_index)
{
  boost::unique_lock<boost::shared_mutex> lock(step_plan_mutex);

  // check for errors
  msgs::ErrorStatus status = isConsistent(step_plan);
  if (status.error != msgs::ErrorStatus::NO_ERROR)
    return status;
  else if (step_plan.steps.empty())
    return ErrorStatusWarning(msgs::ErrorStatus::WARN_INVALID_STEP_PLAN, "stitchStepPlan", "Got empty plan!");

  if (!_empty())
  {
    if (step_plan.header.frame_id != header.frame_id)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "stitchStepPlan", "Frame id mismatch of plans: " + header.frame_id + " vs. " + step_plan.header.frame_id);
    else if (mode != step_plan.mode)
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "stitchStepPlan", "Plans have different modes!");
    else if (step_index > 0 && (steps.empty() || step_index > steps.rbegin()->second.step_index))
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "stitchStepPlan", "Can't stitch as requested step index is not in current step plan!");
    /// TODO: Check if stiching would result in consistent step plan
  }

  // stitch plan
  return _stitchStepPlan(step_plan, step_index);
}

tf::Transform StepPlan::getTransform(const geometry_msgs::Pose& current, const geometry_msgs::Pose& target)
{
  tf::Pose ref_current;
  tf::poseMsgToTF(current, ref_current);

  tf::Pose ref_target;
  tf::poseMsgToTF(target, ref_target);
  //ref_target.setRotation(tf::createQuaternionFromYaw(tf::getYaw(ref_target.getRotation()))); /// HACK to clamp everything to flat ground

  // determine transformation
  return ref_target * ref_current.inverse();
}

void StepPlan::transformStepPlan(msgs::StepPlan& step_plan, const tf::Transform& transform)
{
  tf::Pose pose;

  // start pose
  tf::poseMsgToTF(step_plan.start.left.pose, pose);
  pose = transform * pose;
  tf::poseTFToMsg(pose, step_plan.start.left.pose);

  tf::poseMsgToTF(step_plan.start.right.pose, pose);
  pose = transform * pose;
  tf::poseTFToMsg(pose, step_plan.start.right.pose);

  // goal pose
  tf::poseMsgToTF(step_plan.goal.left.pose, pose);
  pose = transform * pose;
  tf::poseTFToMsg(pose, step_plan.goal.left.pose);

  tf::poseMsgToTF(step_plan.goal.right.pose, pose);
  pose = transform * pose;
  tf::poseTFToMsg(pose, step_plan.goal.right.pose);

  // entire plan
  for (std::vector<msgs::Step>::iterator itr = step_plan.steps.begin(); itr != step_plan.steps.end(); itr++)
  {
    msgs::Step& step = *itr;

    tf::poseMsgToTF(step.foot.pose, pose);
    pose = transform * pose;
    tf::poseTFToMsg(pose, step.foot.pose);
  }
}

void StepPlan::_clear()
{
  header = std_msgs::Header();
  start = msgs::Feet();
  goal = msgs::Feet();
  steps.clear();
  mode = 0;
  data.clear();
}

bool StepPlan::_empty() const
{
  return steps.empty();
}

size_t StepPlan::_size() const
{
  return steps.size();
}

bool StepPlan::_hasStep(unsigned int step_index) const
{
  return steps.find(step_index) != steps.end();
}

bool StepPlan::_getStep(msgs::Step& step, unsigned int step_index) const
{
  std::map<unsigned int, msgs::Step>::const_iterator itr = steps.find(step_index);
  if (itr != steps.end())
  {
    step = itr->second;
    return true;
  }
  return false;
}

msgs::ErrorStatus StepPlan::_insertStep(const msgs::Step& step)
{
  // check for errors
  if (steps.empty())
    header = step.header;
  else if (step.header.frame_id != header.frame_id)
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "insertStep", "Frame id mismatch! Plan: " + header.frame_id + " vs. step: " + step.header.frame_id);

  // insert step
  steps[step.step_index] = step;

  return msgs::ErrorStatus();
}

msgs::ErrorStatus StepPlan::_updateStep(const msgs::Step& step)
{
  // check for errors
  if (steps.empty())
    header = step.header;
  else if (step.header.frame_id != header.frame_id)
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "updateStep", "Frame id mismatch! Plan: " + header.frame_id + " vs. step: " + step.header.frame_id);

  // check if step has already existed
  bool modified(_hasStep(step.step_index));

  // insert/update step
  steps[step.step_index] = step;
  steps[step.step_index].modified = modified;

  return msgs::ErrorStatus();
}

msgs::ErrorStatus StepPlan::_appendStepPlan(const msgs::StepPlan& step_plan)
{
  int step_index = 0;

  // init header
  if (steps.empty())
  {
    header = step_plan.header;
    mode = step_plan.mode;
    data = step_plan.data;
  }
  else
    step_index = steps.rbegin()->second.step_index + 1;

  // append plan
  msgs::ErrorStatus status;
  for (msgs::Step step : step_plan.steps)
  {
    step.step_index = step_index++;
    status += _insertStep(step);

    if (status.error != msgs::ErrorStatus::NO_ERROR)
      break;
  }

  return status;
}

msgs::ErrorStatus StepPlan::_updateStepPlan(const msgs::StepPlan& step_plan)
{
  // init header
  if (steps.empty())
  {
    header = step_plan.header;
    mode = step_plan.mode;
    data = step_plan.data;
  }

  // update plan
  msgs::ErrorStatus status;
  for (std::vector<msgs::Step>::const_iterator itr = step_plan.steps.begin(); itr != step_plan.steps.end(); itr++)
  {
    status += _updateStep(*itr);

    if (status.error != msgs::ErrorStatus::NO_ERROR)
      break;
  }

  return status;
}

msgs::ErrorStatus StepPlan::_stitchStepPlan(const msgs::StepPlan& step_plan, int step_index)
{
  // simple case: current step plan is still empty
  if (steps.empty())
    return _updateStepPlan(step_plan);

  msgs::Step last_current_step;
  msgs::Step first_new_step;

  if (step_index == 0)
  {
    last_current_step = steps.rbegin()->second;
    first_new_step = step_plan.steps.front();
  }
  else
  {
    if (!_getStep(last_current_step, step_index))
      return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_STEP, "stitchStepPlan", "Current step plan doesn't contain step index " + boost::lexical_cast<std::string>(step_index) + "!");

    if (!getStep(first_new_step, step_plan, step_index))
      return ErrorStatusError(msgs::ErrorStatus::ERR_INVALID_STEP, "stitchStepPlan", "Input step plan doesn't contain step index " + boost::lexical_cast<std::string>(step_index) + "!");
  }

  // ensure overlapping first step
  if (last_current_step.foot.foot_index != first_new_step.foot.foot_index)
    return ErrorStatusError(msgs::ErrorStatus::ERR_INCONSISTENT_STEP_PLAN, "stitchStepPlan", std::string("First foot ") +
                            (first_new_step.foot.foot_index == msgs::Foot::RIGHT ? std::string("(RIGHT)") : std::string("(LEFT)")) + std::string(" of input step plan doesn't match ") +
                            (last_current_step.foot.foot_index == msgs::Foot::RIGHT ? std::string("(RIGHT)") : std::string("(LEFT)")) + std::string(" last foot of current step plan."));

  // transform input plan to be relative to current plan's reference foot pose
  msgs::StepPlan step_plan_transformed = step_plan;
  step_plan_transformed.steps.clear();
  
  for (msgs::Step step : step_plan.steps)
  {
    if (step.step_index >= step_index)
      step_plan_transformed.steps.push_back(step);
  }
  
  // determine transformation 'input plan first step' -> 'current plan last step'
  tf::Transform transform = getTransform(first_new_step.foot.pose, last_current_step.foot.pose);
  transformStepPlan(step_plan_transformed, transform);
  
  // remove remaining tail of old step plan
  unsigned int max_step_index = step_plan_transformed.steps.back().step_index;
  for (std::map<unsigned int, msgs::Step>::iterator itr = steps.begin(); itr != steps.end();)
  {
    if (itr->second.step_index > max_step_index)
      steps.erase(itr++);
    else
      itr++;
  }

  // update plan using the transformed step plan
  return _updateStepPlan(step_plan_transformed);
}

msgs::ErrorStatus StepPlan::fromMsg(const msgs::StepPlan& step_plan)
{
  clear();
  msgs::ErrorStatus status = isConsistent(step_plan);

  // check for errors
  if (status.error != msgs::ErrorStatus::NO_ERROR)
    return status;

  boost::unique_lock<boost::shared_mutex> lock(step_plan_mutex);

  // convert step plan
  header = step_plan.header;

  start = step_plan.start;
  goal = step_plan.goal;

  mode = step_plan.mode;
  data = step_plan.data;

  steps.clear();

  for (unsigned int i = 0u; i < step_plan.steps.size(); i++)
  {
    status += _insertStep(step_plan.steps[i]);

    if (status.error != msgs::ErrorStatus::NO_ERROR)
      break;
  }

  return status;
}

msgs::ErrorStatus StepPlan::toMsg(msgs::StepPlan& step_plan) const
{
  boost::shared_lock<boost::shared_mutex> lock(step_plan_mutex);

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
