//=================================================================================================
// Copyright (c) 2016, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_FOOTSTEP_PLANNING_STEP_PLAN_H__
#define VIGIR_FOOTSTEP_PLANNING_STEP_PLAN_H__

#include <ros/ros.h>

#include <tf/tf.h>

#include <boost/thread/shared_mutex.hpp>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>



namespace vigir_footstep_planning
{
/**
 * Wrapper class for vigir_footstep_planning_msgs::StepPlan to provide
 * advanced operations.
 */
class StepPlan
{
public:
  StepPlan(const msgs::StepPlan& step_plan);
  StepPlan();

  StepPlan& operator=(const msgs::StepPlan& step_plan);
  StepPlan& operator+(const msgs::StepPlan& step_plan);
  StepPlan& operator|(const msgs::StepPlan& step_plan);

  StepPlan& operator+(const msgs::Step& step);
  StepPlan& operator|(const msgs::Step& step);
  StepPlan& operator-(const msgs::Step& step);

  void clear()
  {
    boost::unique_lock<boost::shared_mutex> lock(step_plan_mutex);
    _clear();
  }
  bool empty() const
  {
    boost::shared_lock<boost::shared_mutex> lock(step_plan_mutex);
    return _empty();
  }
  size_t size() const
  {
    boost::shared_lock<boost::shared_mutex> lock(step_plan_mutex);
    return _size();
  }

  int getFirstStepIndex() const;
  int getLastStepIndex() const;

  msgs::ErrorStatus insertStep(const msgs::Step& step);
  msgs::ErrorStatus updateStep(const msgs::Step& step);

  bool hasStep(unsigned int step_index) const
  {
    boost::shared_lock<boost::shared_mutex> lock(step_plan_mutex);
    return _hasStep(step_index);
  }

  static bool getStep(msgs::Step& step, const msgs::StepPlan& step_plan, unsigned int step_index);
  inline bool getStep(msgs::Step& step, unsigned int step_index) const
  {
    boost::shared_lock<boost::shared_mutex> lock(step_plan_mutex);
    return _getStep(step, step_index);
  }

  bool getStepAt(msgs::Step& step, unsigned int position) const;
  bool getfirstStep(msgs::Step& step) const;
  bool getLastStep(msgs::Step& step) const;

  bool popStep(msgs::Step& step);
  bool popStep();

  void removeStep(unsigned int step_index);
  void removeStepAt(unsigned int position);

  void removeSteps(unsigned int from_step_index, int to_step_index = -1);

  /**
   * @brief Appends a step plan to current step plan. No transformation will be done!
   * @param step_plan Step plan to be merged into current step plan.
   * @return error status
   */
  msgs::ErrorStatus appendStepPlan(const msgs::StepPlan& step_plan);

  /**
   * @brief Merges the given step plan to current step plan.
   * Already exisiting steps *will* be tagged as modified. No transformation will be done!
   * @param step_plan Step plan to be merged into current step plan.
   * @return error status
   */
  msgs::ErrorStatus updateStepPlan(const msgs::StepPlan& step_plan); // caution: Very unrestrictive for input step_plan, does not perform consisty checks!

  /**
   * @brief Stitches the given step plan into the current step plan starting at step_index. Hereby the all
   * in range [0, step_index] are kept and all steps in range (step_index, inf] are taken from input step plan.
   * The steps at step_index from both step plans are taken as reference points (= both are
   * representing the equal position in world frame) to transform the input step plan towards each other.
   * Finally, all steps are stitched to the current step plan based on the determined
   * transformation.
   * @param step_plan Step plan to be merged into current step plan.
   * @param step_index Stitching point where the input step plan will be stitched to the current step plan.
   * @return error status
   */
  msgs::ErrorStatus stitchStepPlan(const msgs::StepPlan& step_plan, int step_index = 0);

  /**
   * @brief Determines transformation T from current to target coordinate system represented by the given poses in a
   * commom coordinate system W so target_W = T * current_W holds.
   * @param current center of current coordinate system in common system W
   * @param target center of target coordinate system in common system W
   * @return transformation between both poses
   */
  static tf::Transform getTransform(const geometry_msgs::Pose& current, const geometry_msgs::Pose& target);
  static void transformStepPlan(msgs::StepPlan& step_plan, const tf::Transform& transform);

  msgs::ErrorStatus fromMsg(const msgs::StepPlan& step_plan);
  msgs::ErrorStatus toMsg(msgs::StepPlan& step_plan) const;

  // typedefs
  typedef boost::shared_ptr<StepPlan> Ptr;
  typedef boost::shared_ptr<const StepPlan> ConstPtr;

protected:
  /** mutex free versions */
  void _clear();
  bool _empty() const;
  size_t _size() const;

  bool _hasStep(unsigned int step_index) const;

  bool _getStep(msgs::Step& step, unsigned int step_index) const;

  /**
   * @brief Inserts step into step plan. If the step does already exists it will be
   * overwritten and *not tagged* as modified.
   * @param step Step to be inserted
   * @return error status
   */
  msgs::ErrorStatus _insertStep(const msgs::Step& step);

  /**
   * @brief Inserts step into step plan. If the step does already exists it will be
   * overwritten and *tagged* as modified.
   * @param step Step to be inserted
   * @return error status
   */
  msgs::ErrorStatus _updateStep(const msgs::Step& step);

  /**
   * @brief Appends a step plan to current step plan. No transformation will be done!
   * @param step_plan Step plan to be merged into current step plan.
   * @return error status
   */
  msgs::ErrorStatus _appendStepPlan(const msgs::StepPlan& step_plan);

  /**
   * @brief Merges the given step plan to current step plan.
   * Already exisiting steps *will* be tagged as modified. No transformation will be done!
   * @param step_plan Step plan to be merged into current step plan.
   * @return error status
   */
  msgs::ErrorStatus _updateStepPlan(const msgs::StepPlan& step_plan); // caution: Very unrestrictive for input step_plan, does not perform consisty checks!

  /**
   * @brief Stitches the given step plan into the current step plan starting at step_index. Hereby the all
   * in range [0, step_index] are kept and all steps in range (step_index, inf] are taken from input step plan.
   * The steps at step_index from both step plans are taken as reference points (= both are
   * representing the equal position in world frame) to transform the input step plan towards each other.
   * Finally, all steps are stitched to the current step plan based on the determined
   * transformation.
   * @param step_plan Step plan to be merged into current step plan.
   * @return error status
   */
  msgs::ErrorStatus _stitchStepPlan(const msgs::StepPlan& step_plan, int step_index = 0);

  mutable boost::shared_mutex step_plan_mutex;

  std_msgs::Header header;
  msgs::Feet start;
  msgs::Feet goal;
  std::map<unsigned int, msgs::Step> steps;
  uint8_t mode;
  std::vector<uint8_t> data;
};
}

#endif
