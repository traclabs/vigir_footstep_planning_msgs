//=================================================================================================
// Copyright (c) 2015, Alexander Stumpf, TU Darmstadt
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

  void clear();

  msgs::ErrorStatus appendStepPlan(const msgs::StepPlan &step_plan);
  msgs::ErrorStatus stitchStepPlan(const msgs::StepPlan &step_plan);

  msgs::ErrorStatus insertStep(const msgs::Step& step);
  msgs::ErrorStatus updateStep(const msgs::Step& step);
  void removeStep(unsigned int step_index);
  bool hasStep(unsigned int step_index) const;

  msgs::ErrorStatus fromMsg(const msgs::StepPlan &step_plan);
  msgs::ErrorStatus toMsg(msgs::StepPlan& step_plan) const;

  // typedefs
  typedef boost::shared_ptr<StepPlan> Ptr;
  typedef boost::shared_ptr<const StepPlan> ConstPtr;

protected:
  std_msgs::Header header;
  msgs::Feet start;
  msgs::Feet goal;
  std::map<unsigned int, msgs::Step> steps;
  uint8_t mode;
  std::vector<uint8_t> data;
};
}

#endif
