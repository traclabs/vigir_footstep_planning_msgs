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

#ifndef VIGIR_FOOTSTEP_PLANNING_MSGS_H__
#define VIGIR_FOOTSTEP_PLANNING_MSGS_H__

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>

#include <vigir_footstep_planning_msgs/serialization.h>

// messages
#include <vigir_footstep_planning_msgs/EditStep.h>
#include <vigir_footstep_planning_msgs/ErrorStatus.h>
#include <vigir_footstep_planning_msgs/Feet.h>
#include <vigir_footstep_planning_msgs/FeetPoseRequest.h>
#include <vigir_footstep_planning_msgs/FootstepExecutionStatus.h>
#include <vigir_footstep_planning_msgs/Foot.h>
#include <vigir_footstep_planning_msgs/PatternParameters.h>
#include <vigir_footstep_planning_msgs/PatternGeneratorParameters.h>
#include <vigir_footstep_planning_msgs/ParameterSet.h>
#include <vigir_footstep_planning_msgs/Parameter.h>
#include <vigir_footstep_planning_msgs/PlanningFeedback.h>
#include <vigir_footstep_planning_msgs/StepPlanFeedback.h>
#include <vigir_footstep_planning_msgs/StepPlanRequest.h>
#include <vigir_footstep_planning_msgs/StepPlan.h>
#include <vigir_footstep_planning_msgs/Step.h>
#include <vigir_footstep_planning_msgs/UpdateMode.h>

// services
#include <vigir_footstep_planning_msgs/EditStepService.h>
#include <vigir_footstep_planning_msgs/GenerateFeetPoseService.h>
#include <vigir_footstep_planning_msgs/GeneratePatternService.h>
#include <vigir_footstep_planning_msgs/GetAllParameterSetsService.h>
#include <vigir_footstep_planning_msgs/GetParameterSetNamesService.h>
#include <vigir_footstep_planning_msgs/GetParameterSetService.h>
#include <vigir_footstep_planning_msgs/GetStepPlanService.h>
#include <vigir_footstep_planning_msgs/PatternGeneratorParametersService.h>
#include <vigir_footstep_planning_msgs/SetParameterSetService.h>
#include <vigir_footstep_planning_msgs/SetStepPlanService.h>
#include <vigir_footstep_planning_msgs/StepPlanRequestService.h>
#include <vigir_footstep_planning_msgs/StitchStepPlanService.h>
#include <vigir_footstep_planning_msgs/StitchStepPlanService.h>
#include <vigir_footstep_planning_msgs/TransformFeetPosesService.h>
#include <vigir_footstep_planning_msgs/TransformFootPoseService.h>
#include <vigir_footstep_planning_msgs/TransformStepPlanService.h>
#include <vigir_footstep_planning_msgs/StitchStepPlanService.h>
#include <vigir_footstep_planning_msgs/UpdateFeetService.h>
#include <vigir_footstep_planning_msgs/UpdateFootService.h>
#include <vigir_footstep_planning_msgs/UpdateStepPlanService.h>

// actions
#include <vigir_footstep_planning_msgs/EditStepAction.h>
#include <vigir_footstep_planning_msgs/ExecuteStepPlanAction.h>
#include <vigir_footstep_planning_msgs/GenerateFeetPoseAction.h>
#include <vigir_footstep_planning_msgs/GeneratePatternAction.h>
#include <vigir_footstep_planning_msgs/GetAllParameterSetsAction.h>
#include <vigir_footstep_planning_msgs/GetParameterSetAction.h>
#include <vigir_footstep_planning_msgs/GetParameterSetNamesAction.h>
#include <vigir_footstep_planning_msgs/GetStepPlanAction.h>
#include <vigir_footstep_planning_msgs/SetParameterSetAction.h>
#include <vigir_footstep_planning_msgs/SetStepPlanAction.h>
#include <vigir_footstep_planning_msgs/StepPlanRequestAction.h>
#include <vigir_footstep_planning_msgs/StitchStepPlanAction.h>
#include <vigir_footstep_planning_msgs/UpdateFeetAction.h>
#include <vigir_footstep_planning_msgs/UpdateFootAction.h>
#include <vigir_footstep_planning_msgs/UpdateStepPlanAction.h>



namespace vigir_footstep_planning
{
namespace msgs
{
using namespace vigir_footstep_planning_msgs;
}

// Extension to ErrorStatus message
msgs::ErrorStatus operator+(const msgs::ErrorStatus& lhs, const msgs::ErrorStatus& rhs);
msgs::ErrorStatus operator+=(msgs::ErrorStatus& lhs, const msgs::ErrorStatus& rhs);

msgs::ErrorStatus isConsistent(const msgs::StepPlan& result);

std::string ErrorStatusCodeToString(unsigned int error);
std::string WarningStatusCodeToString(unsigned int warning);

msgs::ErrorStatus createErrorStatus(const std::string& context, unsigned int error, const std::string& error_msg, unsigned int warning, const std::string& warning_msg, bool output = true);
msgs::ErrorStatus ErrorStatusError(unsigned int error, const std::string& context, const std::string& error_msg, bool output = true);
msgs::ErrorStatus ErrorStatusWarning(unsigned int warning, const std::string& context, const std::string& warning_msg, bool output = true);

bool hasError(const msgs::ErrorStatus& status);
bool hasWarning(const msgs::ErrorStatus& status);
bool isOk(const msgs::ErrorStatus& status);

std::string toString(const msgs::ErrorStatus& error_status);

std::string toString(const msgs::FootstepExecutionStatus& execution_status);

// some helper
std::string toString(const XmlRpc::XmlRpcValue& val);
std::string toString(const XmlRpc::XmlRpcValue::Type& type);

std::string& strip(std::string& s, const char c);
std::string strip_const(const std::string& s, const char c);

template <typename Tin, typename Tout>
void copyPosition(const Tin &p_in, Tout &p_out)
{
  p_out.x = p_in.x;
  p_out.y = p_in.y;
  p_out.z = p_in.z;
}
}

#endif
