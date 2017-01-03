//=================================================================================================
// Copyright (c) 2017, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_FOOTSTEP_PLANNING_VISUALIZATION_H__
#define VIGIR_FOOTSTEP_PLANNING_VISUALIZATION_H__

#include <ros/ros.h>
#include <tf/tf.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>



namespace vigir_footstep_planning
{
namespace msgs
{
void footToFootMarker(const Foot& foot, const geometry_msgs::Vector3& foot_size, const std_msgs::ColorRGBA& color, visualization_msgs::Marker& marker);
void feetToFootMarkerArray(const Feet& feet, const geometry_msgs::Vector3& foot_size, const std_msgs::ColorRGBA& color, visualization_msgs::MarkerArray& marker_array);
void stepToFootMarker(const Step& step, const geometry_msgs::Vector3& foot_size, const std_msgs::ColorRGBA& color, visualization_msgs::Marker& marker);
void stepPlanToFootMarkerArray(const std::vector<Step>& steps, const geometry_msgs::Vector3& foot_size, visualization_msgs::MarkerArray& marker_array, bool add_step_index = true);
void stepPlanToFootMarkerArray(const StepPlan& step_plan, const geometry_msgs::Vector3& foot_size, visualization_msgs::MarkerArray& marker_array, bool add_step_index = true);

void feetToUpperBodyMarker(const Feet& feet, const geometry_msgs::Vector3& upper_body_size, const geometry_msgs::Vector3& upper_body_origin_shift, const std_msgs::ColorRGBA& color, visualization_msgs::Marker& marker, bool flat = false);
void stepPlanToUpperBodyMarkerArray(const std::vector<Step>& steps, const geometry_msgs::Vector3& upper_body_size, const geometry_msgs::Vector3& upper_body_origin_shift, visualization_msgs::MarkerArray& marker_array, bool add_step_index = true);
void stepPlanToUpperBodyMarkerArray(const StepPlan& step_plan, const geometry_msgs::Vector3& upper_body_size, const geometry_msgs::Vector3& upper_body_origin_shift, visualization_msgs::MarkerArray& marker_array, bool add_step_index = true);

void stepPlanToPath(const StepPlan& step_plan, nav_msgs::Path& path);
}
}

#endif
