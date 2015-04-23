#include <vigir_footstep_planning_msgs/visualization.h>

namespace vigir_footstep_planning
{
namespace msgs
{
void footToFootMarker(const Foot& foot, const geometry_msgs::Vector3& foot_size, const std_msgs::ColorRGBA& color, visualization_msgs::Marker& marker)
{
  marker.header = foot.header;
  marker.ns = "vigir_footstep_planning/step_plan";
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose = foot.pose;
  marker.pose.position.z += 0.5*foot_size.z;
  marker.scale = foot_size;
  marker.color = color;

  marker.lifetime = ros::Duration();
}

void feetToFootMarkerArray(const Feet& feet, const geometry_msgs::Vector3& foot_size, const std_msgs::ColorRGBA& color, visualization_msgs::MarkerArray& marker_array)
{
  visualization_msgs::Marker marker;
  marker_array.markers.clear();

  footToFootMarker(feet.left, foot_size, color, marker);
  marker_array.markers.push_back(marker);
  marker.id++;

  footToFootMarker(feet.right, foot_size, color, marker);
  marker_array.markers.push_back(marker);
}

void stepToFootMarker(const Step& step, const geometry_msgs::Vector3& foot_size, const std_msgs::ColorRGBA& color, visualization_msgs::Marker& marker)
{
  footToFootMarker(step.foot, foot_size, color, marker);
}

void stepPlanToFootMarkerArray(const std::vector<Step>& steps, const geometry_msgs::Vector3& foot_size, visualization_msgs::MarkerArray& marker_array, bool add_step_index)
{
  std_msgs::ColorRGBA color;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 0.0;
  color.a = 0.6;

  visualization_msgs::Marker marker;

  marker_array.markers.clear();

  for (std::vector<Step>::const_iterator itr = steps.begin(); itr != steps.end(); itr++)
  {
    // colorize
    color.r = itr->foot.foot_index == Foot::LEFT ? 1.0 : 0.0;
    color.g = itr->foot.foot_index == Foot::LEFT ? 0.0 : 1.0;

    // transform
    stepToFootMarker(*itr, foot_size, color, marker);
    marker_array.markers.push_back(marker);
    marker.id++;

    // add text
    if (add_step_index)
    {
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.text = boost::lexical_cast<std::string>(itr->step_index);
      marker.scale.z *= 3;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 0.7;

      marker_array.markers.push_back(marker);
      marker.id++;
    }
  }
}

void stepPlanToFootMarkerArray(const StepPlan& step_plan, const geometry_msgs::Vector3& foot_size, visualization_msgs::MarkerArray& marker_array, bool add_step_index)
{
  stepPlanToFootMarkerArray(step_plan.steps, foot_size, marker_array, add_step_index);
}

void feetToUpperBodyMarker(const Feet& feet, const geometry_msgs::Vector3& upper_body_size, const geometry_msgs::Vector3& upper_body_origin_shift, const std_msgs::ColorRGBA& color, visualization_msgs::Marker& marker, bool flat)
{
  marker.header = feet.header;
  marker.ns = "vigir_footstep_planning/upper_body";
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  // approximate upper body position
  marker.pose.position.x = 0.5 * (feet.left.pose.position.x + feet.right.pose.position.x);
  marker.pose.position.y = 0.5 * (feet.left.pose.position.y + feet.right.pose.position.y);
  marker.pose.position.z = 0.5 * (feet.left.pose.position.z + feet.right.pose.position.z);
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.5 * (tf::getYaw(feet.left.pose.orientation) + tf::getYaw(feet.right.pose.orientation)));

  // determine shift of polygon based on orientation
  tf::Transform transform;
  tf::poseMsgToTF(marker.pose, transform);
  tf::Vector3 shifted;
  tf::vector3MsgToTF(upper_body_origin_shift, shifted);
  shifted = transform * shifted;

  marker.pose.position.x = shifted.getX();
  marker.pose.position.y = shifted.getY();
  marker.pose.position.z = shifted.getZ();

  // finalize marker
  marker.pose.position.z += flat ? 0.01 : 0.5*upper_body_size.z;
  marker.scale = upper_body_size;
  if (flat)
    marker.scale.z = 0.02;
  marker.color = color;

  marker.lifetime = ros::Duration();
}

void stepPlanToUpperBodyMarkerArray(const std::vector<Step>& steps, const geometry_msgs::Vector3& upper_body_size, const geometry_msgs::Vector3& upper_body_origin_shift, visualization_msgs::MarkerArray& marker_array, bool add_step_index)
{
  std_msgs::ColorRGBA color;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 0.4;
  color.a = 0.2;

  visualization_msgs::Marker marker;

  marker_array.markers.clear();

  Feet feet;
  for (std::vector<Step>::const_iterator itr = steps.begin(); itr != steps.end(); itr++)
  {
    const Step& step = *itr;
    if (step.foot.foot_index == Foot::LEFT)
      feet.left = step.foot;
    else
      feet.right = step.foot;

    if (itr == steps.begin())
    {
      feet.header = step.header;
      continue;
    }

    // transform
    feetToUpperBodyMarker(feet, upper_body_size, upper_body_origin_shift, color, marker);
    marker_array.markers.push_back(marker);
    marker.id++;

    // add text
    if (add_step_index)
    {
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.text = boost::lexical_cast<std::string>(itr->step_index);
      marker.scale.z *= 0.1;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 0.7;

      marker_array.markers.push_back(marker);
      marker.id++;
    }
  }
}

void stepPlanToUpperBodyMarkerArray(const StepPlan& step_plan, const geometry_msgs::Vector3& upper_body_size, const geometry_msgs::Vector3& upper_body_origin_shift, visualization_msgs::MarkerArray& marker_array, bool add_step_index)
{
  stepPlanToUpperBodyMarkerArray(step_plan.steps, upper_body_size, upper_body_origin_shift, marker_array, add_step_index);
}

void stepPlanToPath(const StepPlan& step_plan, nav_msgs::Path& path)
{
  path.poses.clear();

  path.header = step_plan.header;
  for (std::vector<Step>::const_iterator itr = step_plan.steps.begin(); itr != step_plan.steps.end(); itr++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = step_plan.header;
    pose.pose = itr->foot.pose;
    path.poses.push_back(pose);
  }
}
}
}
