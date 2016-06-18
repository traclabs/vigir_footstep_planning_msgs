#include <vigir_footstep_planning_msgs/msgs_test_helper.h>

#include <gtest/gtest.h>



namespace vigir_footstep_planning
{
void isEqualTest(const std_msgs::String& exp, const std_msgs::String& res)
{
  EXPECT_EQ(exp.data, res.data);
}

void isEqualTest(const std_msgs::Header& exp, const std_msgs::Header& res)
{
  EXPECT_EQ(exp.seq, res.seq);
  EXPECT_EQ(exp.stamp, res.stamp);
  EXPECT_EQ(exp.frame_id, res.frame_id);
}

void isEqualTest(const geometry_msgs::Point& exp, const geometry_msgs::Point& res)
{
  EXPECT_DOUBLE_EQ(exp.x, res.x);
  EXPECT_DOUBLE_EQ(exp.y, res.y);
  EXPECT_DOUBLE_EQ(exp.z, res.z);
}

void isEqualTest(const geometry_msgs::Quaternion& exp, const geometry_msgs::Quaternion& res)
{
  EXPECT_DOUBLE_EQ(exp.x, res.x);
  EXPECT_DOUBLE_EQ(exp.y, res.y);
  EXPECT_DOUBLE_EQ(exp.z, res.z);
  EXPECT_DOUBLE_EQ(exp.w, res.w);
}

void isEqualTest(const geometry_msgs::Pose& exp, const geometry_msgs::Pose& res)
{
  isEqualTest(exp.position, res.position);
  isEqualTest(exp.orientation, res.orientation);
}

void isEqualTest(const msgs::Foot& exp, const msgs::Foot& res)
{
  isEqualTest(exp.header, res.header);
  EXPECT_EQ(exp.foot_index, res.foot_index);
  isEqualTest(exp.pose, res.pose);
}

void isEqualTest(const msgs::Feet& exp, const msgs::Feet& res)
{
  isEqualTest(exp.header, res.header);
  isEqualTest(exp.left, res.left);
  isEqualTest(exp.right, res.right);
}

void isEqualTest(const msgs::Step& exp, const msgs::Step& res)
{
  isEqualTest(exp.header, res.header);
  isEqualTest(exp.foot, res.foot);
  EXPECT_EQ(exp.step_index, res.step_index);
  EXPECT_FLOAT_EQ(exp.cost, res.cost);
  EXPECT_FLOAT_EQ(exp.risk, res.risk);
  EXPECT_EQ(exp.valid, res.valid);
  EXPECT_EQ(exp.colliding, res.colliding);
  EXPECT_EQ(exp.locked, res.locked);
  EXPECT_EQ(exp.modified, res.modified);
  EXPECT_FLOAT_EQ(exp.sway_duration, res.sway_duration);
  EXPECT_FLOAT_EQ(exp.step_duration, res.step_duration);
  EXPECT_FLOAT_EQ(exp.swing_height, res.swing_height);
}

void isEqualTest(const msgs::StepPlan& exp, const msgs::StepPlan& res)
{
  isEqualTest(exp.header, res.header);
  isEqualTest(exp.parameter_set_name, res.parameter_set_name);
  isEqualTest(exp.start, res.start);
  isEqualTest(exp.goal, res.goal);

  ASSERT_EQ(exp.steps.size(), res.steps.size());
  for (size_t i = 0; i < exp.steps.size(); i++)
    isEqualTest(exp.steps[i], res.steps[i]);

  EXPECT_EQ(exp.mode, res.mode);
}

void isEqualTest(const StepPlan& exp, const StepPlan& res)
{
  msgs::StepPlan exp_steps;
  exp.toMsg(exp_steps);

  msgs::StepPlan res_steps;
  res.toMsg(res_steps);

  isEqualTest(exp_steps, res_steps);
}
}
