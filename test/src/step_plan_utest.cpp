#include <ros/ros.h>

#include <math.h>

#include <gtest/gtest.h>

#include <vigir_footstep_planning_msgs/step_plan.h>
#include <vigir_footstep_planning_msgs/msgs_test_helper.h>



using namespace vigir_footstep_planning;

msgs::StepPlan genStepPlan(geometry_msgs::Pose start, unsigned int steps, unsigned int foot_index = msgs::Foot::LEFT, unsigned int start_step_index = 0, bool closing_step = true)
{
  msgs::StepPlan step_plan;

  double yaw = tf::getYaw(start.orientation);

  // generate initial step
  msgs::Step step;
  step.foot.foot_index = foot_index;
  step.foot.pose = start;
  step.step_index = start_step_index;
  step.valid = true;

  step_plan.steps.push_back(step);

  // add extra step when closing step is set
  if (closing_step)
    steps++;

  for (unsigned int i = 0; i < steps; i++)
  {
    // alternate left and right foot pose
    if (step.foot.foot_index == msgs::Foot::RIGHT)
    {
      step.foot.foot_index = msgs::Foot::LEFT;
      step.foot.pose.position.x += 0.2 * -sin(yaw);
      step.foot.pose.position.y += 0.2 * cos(yaw);
    }
    else
    {
      step.foot.foot_index = msgs::Foot::RIGHT;
      step.foot.pose.position.x += -0.2 * -sin(yaw);
      step.foot.pose.position.y += -0.2 * cos(yaw);
    }

    // forward movement
    // closing step handling
    if (!closing_step || i < steps-1)
    {
      step.foot.pose.position.x += 0.25 * cos(yaw);
      step.foot.pose.position.y += 0.25 * sin(yaw);
    }

    step.step_index++;

    step_plan.steps.push_back(step);
  }
  return step_plan;
}

// Test for checking if appendStepPlan generates correct results
TEST(StepPlan, appendStepPlan)
{
  geometry_msgs::Pose start;
  start.position.x = 1.0;
  start.position.z = 1.0;
  start.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // generate plan 1: 4 steps, no closing step
  msgs::StepPlan step_plan_1 = genStepPlan(start, 4, msgs::Foot::LEFT, 0, false);

  // generate plan 2: 6 steps, starting from last step of step_plan_1
  msgs::StepPlan step_plan_2 = genStepPlan(step_plan_1.steps.back().foot.pose, 6, msgs::Foot::LEFT);
  step_plan_2.steps.erase(step_plan_2.steps.begin());

  // append plan 1 and 2
  StepPlan step_plan_result(step_plan_1);
  step_plan_result.appendStepPlan(step_plan_2);
  ASSERT_EQ(12, step_plan_result.size());

  // generate expected step plan
  StepPlan step_plan_exp(genStepPlan(start, 10));
  ASSERT_EQ(12, step_plan_exp.size());

  // compare
  isEqualTest(step_plan_exp, step_plan_result);
}

// Test for checking if updateStepPlan generates correct results
TEST(StepPlan, updateStepPlan)
{
  geometry_msgs::Pose start;
  start.position.x = 1.0;
  start.position.z = 1.0;
  start.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // generate plan 1: 6 steps
  msgs::StepPlan step_plan_1 = genStepPlan(start, 6, msgs::Foot::LEFT);

  // generate plan 2: 8 steps, starting from step #3 of step_plan_1
  msgs::StepPlan step_plan_2 = genStepPlan(step_plan_1.steps[3].foot.pose, 8, msgs::Foot::RIGHT, 3);

  // update plan 1 with 2
  StepPlan step_plan_result(step_plan_1);
  step_plan_result.updateStepPlan(step_plan_2);
  ASSERT_EQ(13, step_plan_result.size());

  // generate expected step plan
  StepPlan step_plan_exp(genStepPlan(start, 11));
  ASSERT_EQ(13, step_plan_exp.size());

  for (size_t i = 3; i < 8; i++)
  {
    msgs::Step step;
    EXPECT_TRUE(step_plan_exp.getStep(step, i));
    step_plan_exp.updateStep(step);
  }

  // compare
  isEqualTest(step_plan_exp, step_plan_result);
}

// Test for checking if stitchStepPlan generates correct results
TEST(StepPlan, stitchStepPlan)
{
  geometry_msgs::Pose start_1;
  start_1.position.x = 1.0;
  start_1.position.y = 1.1;
  start_1.position.z = 1.0;
  start_1.orientation = tf::createQuaternionMsgFromYaw(M_PI_4);

  // generate plan 1: 6 steps
  msgs::StepPlan step_plan_1 = genStepPlan(start_1, 6);

  geometry_msgs::Pose start_2;
  start_2.position.x = 2.1;
  start_2.position.y = 1.0;
  start_2.position.z = 1.0;
  start_2.orientation = tf::createQuaternionMsgFromYaw(1.0);

  // generate plan 2: 6 steps
  msgs::StepPlan step_plan_2 = genStepPlan(start_2, 6, msgs::Foot::RIGHT, 3);

  // stitch plan 1 with 2
  StepPlan step_plan_result(step_plan_1);
  step_plan_result.stitchStepPlan(step_plan_2, 3);
  ASSERT_EQ(11, step_plan_result.size());

  // generate expected step plan
  StepPlan step_plan_exp(genStepPlan(start_1, 9));
  ASSERT_EQ(11, step_plan_exp.size());

  for (size_t i = 3; i < 8; i++)
  {
    msgs::Step step;
    EXPECT_TRUE(step_plan_exp.getStep(step, i));
    step_plan_exp.updateStep(step);
  }

  // compare
  isEqualTest(step_plan_exp, step_plan_result);
}
