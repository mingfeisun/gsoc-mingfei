/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>
#include "sdf/JointAxis.hh"

/////////////////////////////////////////////////
TEST(DOMJointAxis, Construction)
{
  sdf::JointAxis axis;
  EXPECT_EQ(nullptr, axis.Element());
  EXPECT_DOUBLE_EQ(0.0, axis.InitialPosition());
  EXPECT_EQ(ignition::math::Vector3d::UnitZ, axis.Xyz());
  EXPECT_FALSE(axis.UseParentModelFrame());
  EXPECT_DOUBLE_EQ(0.0, axis.Damping());
  EXPECT_DOUBLE_EQ(0.0, axis.Friction());
  EXPECT_DOUBLE_EQ(0.0, axis.SpringReference());
  EXPECT_DOUBLE_EQ(0.0, axis.SpringStiffness());
  EXPECT_DOUBLE_EQ(-1e16, axis.Lower());
  EXPECT_DOUBLE_EQ(1e16, axis.Upper());
  EXPECT_DOUBLE_EQ(-1, axis.Effort());
  EXPECT_DOUBLE_EQ(-1, axis.MaxVelocity());
  EXPECT_DOUBLE_EQ(1e8, axis.Stiffness());
  EXPECT_DOUBLE_EQ(1.0, axis.Dissipation());

  axis.SetInitialPosition(1.2);
  EXPECT_DOUBLE_EQ(1.2, axis.InitialPosition());

  axis.SetXyz(ignition::math::Vector3d(0, 1, 0));
  EXPECT_EQ(ignition::math::Vector3d::UnitY, axis.Xyz());

  axis.SetUseParentModelFrame(true);
  EXPECT_TRUE(axis.UseParentModelFrame());

  axis.SetDamping(0.2);
  EXPECT_DOUBLE_EQ(0.2, axis.Damping());

  axis.SetFriction(1.3);
  EXPECT_DOUBLE_EQ(1.3, axis.Friction());

  axis.SetSpringReference(2.4);
  EXPECT_DOUBLE_EQ(2.4, axis.SpringReference());

  axis.SetSpringStiffness(-1.2);
  EXPECT_DOUBLE_EQ(-1.2, axis.SpringStiffness());

  axis.SetLower(-10.8);
  EXPECT_DOUBLE_EQ(-10.8, axis.Lower());

  axis.SetUpper(123.4);
  EXPECT_DOUBLE_EQ(123.4, axis.Upper());

  axis.SetEffort(3.2);
  EXPECT_DOUBLE_EQ(3.2, axis.Effort());

  axis.SetMaxVelocity(54.2);
  EXPECT_DOUBLE_EQ(54.2, axis.MaxVelocity());

  axis.SetStiffness(1e2);
  EXPECT_DOUBLE_EQ(1e2, axis.Stiffness());

  axis.SetDissipation(1.5);
  EXPECT_DOUBLE_EQ(1.5, axis.Dissipation());
}

/////////////////////////////////////////////////
TEST(DOMJointAxis, CopyConstructor)
{
  sdf::JointAxis jointAxis;
  jointAxis.SetXyz(ignition::math::Vector3d(0, 1, 0));

  sdf::JointAxis jointAxisCopy(jointAxis);
  EXPECT_EQ(jointAxis.Xyz(), jointAxisCopy.Xyz());
}

/////////////////////////////////////////////////
TEST(DOMJointAxis, AssignmentOperator)
{
  sdf::JointAxis jointAxis;
  jointAxis.SetXyz(ignition::math::Vector3d(0, 1, 0));

  sdf::JointAxis jointAxisCopy;
  jointAxisCopy = jointAxis;
  EXPECT_EQ(jointAxis.Xyz(), jointAxisCopy.Xyz());
}

/////////////////////////////////////////////////
TEST(DOMJointAxis, MoveConstructor)
{
  ignition::math::Vector3d axis{0, 1, 0};
  sdf::JointAxis jointAxis;
  jointAxis.SetXyz(axis);

  sdf::JointAxis jointAxisMoved(std::move(jointAxis));
  EXPECT_EQ(axis, jointAxisMoved.Xyz());
}

/////////////////////////////////////////////////
TEST(DOMJointAxis, MoveAssignmentOperator)
{
  ignition::math::Vector3d axis{0, 1, 0};
  sdf::JointAxis jointAxis;
  jointAxis.SetXyz(axis);

  sdf::JointAxis jointAxisMoved;
  jointAxisMoved = std::move(jointAxis);
  EXPECT_EQ(axis, jointAxisMoved.Xyz());
}

/////////////////////////////////////////////////
TEST(DOMJointAxis, CopyAssignmentAfterMove)
{
  ignition::math::Vector3d axis1{0, 1, 0};
  sdf::JointAxis jointAxis1;
  jointAxis1.SetXyz(axis1);

  ignition::math::Vector3d axis2{1, 0, 0};
  sdf::JointAxis jointAxis2;
  jointAxis2.SetXyz(axis2);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::JointAxis tmp = std::move(jointAxis1);
  jointAxis1 = jointAxis2;
  jointAxis2 = tmp;

  EXPECT_EQ(axis2, jointAxis1.Xyz());
  EXPECT_EQ(axis1, jointAxis2.Xyz());
}
