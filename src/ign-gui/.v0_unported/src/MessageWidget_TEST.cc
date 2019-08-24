/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <ignition/common/Console.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/msgs.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "ignition/gui/BoolWidget.hh"
#include "ignition/gui/ColorWidget.hh"
#include "ignition/gui/CollapsibleWidget.hh"
#include "ignition/gui/EnumWidget.hh"
#include "ignition/gui/GeometryWidget.hh"
#include "ignition/gui/Iface.hh"
#include "ignition/gui/NumberWidget.hh"
#include "ignition/gui/Pose3dWidget.hh"
#include "ignition/gui/PropertyWidget.hh"
#include "ignition/gui/QtMetatypes.hh"
#include "ignition/gui/StringWidget.hh"
#include "ignition/gui/Vector3dWidget.hh"

#include "ignition/gui/MessageWidget.hh"

using namespace ignition;
using namespace gui;

/////////////////////////////////////////////////
TEST(MessageWidgetTest, ConstructAndUpdate)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Invalid constructor
  {
    auto widget = new MessageWidget(nullptr);
    ASSERT_NE(widget, nullptr);

    EXPECT_FALSE(widget->UpdateFromMsg(new msgs::StringMsg()));

    delete widget;
  }

  // Valid constructor, invalid update
  {
    auto widget = new MessageWidget(new msgs::StringMsg());
    ASSERT_NE(widget, nullptr);

    EXPECT_FALSE(widget->UpdateFromMsg(nullptr));

    delete widget;
  }

  // Update type different from constructor
  {
    auto widget = new MessageWidget(new msgs::StringMsg());
    ASSERT_NE(widget, nullptr);

    EXPECT_FALSE(widget->UpdateFromMsg(new msgs::Int32()));

    delete widget;
  }

  // Same type as constructor
  {
    auto widget = new MessageWidget(new msgs::StringMsg());
    ASSERT_NE(widget, nullptr);

    EXPECT_TRUE(widget->UpdateFromMsg(new msgs::StringMsg()));

    delete widget;
  }

  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
TEST(MessageWidgetTest, JointMsgWidget)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  msgs::Joint msg;
  {
    // joint
    msg.set_name("test_joint");
    msg.set_id(1122u);
    msg.set_parent("test_joint_parent");
    msg.set_parent_id(212121u);
    msg.set_child("test_joint_child");
    msg.set_child_id(454545u);

    // type
    msg.set_type(msgs::ConvertJointType("revolute"));

    // pose
    math::Vector3d pos(4.0, -1.0, 3.5);
    math::Quaterniond quat(0.0, 1.57, 0.0);
    msgs::Set(msg.mutable_pose(), math::Pose3d(pos, quat));

    // axis1
    auto axisMsg = msg.mutable_axis1();
    msgs::Set(axisMsg->mutable_xyz(), math::Vector3d::UnitX);
    axisMsg->set_use_parent_model_frame(false);
    axisMsg->set_limit_lower(-999.0);
    axisMsg->set_limit_upper(999.0);
    axisMsg->set_limit_effort(-1.0);
    axisMsg->set_limit_velocity(-1.0);
    axisMsg->set_damping(0.0);

    // other joint physics properties
    msg.set_cfm(0.2);
    msg.set_bounce(0.3);
    msg.set_velocity(0.4);
    msg.set_fudge_factor(0.5);
    msg.set_limit_cfm(0.6);
    msg.set_limit_erp(0.7);
    msg.set_suspension_cfm(0.8);
    msg.set_suspension_erp(0.9);
  }

  // Create widget
  auto widget = new MessageWidget(&msg);
  ASSERT_NE(widget, nullptr);

  // Retrieve message
  {
    auto retMsg = dynamic_cast<msgs::Joint *>(widget->Msg());
    ASSERT_NE(retMsg, nullptr);

    // joint
    EXPECT_EQ(retMsg->name(), "test_joint");
    EXPECT_EQ(retMsg->id(), 1122u);
    EXPECT_EQ(retMsg->parent(), "test_joint_parent");
    EXPECT_EQ(retMsg->parent_id(), 212121u);
    EXPECT_EQ(retMsg->child(), "test_joint_child");
    EXPECT_EQ(retMsg->child_id(), 454545u);

    // type
    EXPECT_EQ(retMsg->type(), msgs::ConvertJointType("revolute"));

    // pose
    auto poseMsg = retMsg->pose();
    auto posMsg = poseMsg.position();
    EXPECT_DOUBLE_EQ(posMsg.x(), 4.0);
    EXPECT_DOUBLE_EQ(posMsg.y(), -1.0);
    EXPECT_DOUBLE_EQ(posMsg.z(), 3.5);
    auto quat = msgs::Convert(poseMsg.orientation());
    EXPECT_DOUBLE_EQ(quat.Euler().X(), 0.0);
    EXPECT_LT(fabs(quat.Euler().Y() - 1.57), 0.0001);
    EXPECT_DOUBLE_EQ(quat.Euler().Z(), 0.0);

    // axis1
    auto axisMsg = msg.mutable_axis1();
    EXPECT_DOUBLE_EQ(axisMsg->xyz().x(), 1.0);
    EXPECT_DOUBLE_EQ(axisMsg->xyz().y(), 0.0);
    EXPECT_DOUBLE_EQ(axisMsg->xyz().z(), 0.0);
    EXPECT_DOUBLE_EQ(axisMsg->use_parent_model_frame(), false);
    EXPECT_DOUBLE_EQ(axisMsg->limit_lower(), -999.0);
    EXPECT_DOUBLE_EQ(axisMsg->limit_upper(), 999.0);
    EXPECT_DOUBLE_EQ(axisMsg->limit_effort(), -1.0);
    EXPECT_DOUBLE_EQ(axisMsg->limit_velocity(), -1.0);
    EXPECT_DOUBLE_EQ(axisMsg->damping(), 0.0);

    // other joint physics properties
    EXPECT_DOUBLE_EQ(retMsg->cfm(), 0.2);
    EXPECT_DOUBLE_EQ(retMsg->bounce(), 0.3);
    EXPECT_DOUBLE_EQ(retMsg->velocity(), 0.4);
    EXPECT_DOUBLE_EQ(retMsg->fudge_factor(), 0.5);
    EXPECT_DOUBLE_EQ(retMsg->limit_cfm(), 0.6);
    EXPECT_DOUBLE_EQ(retMsg->limit_erp(), 0.7);
    EXPECT_DOUBLE_EQ(retMsg->suspension_cfm(), 0.8);
    EXPECT_DOUBLE_EQ(retMsg->suspension_erp(), 0.9);
  }

  // Expand all widgets so they're generated
  widget->ToggleAll(true);

  // update fields in the message widget and
  // verify that the new message contains the updated values.
  // Joint type revolute -> universal
  {
    // joint
    EXPECT_TRUE(widget->SetPropertyValue("name", QVariant::fromValue(
        std::string("test_joint_updated"))));
    EXPECT_TRUE(widget->SetPropertyValue("id", 9999999u));
    EXPECT_TRUE(widget->SetPropertyValue("parent", QVariant::fromValue(
        std::string("test_joint_parent_updated"))));
    EXPECT_TRUE(widget->SetPropertyValue("parent_id", 1u));
    EXPECT_TRUE(widget->SetPropertyValue("child", QVariant::fromValue(
        std::string("test_joint_child_updated"))));
    EXPECT_TRUE(widget->SetPropertyValue("child_id", 2u));

    // type
    EXPECT_TRUE(widget->SetPropertyValue("type", QVariant::fromValue(
        msgs::Joint_Type_Name(msgs::Joint_Type_UNIVERSAL))));

    // pose
    math::Vector3d pos(2.0, 9.0, -4.0);
    math::Quaterniond quat(0.0, 0.0, 1.57);
    EXPECT_TRUE(widget->SetPropertyValue("pose", QVariant::fromValue(
        math::Pose3d(pos, quat))));

    // axis1
    EXPECT_TRUE(widget->SetPropertyValue("axis1::xyz", QVariant::fromValue(
        math::Vector3d::UnitY)));
    EXPECT_TRUE(widget->SetPropertyValue("axis1::use_parent_model_frame",
        true));
    EXPECT_TRUE(widget->SetPropertyValue("axis1::limit_lower", -1.2));
    EXPECT_TRUE(widget->SetPropertyValue("axis1::limit_upper", -1.0));
    EXPECT_TRUE(widget->SetPropertyValue("axis1::limit_effort", 1.0));
    EXPECT_TRUE(widget->SetPropertyValue("axis1::limit_velocity", 100.0));
    EXPECT_TRUE(widget->SetPropertyValue("axis1::damping", 0.9));

    // axis2
    EXPECT_TRUE(widget->SetPropertyValue("axis2::xyz", QVariant::fromValue(
        math::Vector3d::UnitZ)));
    EXPECT_TRUE(widget->SetPropertyValue("axis2::use_parent_model_frame",
        true));
    EXPECT_TRUE(widget->SetPropertyValue("axis2::limit_lower", -3.2));
    EXPECT_TRUE(widget->SetPropertyValue("axis2::limit_upper", -3.0));
    EXPECT_TRUE(widget->SetPropertyValue("axis2::limit_effort", 3.0));
    EXPECT_TRUE(widget->SetPropertyValue("axis2::limit_velocity", 300.0));
    EXPECT_TRUE(widget->SetPropertyValue("axis2::damping", 3.9));

    // other joint physics properties
    EXPECT_TRUE(widget->SetPropertyValue("cfm", 0.9));
    EXPECT_TRUE(widget->SetPropertyValue("bounce", 0.8));
    EXPECT_TRUE(widget->SetPropertyValue("velocity", 0.7));
    EXPECT_TRUE(widget->SetPropertyValue("fudge_factor", 0.6));
    EXPECT_TRUE(widget->SetPropertyValue("limit_cfm", 0.5));
    EXPECT_TRUE(widget->SetPropertyValue("limit_erp", 0.4));
    EXPECT_TRUE(widget->SetPropertyValue("suspension_cfm", 0.3));
    EXPECT_TRUE(widget->SetPropertyValue("suspension_erp", 0.2));
  }

  // verify widget values
  {
    // joint
    EXPECT_EQ(widget->PropertyValue("name").value<std::string>(),
        "test_joint_updated");
    EXPECT_EQ(widget->PropertyValue("id"), 9999999u);
    EXPECT_EQ(widget->PropertyValue("parent").value<std::string>(),
        "test_joint_parent_updated");
    EXPECT_EQ(widget->PropertyValue("parent_id"), 1u);
    EXPECT_EQ(widget->PropertyValue("child").value<std::string>(),
        "test_joint_child_updated");
    EXPECT_EQ(widget->PropertyValue("child_id"), 2u);

    // type
    EXPECT_TRUE(widget->SetPropertyValue("type", QVariant::fromValue(
        msgs::Joint_Type_Name(msgs::Joint_Type_UNIVERSAL))));

    // pose
    math::Vector3d pos(2.0, 9.0, -4.0);
    math::Quaterniond quat(0.0, 0.0, 1.57);
    EXPECT_EQ(widget->PropertyValue("pose").value<math::Pose3d>(),
        math::Pose3d(pos, quat));

    // axis1
    EXPECT_EQ(widget->PropertyValue(
        "axis1::xyz").value<math::Vector3d>(), math::Vector3d::UnitY);
    EXPECT_EQ(widget->PropertyValue(
        "axis1::use_parent_model_frame").toBool(), true);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "axis1::limit_lower").toDouble(), -1.2);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "axis1::limit_upper").toDouble(), -1.0);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "axis1::limit_effort").toDouble(), 1.0);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "axis1::limit_velocity").toDouble(), 100.0);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "axis1::damping").toDouble(), 0.9);

    // axis2
    EXPECT_EQ(widget->PropertyValue(
        "axis2::xyz").value<math::Vector3d>(), math::Vector3d::UnitZ);
    EXPECT_EQ(widget->PropertyValue(
        "axis1::use_parent_model_frame").toBool(), true);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "axis2::limit_lower").toDouble(), -3.2);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "axis2::limit_upper").toDouble(), -3.0);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "axis2::limit_effort").toDouble(), 3.0);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "axis2::limit_velocity").toDouble(), 300.0);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "axis2::damping").toDouble(), 3.9);

    // other joint physics properties
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "cfm").toDouble(), 0.9);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "bounce").toDouble(), 0.8);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "velocity").toDouble(), 0.7);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "fudge_factor").toDouble(), 0.6);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "limit_cfm").toDouble(), 0.5);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "limit_erp").toDouble(), 0.4);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "suspension_cfm").toDouble(), 0.3);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "suspension_erp").toDouble(), 0.2);
  }

  // verify updates in new msg
  {
    auto retJointMsg = dynamic_cast<msgs::Joint *>(widget->Msg());
    EXPECT_TRUE(retJointMsg != nullptr);

    // joint
    EXPECT_EQ(retJointMsg->name(), "test_joint_updated");
    EXPECT_EQ(retJointMsg->id(), 9999999u);
    EXPECT_EQ(retJointMsg->parent(), "test_joint_parent_updated");
    EXPECT_EQ(retJointMsg->parent_id(), 1u);
    EXPECT_EQ(retJointMsg->child(), "test_joint_child_updated");
    EXPECT_EQ(retJointMsg->child_id(), 2u);

    // type
    EXPECT_EQ(retJointMsg->type(), msgs::ConvertJointType("universal"));

    // pose
    auto poseMsg = retJointMsg->pose();
    auto posMsg = poseMsg.position();
    EXPECT_DOUBLE_EQ(posMsg.x(), 2.0);
    EXPECT_DOUBLE_EQ(posMsg.y(), 9.0);
    EXPECT_DOUBLE_EQ(posMsg.z(), -4.0);
    auto quat = msgs::Convert(poseMsg.orientation());
    EXPECT_DOUBLE_EQ(quat.Euler().X(), 0.0);
    EXPECT_DOUBLE_EQ(quat.Euler().Y(), 0.0);
    EXPECT_DOUBLE_EQ(quat.Euler().Z(), 1.57);

    // axis1
    auto axisMsg = retJointMsg->mutable_axis1();
    EXPECT_DOUBLE_EQ(axisMsg->xyz().x(), 0.0);
    EXPECT_DOUBLE_EQ(axisMsg->xyz().y(), 1.0);
    EXPECT_DOUBLE_EQ(axisMsg->xyz().z(), 0.0);
    EXPECT_EQ(axisMsg->use_parent_model_frame(), true);
    EXPECT_DOUBLE_EQ(axisMsg->limit_lower(), -1.2);
    EXPECT_DOUBLE_EQ(axisMsg->limit_upper(), -1.0);
    EXPECT_DOUBLE_EQ(axisMsg->limit_effort(), 1.0);
    EXPECT_DOUBLE_EQ(axisMsg->limit_velocity(), 100.0);
    EXPECT_DOUBLE_EQ(axisMsg->damping(), 0.9);

    // axis2
    auto axis2Msg = retJointMsg->mutable_axis2();
    EXPECT_DOUBLE_EQ(axis2Msg->xyz().x(), 0.0);
    EXPECT_DOUBLE_EQ(axis2Msg->xyz().y(), 0.0);
    EXPECT_DOUBLE_EQ(axis2Msg->xyz().z(), 1.0);
    EXPECT_EQ(axis2Msg->use_parent_model_frame(), true);
    EXPECT_DOUBLE_EQ(axis2Msg->limit_lower(), -3.2);
    EXPECT_DOUBLE_EQ(axis2Msg->limit_upper(), -3.0);
    EXPECT_DOUBLE_EQ(axis2Msg->limit_effort(), 3.0);
    EXPECT_DOUBLE_EQ(axis2Msg->limit_velocity(), 300.0);
    EXPECT_DOUBLE_EQ(axis2Msg->damping(), 3.9);

    // other joint physics properties
    EXPECT_DOUBLE_EQ(retJointMsg->cfm(), 0.9);
    EXPECT_DOUBLE_EQ(retJointMsg->bounce(), 0.8);
    EXPECT_DOUBLE_EQ(retJointMsg->velocity(), 0.7);
    EXPECT_DOUBLE_EQ(retJointMsg->fudge_factor(), 0.6);
    EXPECT_DOUBLE_EQ(retJointMsg->limit_cfm(), 0.5);
    EXPECT_DOUBLE_EQ(retJointMsg->limit_erp(), 0.4);
    EXPECT_DOUBLE_EQ(retJointMsg->suspension_cfm(), 0.3);
    EXPECT_DOUBLE_EQ(retJointMsg->suspension_erp(), 0.2);
  }

  // update fields in the message widget and
  // verify that the new message contains the updated values.
  // Joint type universal -> ball
  {
    // joint
    EXPECT_TRUE(widget->SetPropertyValue("name", QVariant::fromValue(
        std::string("test_joint_updated2"))));
    EXPECT_TRUE(widget->SetPropertyValue("id", 2222222u));
    EXPECT_TRUE(widget->SetPropertyValue("parent", QVariant::fromValue(
        std::string("test_joint_parent_updated2"))));
    EXPECT_TRUE(widget->SetPropertyValue("parent_id", 10u));
    EXPECT_TRUE(widget->SetPropertyValue("child", QVariant::fromValue(
        std::string("test_joint_child_updated2"))));
    EXPECT_TRUE(widget->SetPropertyValue("child_id", 20u));

    // type
    EXPECT_TRUE(widget->SetPropertyValue("type", QVariant::fromValue(
        msgs::Joint_Type_Name(msgs::Joint_Type_BALL))));

    // pose
    math::Vector3d pos(-2.0, 1.0, 2.0);
    math::Quaterniond quat(0.0, 0.0, 0.0);
    EXPECT_TRUE(widget->SetPropertyValue("pose", QVariant::fromValue(
        math::Pose3d(pos, quat))));

    // other joint physics properties
    EXPECT_TRUE(widget->SetPropertyValue("cfm", 0.19));
    EXPECT_TRUE(widget->SetPropertyValue("bounce", 0.18));
    EXPECT_TRUE(widget->SetPropertyValue("velocity", 2.7));
    EXPECT_TRUE(widget->SetPropertyValue("fudge_factor", 0.26));
    EXPECT_TRUE(widget->SetPropertyValue("limit_cfm", 0.15));
    EXPECT_TRUE(widget->SetPropertyValue("limit_erp", 0.24));
    EXPECT_TRUE(widget->SetPropertyValue("suspension_cfm", 0.13));
    EXPECT_TRUE(widget->SetPropertyValue("suspension_erp", 0.12));
  }

  // verify widget values
  {
    // joint
    EXPECT_EQ(widget->PropertyValue("name").value<std::string>(),
        "test_joint_updated2");
    EXPECT_EQ(widget->PropertyValue("id"), 2222222u);
    EXPECT_EQ(widget->PropertyValue("parent").value<std::string>(),
        "test_joint_parent_updated2");
    EXPECT_EQ(widget->PropertyValue("parent_id"), 10u);
    EXPECT_EQ(widget->PropertyValue("child").value<std::string>(),
        "test_joint_child_updated2");
    EXPECT_EQ(widget->PropertyValue("child_id"), 20u);

    // type
    EXPECT_TRUE(widget->SetPropertyValue("type", QVariant::fromValue(
        msgs::Joint_Type_Name(msgs::Joint_Type_BALL))));

    // pose
    math::Vector3d pos(-2.0, 1.0, 2.0);
    math::Quaterniond quat(0.0, 0.0, 0.0);
    EXPECT_EQ(widget->PropertyValue("pose"), QVariant::fromValue(
        math::Pose3d(pos, quat)));

    // other joint physics properties
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "cfm").toDouble(), 0.19);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "bounce").toDouble(), 0.18);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "velocity").toDouble(), 2.7);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "fudge_factor").toDouble(), 0.26);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "limit_cfm").toDouble(), 0.15);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "limit_erp").toDouble(), 0.24);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "suspension_cfm").toDouble(), 0.13);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "suspension_erp").toDouble(), 0.12);
  }

  // verify updates in new msg
  {
    auto retJointMsg = dynamic_cast<msgs::Joint *>(widget->Msg());
    EXPECT_TRUE(retJointMsg != nullptr);

    // joint
    EXPECT_EQ(retJointMsg->name(), "test_joint_updated2");
    EXPECT_EQ(retJointMsg->id(), 2222222u);
    EXPECT_EQ(retJointMsg->parent(), "test_joint_parent_updated2");
    EXPECT_EQ(retJointMsg->parent_id(), 10u);
    EXPECT_EQ(retJointMsg->child(), "test_joint_child_updated2");
    EXPECT_EQ(retJointMsg->child_id(), 20u);

    // type
    EXPECT_EQ(retJointMsg->type(), msgs::ConvertJointType("ball"));

    // pose
    auto poseMsg = retJointMsg->pose();
    auto posMsg = poseMsg.position();
    EXPECT_DOUBLE_EQ(posMsg.x(), -2.0);
    EXPECT_DOUBLE_EQ(posMsg.y(), 1.0);
    EXPECT_DOUBLE_EQ(posMsg.z(), 2.0);
    auto quat = msgs::Convert(poseMsg.orientation());
    EXPECT_DOUBLE_EQ(quat.Euler().X(), 0.0);
    EXPECT_DOUBLE_EQ(quat.Euler().Y(), 0.0);
    EXPECT_DOUBLE_EQ(quat.Euler().Z(), 0.0);

    // other joint physics properties
    EXPECT_DOUBLE_EQ(retJointMsg->cfm(), 0.19);
    EXPECT_DOUBLE_EQ(retJointMsg->bounce(), 0.18);
    EXPECT_DOUBLE_EQ(retJointMsg->velocity(), 2.7);
    EXPECT_DOUBLE_EQ(retJointMsg->fudge_factor(), 0.26);
    EXPECT_DOUBLE_EQ(retJointMsg->limit_cfm(), 0.15);
    EXPECT_DOUBLE_EQ(retJointMsg->limit_erp(), 0.24);
    EXPECT_DOUBLE_EQ(retJointMsg->suspension_cfm(), 0.13);
    EXPECT_DOUBLE_EQ(retJointMsg->suspension_erp(), 0.12);
  }

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
// Test nested pose and color fields
TEST(MessageWidgetTest, VisualMsgWidget)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  msgs::Visual msg;
  {
    // visual
    msg.set_name("test_visual");
    msg.set_id(12345u);
    msg.set_parent_name("test_visual_parent");
    msg.set_parent_id(54321u);
    msg.set_cast_shadows(true);
    msg.set_transparency(0.0);
    msg.set_visible(true);
    msg.set_delete_me(false);
    msg.set_is_static(false);
    msgs::Set(msg.mutable_scale(), math::Vector3d(1.0, 1.0, 1.0));

    // pose
    math::Vector3d pos(2.0, 3.0, 4.0);
    math::Quaterniond quat(1.57, 0.0, 0.0);
    msgs::Set(msg.mutable_pose(), math::Pose3d(pos, quat));

    // geometry
    auto geometryMsg = msg.mutable_geometry();
    geometryMsg->set_type(msgs::Geometry::CYLINDER);
    auto cylinderGeomMsg = geometryMsg->mutable_cylinder();
    cylinderGeomMsg->set_radius(3.0);
    cylinderGeomMsg->set_length(0.2);

    // material
    auto materialMsg = msg.mutable_material();
    materialMsg->set_shader_type(msgs::Material::Material::VERTEX);
    materialMsg->set_normal_map("test_normal_map");
    msgs::Set(materialMsg->mutable_ambient(), math::Color(0.0, 1.0, 0.0, 1.0));
    msgs::Set(materialMsg->mutable_diffuse(), math::Color(0.0, 1.0, 1.0, 0.4));
    msgs::Set(materialMsg->mutable_specular(), math::Color(1.0, 1.0, 1.0, 0.6));
    msgs::Set(materialMsg->mutable_emissive(), math::Color(0.0, 0.5, 0.2, 1.0));
    materialMsg->set_lighting(true);

    // material::script
    auto scriptMsg = materialMsg->mutable_script();
    scriptMsg->add_uri("test_script_uri_0");
    scriptMsg->add_uri("test_script_uri_1");
    scriptMsg->set_name("test_script_name");
  }

  // Create widget
  auto widget = new MessageWidget(&msg);
  ASSERT_NE(widget, nullptr);

  // Retrieve message
  {
    auto retMsg = dynamic_cast<msgs::Visual *>(widget->Msg());
    ASSERT_NE(retMsg, nullptr);

    // visual
    EXPECT_EQ(retMsg->name(), "test_visual");
    EXPECT_EQ(retMsg->id(), 12345u);
    EXPECT_EQ(retMsg->parent_name(), "test_visual_parent");
    EXPECT_EQ(retMsg->parent_id(), 54321u);
    EXPECT_EQ(retMsg->cast_shadows(), true);
    EXPECT_DOUBLE_EQ(retMsg->transparency(), 0.0);
    EXPECT_EQ(retMsg->visible(), true);
    EXPECT_EQ(retMsg->delete_me(), false);
    EXPECT_EQ(retMsg->is_static(), false);

    auto scaleMsg = retMsg->scale();
    EXPECT_DOUBLE_EQ(scaleMsg.x(), 1.0);
    EXPECT_DOUBLE_EQ(scaleMsg.y(), 1.0);
    EXPECT_DOUBLE_EQ(scaleMsg.z(), 1.0);

    // pose
    auto poseMsg = retMsg->pose();
    auto posMsg = poseMsg.position();
    EXPECT_DOUBLE_EQ(posMsg.x(), 2.0);
    EXPECT_DOUBLE_EQ(posMsg.y(), 3.0);
    EXPECT_DOUBLE_EQ(posMsg.z(), 4.0);
    auto quat = msgs::Convert(poseMsg.orientation());
    EXPECT_DOUBLE_EQ(quat.Euler().X(), 1.57);
    EXPECT_DOUBLE_EQ(quat.Euler().Y(), 0.0);
    EXPECT_DOUBLE_EQ(quat.Euler().Z(), 0.0);

    // geometry
    auto geometryMsg = retMsg->geometry();
    EXPECT_EQ(geometryMsg.type(), msgs::Geometry::CYLINDER);
    auto cylinderGeomMsg = geometryMsg.cylinder();
    EXPECT_DOUBLE_EQ(cylinderGeomMsg.radius(), 3.0);
    EXPECT_DOUBLE_EQ(cylinderGeomMsg.length(), 0.2);

    // material
    auto materialMsg = retMsg->material();
    EXPECT_EQ(materialMsg.shader_type(), msgs::Material::Material::VERTEX);
    EXPECT_EQ(materialMsg.normal_map(), "test_normal_map");
    auto ambientMsg = materialMsg.ambient();
    EXPECT_DOUBLE_EQ(ambientMsg.r(), 0.0f);
    EXPECT_DOUBLE_EQ(ambientMsg.g(), 1.0f);
    EXPECT_DOUBLE_EQ(ambientMsg.b(), 0.0f);
    EXPECT_DOUBLE_EQ(ambientMsg.a(), 1.0f);
    auto diffuseMsg = materialMsg.diffuse();
    EXPECT_DOUBLE_EQ(diffuseMsg.r(), 0.0f);
    EXPECT_DOUBLE_EQ(diffuseMsg.g(), 1.0f);
    EXPECT_DOUBLE_EQ(diffuseMsg.b(), 1.0f);
    EXPECT_DOUBLE_EQ(diffuseMsg.a(), 0.4f);
    auto specularMsg = materialMsg.specular();
    EXPECT_DOUBLE_EQ(specularMsg.r(), 1.0f);
    EXPECT_DOUBLE_EQ(specularMsg.g(), 1.0f);
    EXPECT_DOUBLE_EQ(specularMsg.b(), 1.0f);
    EXPECT_DOUBLE_EQ(specularMsg.a(), 0.6f);
    auto emissiveMsg = materialMsg.emissive();
    EXPECT_DOUBLE_EQ(emissiveMsg.r(), 0.0f);
    EXPECT_DOUBLE_EQ(emissiveMsg.g(), 0.5f);
    EXPECT_DOUBLE_EQ(emissiveMsg.b(), 0.2f);
    EXPECT_DOUBLE_EQ(emissiveMsg.a(), 1.0f);
    EXPECT_EQ(materialMsg.lighting(), true);

    // material::script
    auto scriptMsg = materialMsg.script();
    EXPECT_EQ(scriptMsg.uri(0), "test_script_uri_0");
    EXPECT_EQ(scriptMsg.uri(1), "test_script_uri_1");
    EXPECT_EQ(scriptMsg.name(), "test_script_name");
  }

  // Update from message
  {
    // visual
    msg.set_name("test_visual_2");
    msg.set_id(123452u);
    msg.set_parent_name("test_visual_parent_2");
    msg.set_parent_id(543212u);
    msg.set_cast_shadows(false);
    msg.set_transparency(0.2);
    msg.set_visible(false);
    msg.set_delete_me(true);
    msg.set_is_static(true);
    msgs::Set(msg.mutable_scale(), math::Vector3d(1.0, 1.0, 2.0));

    // pose
    math::Vector3d pos(2.0, 3.0, 2.0);
    math::Quaterniond quat(0.0, 0.0, 0.0);
    msgs::Set(msg.mutable_pose(), math::Pose3d(pos, quat));

    // material
    auto materialMsg = msg.mutable_material();
    materialMsg->set_shader_type(msgs::Material::Material::VERTEX);
    materialMsg->set_normal_map("test_normal_map_2");
    msgs::Set(materialMsg->mutable_ambient(), math::Color(0.0, 1.0, 0.0, 0.2));
    msgs::Set(materialMsg->mutable_diffuse(), math::Color(0.0, 1.0, 1.0, 0.2));
    msgs::Set(materialMsg->mutable_specular(), math::Color(1.0, 1.0, 1.0, 0.2));
    msgs::Set(materialMsg->mutable_emissive(), math::Color(0.0, 0.5, 0.2, 0.2));
    materialMsg->set_lighting(false);

    // material::script
    auto scriptMsg = materialMsg->mutable_script();
    scriptMsg->set_name("test_script_name_2");
  }
  widget->UpdateFromMsg(&msg);

  // Retrieve message
  {
    auto retMsg = dynamic_cast<msgs::Visual *>(widget->Msg());
    ASSERT_NE(retMsg, nullptr);

    // visual
    EXPECT_EQ(retMsg->name(), "test_visual_2");
    EXPECT_EQ(retMsg->id(), 123452u);
    EXPECT_EQ(retMsg->parent_name(), "test_visual_parent_2");
    EXPECT_EQ(retMsg->parent_id(), 543212u);
    EXPECT_EQ(retMsg->cast_shadows(), false);
    EXPECT_DOUBLE_EQ(retMsg->transparency(), 0.2);
    EXPECT_EQ(retMsg->visible(), false);
    EXPECT_EQ(retMsg->delete_me(), true);
    EXPECT_EQ(retMsg->is_static(), true);

    auto scaleMsg = retMsg->scale();
    EXPECT_DOUBLE_EQ(scaleMsg.x(), 1.0);
    EXPECT_DOUBLE_EQ(scaleMsg.y(), 1.0);
    EXPECT_DOUBLE_EQ(scaleMsg.z(), 2.0);

    // pose
    auto poseMsg = retMsg->pose();
    auto posMsg = poseMsg.position();
    EXPECT_DOUBLE_EQ(posMsg.x(), 2.0);
    EXPECT_DOUBLE_EQ(posMsg.y(), 3.0);
    EXPECT_DOUBLE_EQ(posMsg.z(), 2.0);
    auto quat = msgs::Convert(poseMsg.orientation());
    EXPECT_DOUBLE_EQ(quat.Euler().X(), 0.0);
    EXPECT_DOUBLE_EQ(quat.Euler().Y(), 0.0);
    EXPECT_DOUBLE_EQ(quat.Euler().Z(), 0.0);

    // material
    auto materialMsg = retMsg->material();
    EXPECT_EQ(materialMsg.shader_type(), msgs::Material::Material::VERTEX);
    EXPECT_EQ(materialMsg.normal_map(), "test_normal_map_2");
    auto ambientMsg = materialMsg.ambient();
    EXPECT_DOUBLE_EQ(ambientMsg.r(), 0.0f);
    EXPECT_DOUBLE_EQ(ambientMsg.g(), 1.0f);
    EXPECT_DOUBLE_EQ(ambientMsg.b(), 0.0f);
    EXPECT_DOUBLE_EQ(ambientMsg.a(), 0.2f);
    auto diffuseMsg = materialMsg.diffuse();
    EXPECT_DOUBLE_EQ(diffuseMsg.r(), 0.0f);
    EXPECT_DOUBLE_EQ(diffuseMsg.g(), 1.0f);
    EXPECT_DOUBLE_EQ(diffuseMsg.b(), 1.0f);
    EXPECT_DOUBLE_EQ(diffuseMsg.a(), 0.2f);
    auto specularMsg = materialMsg.specular();
    EXPECT_DOUBLE_EQ(specularMsg.r(), 1.0f);
    EXPECT_DOUBLE_EQ(specularMsg.g(), 1.0f);
    EXPECT_DOUBLE_EQ(specularMsg.b(), 1.0f);
    EXPECT_DOUBLE_EQ(specularMsg.a(), 0.2f);
    auto emissiveMsg = materialMsg.emissive();
    EXPECT_DOUBLE_EQ(emissiveMsg.r(), 0.0f);
    EXPECT_DOUBLE_EQ(emissiveMsg.g(), 0.5f);
    EXPECT_DOUBLE_EQ(emissiveMsg.b(), 0.2f);
    EXPECT_DOUBLE_EQ(emissiveMsg.a(), 0.2f);
    EXPECT_EQ(materialMsg.lighting(), false);

    // material::script
    auto scriptMsg = materialMsg.script();
    EXPECT_EQ(scriptMsg.name(), "test_script_name_2");
  }

  // Expand all widgets so they're generated
  widget->ToggleAll(true);

  // update fields in the message widget and
  // verify that the new message contains the updated values.
  {
    // visual
    EXPECT_TRUE(widget->SetPropertyValue("name", QVariant::fromValue(
        std::string("test_visual_updated"))));
    EXPECT_TRUE(widget->SetPropertyValue("id", 11111u));
    EXPECT_TRUE(widget->SetPropertyValue("parent_name", QVariant::fromValue(
        std::string("test_visual_parent_updated"))));
    EXPECT_TRUE(widget->SetPropertyValue("parent_id", 55555u));
    EXPECT_TRUE(widget->SetPropertyValue("cast_shadows", false));
    EXPECT_TRUE(widget->SetPropertyValue("transparency", 1.0));
    EXPECT_TRUE(widget->SetPropertyValue("visible", false));
    EXPECT_TRUE(widget->SetPropertyValue("delete_me", true));
    EXPECT_TRUE(widget->SetPropertyValue("is_static", true));
    EXPECT_TRUE(widget->SetPropertyValue("scale", QVariant::fromValue(
        math::Vector3d(2.0, 1.5, 0.5))));

    // pose
    math::Vector3d pos(-2.0, -3.0, -4.0);
    math::Quaterniond quat(0.0, 1.57, 0.0);
    EXPECT_TRUE(widget->SetPropertyValue("pose", QVariant::fromValue(
        math::Pose3d(pos, quat))));

    // geometry
    msgs::Geometry newGeom;
    newGeom.set_type(msgs::Geometry::BOX);
    msgs::Set(newGeom.mutable_box()->mutable_size(),
              math::Vector3d(5.0, 3.0, 4.0));
    EXPECT_TRUE(widget->SetPropertyValue("geometry", QVariant::fromValue(
        newGeom)));

    // material
    EXPECT_TRUE(widget->SetPropertyValue("material::normal_map",
        QVariant::fromValue(std::string("test_normal_map_updated"))));
    EXPECT_TRUE(widget->SetPropertyValue("material::ambient",
        QVariant::fromValue(math::Color(0.2, 0.3, 0.4, 0.5))));
    EXPECT_TRUE(widget->SetPropertyValue("material::diffuse",
        QVariant::fromValue(math::Color(0.1, 0.8, 0.6, 0.4))));
    EXPECT_TRUE(widget->SetPropertyValue("material::specular",
        QVariant::fromValue(math::Color(0.5, 0.4, 0.3, 0.2))));
    EXPECT_TRUE(widget->SetPropertyValue("material::emissive",
        QVariant::fromValue(math::Color(0.4, 0.6, 0.8, 0.1))));
    EXPECT_TRUE(widget->SetPropertyValue("material::lighting", false));
    // material::script
    EXPECT_TRUE(widget->SetPropertyValue("material::script::name",
        QVariant::fromValue(std::string("test_script_name_updated"))));
  }

  // verify widget values
  {
    EXPECT_EQ(widget->PropertyValue(
        "name").value<std::string>(), "test_visual_updated");
    EXPECT_EQ(widget->PropertyValue(
        "id"), 11111u);
    EXPECT_EQ(widget->PropertyValue(
        "parent_name").value<std::string>(), "test_visual_parent_updated");
    EXPECT_EQ(widget->PropertyValue(
        "parent_id"), 55555u);
    EXPECT_EQ(widget->PropertyValue(
        "cast_shadows").toBool(), false);
    EXPECT_DOUBLE_EQ(widget->PropertyValue(
        "transparency").toDouble(), 1.0);
    EXPECT_EQ(widget->PropertyValue(
        "visible").toBool(), false);
    EXPECT_EQ(widget->PropertyValue(
        "delete_me").toBool(), true);
    EXPECT_EQ(widget->PropertyValue(
        "is_static").toBool(), true);
    EXPECT_EQ(widget->PropertyValue(
        "scale").value<math::Vector3d>(), math::Vector3d(2.0, 1.5, 0.5));

    // pose
    math::Vector3d pos(-2.0, -3.0, -4.0);
    math::Quaterniond quat(0.0, 1.57, 0.0);
    EXPECT_EQ(widget->PropertyValue("pose"), QVariant::fromValue(
        math::Pose3d(pos, quat)));

    // geometry
    auto geomValue =
        widget->PropertyValue("geometry").value<msgs::Geometry>();
    EXPECT_EQ(msgs::ConvertGeometryType(geomValue.type()), "box");
    EXPECT_EQ(msgs::Convert(geomValue.box().size()),
              math::Vector3d(5.0, 3.0, 4.0));

    // material
    EXPECT_EQ(widget->PropertyValue(
        "material::normal_map").value<std::string>(),
        "test_normal_map_updated");
    EXPECT_EQ(widget->PropertyValue("material::ambient"),
        QVariant::fromValue(math::Color(0.2, 0.3, 0.4, 0.5)));
    EXPECT_EQ(widget->PropertyValue("material::diffuse"),
        QVariant::fromValue(math::Color(0.1, 0.8, 0.6, 0.4)));
    EXPECT_EQ(widget->PropertyValue("material::specular"),
        QVariant::fromValue(math::Color(0.5, 0.4, 0.3, 0.2)));
    EXPECT_EQ(widget->PropertyValue("material::emissive"),
        QVariant::fromValue(math::Color(0.4, 0.6, 0.8, 0.1)));
    EXPECT_EQ(widget->PropertyValue("material::lighting").toBool(), false);
    // material::script
    EXPECT_EQ(widget->PropertyValue(
        "material::script::name").value<std::string>(),
        "test_script_name_updated");
  }

  // verify updates in new msg
  {
    auto retMsg =
        dynamic_cast<msgs::Visual *>(widget->Msg());
    EXPECT_TRUE(retMsg != nullptr);

    // visual
    EXPECT_EQ(retMsg->name(), "test_visual_updated");
    EXPECT_EQ(retMsg->id(), 11111u);
    EXPECT_EQ(retMsg->parent_name(), "test_visual_parent_updated");
    EXPECT_EQ(retMsg->parent_id(), 55555u);
    EXPECT_EQ(retMsg->cast_shadows(), false);
    EXPECT_DOUBLE_EQ(retMsg->transparency(), 1.0);
    EXPECT_EQ(retMsg->visible(), false);
    EXPECT_EQ(retMsg->delete_me(), true);
    EXPECT_EQ(retMsg->is_static(), true);
    auto scaleMsg = retMsg->scale();
    EXPECT_DOUBLE_EQ(scaleMsg.x(), 2.0);
    EXPECT_DOUBLE_EQ(scaleMsg.y(), 1.5);
    EXPECT_DOUBLE_EQ(scaleMsg.z(), 0.5);

    // pose
    auto poseMsg = retMsg->pose();
    auto posMsg = poseMsg.position();
    EXPECT_DOUBLE_EQ(posMsg.x(), -2.0);
    EXPECT_DOUBLE_EQ(posMsg.y(), -3.0);
    EXPECT_DOUBLE_EQ(posMsg.z(), -4.0);
    auto quat = msgs::Convert(poseMsg.orientation());
    EXPECT_DOUBLE_EQ(quat.Euler().X(), 0.0);
    EXPECT_LT(fabs(quat.Euler().Y() - 1.57), 0.0001);
    EXPECT_DOUBLE_EQ(quat.Euler().Z(), 0.0);

    // geometry
    auto geometryMsg = retMsg->geometry();
    EXPECT_EQ(geometryMsg.type(), msgs::Geometry::BOX);
    auto boxGeomMsg = geometryMsg.box();
    auto boxGeomSizeMsg = boxGeomMsg.size();
    EXPECT_DOUBLE_EQ(boxGeomSizeMsg.x(), 5.0);
    EXPECT_DOUBLE_EQ(boxGeomSizeMsg.y(), 3.0);
    EXPECT_DOUBLE_EQ(boxGeomSizeMsg.z(), 4.0);

    // material
    auto materialMsg = retMsg->material();
    EXPECT_EQ(materialMsg.shader_type(), msgs::Material::Material::VERTEX);
    EXPECT_EQ(materialMsg.normal_map(), "test_normal_map_updated");
    auto ambientMsg = materialMsg.ambient();
    EXPECT_DOUBLE_EQ(ambientMsg.r(), 0.2f);
    EXPECT_DOUBLE_EQ(ambientMsg.g(), 0.3f);
    EXPECT_DOUBLE_EQ(ambientMsg.b(), 0.4f);
    EXPECT_DOUBLE_EQ(ambientMsg.a(), 0.5f);
    auto diffuseMsg = materialMsg.diffuse();
    EXPECT_DOUBLE_EQ(diffuseMsg.r(), 0.1f);
    EXPECT_DOUBLE_EQ(diffuseMsg.g(), 0.8f);
    EXPECT_DOUBLE_EQ(diffuseMsg.b(), 0.6f);
    EXPECT_DOUBLE_EQ(diffuseMsg.a(), 0.4f);
    auto specularMsg = materialMsg.specular();
    EXPECT_DOUBLE_EQ(specularMsg.r(), 0.5f);
    EXPECT_DOUBLE_EQ(specularMsg.g(), 0.4f);
    EXPECT_DOUBLE_EQ(specularMsg.b(), 0.3f);
    EXPECT_DOUBLE_EQ(specularMsg.a(), 0.2f);
    auto emissiveMsg = materialMsg.emissive();
    EXPECT_DOUBLE_EQ(emissiveMsg.r(), 0.4f);
    EXPECT_DOUBLE_EQ(emissiveMsg.g(), 0.6f);
    EXPECT_DOUBLE_EQ(emissiveMsg.b(), 0.8f);
    EXPECT_DOUBLE_EQ(emissiveMsg.a(), 0.1f);
    EXPECT_EQ(materialMsg.lighting(), false);

    // material::script
    auto scriptMsg = materialMsg.script();
    EXPECT_EQ(scriptMsg.uri(0), "test_script_uri_0");
    EXPECT_EQ(scriptMsg.uri(1), "test_script_uri_1");
    EXPECT_EQ(scriptMsg.name(), "test_script_name_updated");
  }

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
// Test LINE and PLAIN_TEXT string fields, and repeated messages
TEST(MessageWidgetTest, PluginVMsgWidget)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  msgs::Plugin_V msg;

  auto pluginsMsg0 = msg.add_plugins();
  pluginsMsg0->set_name("test_plugin");
  pluginsMsg0->set_filename("test_plugin_filename");
  pluginsMsg0->set_innerxml("<param>1</param>\n");

  // Create widget
  auto widget = new MessageWidget(&msg);
  ASSERT_NE(widget, nullptr);
  widget->ToggleAll(true);

  EXPECT_NE(nullptr, widget->PropertyWidgetByName("plugins::0"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("plugins::1"));

  // Check the repetition collapsible button has been properly named
  auto prop0Col = qobject_cast<CollapsibleWidget *>(
      widget->PropertyWidgetByName("plugins::0"));
  ASSERT_NE(nullptr, prop0Col);

  auto prop0Label = prop0Col->findChild<QLabel *>("collapsibleButtonLabel");
  ASSERT_NE(nullptr, prop0Label);

  EXPECT_EQ("Test plugin", prop0Label->text().toStdString());

  auto count =  widget->PropertyWidgetCount();

  // Retrieve message
  auto retMsg = dynamic_cast<msgs::Plugin_V *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);
  ASSERT_EQ(retMsg->plugins_size(), 1);

  EXPECT_EQ(retMsg->plugins(0).name(), "test_plugin");
  EXPECT_EQ(retMsg->plugins(0).filename(), "test_plugin_filename");
  EXPECT_EQ(retMsg->plugins(0).innerxml(), "<param>1</param>\n");

  // Update from message - change the only plugin
  msg.clear_plugins();

  pluginsMsg0 = msg.add_plugins();
  pluginsMsg0->set_name("test_plugin_new");
  pluginsMsg0->set_filename("test_plugin_filename_new");
  pluginsMsg0->set_innerxml("<param>2</param>\n");

  widget->UpdateFromMsg(&msg);
  EXPECT_EQ(count,  widget->PropertyWidgetCount());
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("plugins::0"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("plugins::1"));

  // FIXME: The repetition collapsible button is not renamed on subsequent
  // messages, see issue #29
  // prop0Col = qobject_cast<CollapsibleWidget *>(
  //     widget->PropertyWidgetByName("plugins::0"));
  // ASSERT_NE(nullptr, prop0Col);

  // prop0Label = prop0Col->findChild<QLabel *>("collapsibleButtonLabel");
  // ASSERT_NE(nullptr, prop0Label);

  // EXPECT_EQ("Test plugin new", prop0Label->text().toStdString());

  // Check new message
  retMsg = dynamic_cast<msgs::Plugin_V *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);
  ASSERT_EQ(retMsg->plugins_size(), 1);

  EXPECT_EQ(retMsg->plugins(0).name(), "test_plugin_new");
  EXPECT_EQ(retMsg->plugins(0).filename(), "test_plugin_filename_new");
  EXPECT_EQ(retMsg->plugins(0).innerxml(), "<param>2</param>\n");

  // Update fields of plugin 1
  EXPECT_TRUE(widget->SetPropertyValue("plugins::0::name", QVariant::fromValue(
      std::string("test_plugin_updated"))));
  EXPECT_TRUE(widget->SetPropertyValue("plugins::0::filename",
      QVariant::fromValue(std::string("test_plugin_filename_updated"))));
  EXPECT_TRUE(widget->SetPropertyValue("plugins::0::innerxml",
      QVariant::fromValue(std::string("<param2>new_param</param2>\n"))));

  // Check fields
  EXPECT_EQ(widget->PropertyValue(
      "plugins::0::name").value<std::string>(), "test_plugin_updated");
  EXPECT_EQ(widget->PropertyValue("plugins::0::filename").value<std::string>(),
      "test_plugin_filename_updated");
  EXPECT_EQ(widget->PropertyValue("plugins::0::innerxml").value<std::string>(),
      "<param2>new_param</param2>\n");

  // Check new message
  retMsg = dynamic_cast<msgs::Plugin_V *>(widget->Msg());
  EXPECT_NE(retMsg, nullptr);
  ASSERT_EQ(retMsg->plugins_size(), 1);

  EXPECT_EQ(retMsg->plugins(0).name(), "test_plugin_updated");
  EXPECT_EQ(retMsg->plugins(0).filename(), "test_plugin_filename_updated");
  EXPECT_EQ(retMsg->plugins(0).innerxml(), "<param2>new_param</param2>\n");

  // Update from message - add more plugins
  msg.clear_plugins();

  pluginsMsg0 = msg.add_plugins();
  pluginsMsg0->set_name("test_plugin_0");
  pluginsMsg0->set_filename("test_plugin_filename_0");
  pluginsMsg0->set_innerxml("<param>0</param>\n");

  auto pluginsMsg1 = msg.add_plugins();
  pluginsMsg1->set_name("test_plugin_1");
  pluginsMsg1->set_filename("test_plugin_filename_1");
  pluginsMsg1->set_innerxml("<param>1</param>\n");

  widget->UpdateFromMsg(&msg);
  EXPECT_LT(count,  widget->PropertyWidgetCount());
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("plugins::0"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("plugins::1"));

  // Check new message
  retMsg = dynamic_cast<msgs::Plugin_V *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);
  ASSERT_EQ(retMsg->plugins_size(), 2);

  EXPECT_EQ(retMsg->plugins(0).name(), "test_plugin_0");
  EXPECT_EQ(retMsg->plugins(0).filename(), "test_plugin_filename_0");
  EXPECT_EQ(retMsg->plugins(0).innerxml(), "<param>0</param>\n");

  EXPECT_EQ(retMsg->plugins(1).name(), "test_plugin_1");
  EXPECT_EQ(retMsg->plugins(1).filename(), "test_plugin_filename_1");
  EXPECT_EQ(retMsg->plugins(1).innerxml(), "<param>1</param>\n");

  // Update from message - remove plugins
  msg.clear_plugins();

  pluginsMsg0 = msg.add_plugins();
  pluginsMsg0->set_name("test_plugin_0_only");
  pluginsMsg0->set_filename("test_plugin_filename_0_only");
  pluginsMsg0->set_innerxml("<param>0_only</param>\n");

  widget->UpdateFromMsg(&msg);
  // Widget count not properly reduced on OSX, issue #23
#if !defined(__APPLE__)
  EXPECT_EQ(count,  widget->PropertyWidgetCount());
#endif
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("plugins::0"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("plugins::1"));

  // Check new message
  retMsg = dynamic_cast<msgs::Plugin_V *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);
  ASSERT_EQ(retMsg->plugins_size(), 1);

  EXPECT_EQ(retMsg->plugins(0).name(), "test_plugin_0_only");
  EXPECT_EQ(retMsg->plugins(0).filename(), "test_plugin_filename_0_only");
  EXPECT_EQ(retMsg->plugins(0).innerxml(), "<param>0_only</param>\n");

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
// Test double, uint32 and bool fields
TEST(MessageWidgetTest, SurfaceMsgWidget)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  msgs::Surface msg;
  msg.set_kp(100.5);
  msg.set_collide_bitmask(1);
  msg.set_collide_without_contact(true);

  // Create widget
  auto widget = new MessageWidget(&msg);
  ASSERT_NE(widget, nullptr);

  // Retrieve message
  auto retMsg = dynamic_cast<msgs::Surface *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);

  EXPECT_DOUBLE_EQ(retMsg->kp(), 100.5);
  EXPECT_EQ(retMsg->collide_bitmask(), 1u);
  EXPECT_TRUE(retMsg->collide_without_contact());

  // Update from message
  msg.set_kp(888.44);
  msg.set_collide_bitmask(444);
  msg.set_collide_without_contact(false);

  widget->UpdateFromMsg(&msg);

  // Check new message
  retMsg = dynamic_cast<msgs::Surface *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);

  EXPECT_DOUBLE_EQ(retMsg->kp(), 888.44);
  EXPECT_EQ(retMsg->collide_bitmask(), 444u);
  EXPECT_FALSE(retMsg->collide_without_contact());

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
// Test float fields
TEST(MessageWidgetTest, LightMsgWidget)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  msgs::Light msg;
  msg.set_spot_falloff(0.5);

  // Create widget
  auto widget = new MessageWidget(&msg);
  ASSERT_NE(widget, nullptr);

  // Retrieve message
  auto retMsg = dynamic_cast<msgs::Light *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);

  EXPECT_LT(fabs(retMsg->spot_falloff()- 0.5), 0.000001);

  // Update from message
  msg.set_spot_falloff(0.001);

  widget->UpdateFromMsg(&msg);

  // Check new message
  retMsg = dynamic_cast<msgs::Light *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);

  EXPECT_LT(fabs(retMsg->spot_falloff()- 0.001), 0.000001);

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
// Test uint64 fields
TEST(MessageWidgetTest, WorldStatsMsgWidget)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  msgs::WorldStatistics msg;
  msg.set_iterations(555);

  // Create widget
  auto widget = new MessageWidget(&msg);
  ASSERT_NE(widget, nullptr);

  // Retrieve message
  auto retMsg = dynamic_cast<msgs::WorldStatistics *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);

  EXPECT_EQ(retMsg->iterations(), 555u);

  // Update from message
  msg.set_iterations(99999999);

  widget->UpdateFromMsg(&msg);

  // Check new message
  retMsg = dynamic_cast<msgs::WorldStatistics *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);

  EXPECT_EQ(retMsg->iterations(), 99999999u);

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
// Test repeated int32 fields
TEST(MessageWidgetTest, Int32VMsgWidget)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message with one field
  msgs::Int32_V msg;
  msg.add_data(0);

  // Create widget
  auto widget = new MessageWidget(&msg);
  ASSERT_NE(nullptr, widget);
  widget->ToggleAll(true);

  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::0"));

  // Retrieve message
  auto retMsg = dynamic_cast<msgs::Int32_V *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);
  ASSERT_EQ(retMsg->data_size(), 1);
  EXPECT_EQ(retMsg->data(0), 0);

  // Update from message with 2 of each repeated fiels
  msg.clear_data();
  msg.add_data(1);
  msg.add_data(2);

  widget->UpdateFromMsg(&msg);

  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::0"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::1"));

  // Check new message
  retMsg = dynamic_cast<msgs::Int32_V *>(widget->Msg());
  ASSERT_NE(nullptr, retMsg);
  ASSERT_EQ(retMsg->data_size(), 2);

  EXPECT_EQ(retMsg->data(0), 1);
  EXPECT_EQ(retMsg->data(1), 2);

  // Update fields
  EXPECT_TRUE(widget->SetPropertyValue("data::0", QVariant::fromValue(3)));

  // Check fields
  EXPECT_EQ(widget->PropertyValue("data::0").value<int>(), 3);

  // Check new message
  retMsg = dynamic_cast<msgs::Int32_V *>(widget->Msg());
  ASSERT_NE(nullptr, retMsg);
  ASSERT_EQ(retMsg->data_size(), 2);

  EXPECT_EQ(retMsg->data(0), 3);

  EXPECT_EQ(retMsg->data(1), 2);

  // Update from message - remove entries
  msg.clear_data();
  msg.add_data(4);

  widget->UpdateFromMsg(&msg);
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::0"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("data::1"));

  // Check new message
  retMsg = dynamic_cast<msgs::Int32_V *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);

  ASSERT_EQ(retMsg->data_size(), 1);
  EXPECT_EQ(retMsg->data(0), 4);

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
// Test repeated int64 fields
TEST(MessageWidgetTest, Int64VMsgWidget)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message with one field
  msgs::Int64_V msg;
  msg.add_data(0);

  // Create widget
  auto widget = new MessageWidget(&msg);
  ASSERT_NE(nullptr, widget);
  widget->ToggleAll(true);

  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::0"));

  // Retrieve message
  auto retMsg = dynamic_cast<msgs::Int64_V *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);
  ASSERT_EQ(retMsg->data_size(), 1);
  EXPECT_EQ(retMsg->data(0), 0);

  // Update from message with 2 of each repeated fiels
  msg.clear_data();
  msg.add_data(1);
  msg.add_data(2);

  widget->UpdateFromMsg(&msg);

  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::0"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::1"));

  // Check new message
  retMsg = dynamic_cast<msgs::Int64_V *>(widget->Msg());
  ASSERT_NE(nullptr, retMsg);
  ASSERT_EQ(retMsg->data_size(), 2);

  EXPECT_EQ(retMsg->data(0), 1);
  EXPECT_EQ(retMsg->data(1), 2);

  // Update fields
  EXPECT_TRUE(widget->SetPropertyValue("data::0", QVariant::fromValue(3)));

  // Check fields
  EXPECT_EQ(widget->PropertyValue("data::0").value<int>(), 3);

  // Check new message
  retMsg = dynamic_cast<msgs::Int64_V *>(widget->Msg());
  ASSERT_NE(nullptr, retMsg);
  ASSERT_EQ(retMsg->data_size(), 2);

  EXPECT_EQ(retMsg->data(0), 3);

  EXPECT_EQ(retMsg->data(1), 2);

  // Update from message - remove entries
  msg.clear_data();
  msg.add_data(4);

  widget->UpdateFromMsg(&msg);
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::0"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("data::1"));

  // Check new message
  retMsg = dynamic_cast<msgs::Int64_V *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);

  ASSERT_EQ(retMsg->data_size(), 1);
  EXPECT_EQ(retMsg->data(0), 4);

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
// Test repeated uint64 fields
TEST(MessageWidgetTest, UInt64VMsgWidget)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message with one field
  msgs::UInt64_V msg;
  msg.add_data(0);

  // Create widget
  auto widget = new MessageWidget(&msg);
  ASSERT_NE(nullptr, widget);
  widget->ToggleAll(true);

  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::0"));

  // Retrieve message
  auto retMsg = dynamic_cast<msgs::UInt64_V *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);
  ASSERT_EQ(retMsg->data_size(), 1);
  EXPECT_EQ(retMsg->data(0), 0u);

  // Update from message with 2 of each repeated fiels
  msg.clear_data();
  msg.add_data(1);
  msg.add_data(2);

  widget->UpdateFromMsg(&msg);

  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::0"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::1"));

  // Check new message
  retMsg = dynamic_cast<msgs::UInt64_V *>(widget->Msg());
  ASSERT_NE(nullptr, retMsg);
  ASSERT_EQ(retMsg->data_size(), 2);

  EXPECT_EQ(retMsg->data(0), 1u);
  EXPECT_EQ(retMsg->data(1), 2u);

  // Update fields
  EXPECT_TRUE(widget->SetPropertyValue("data::0", QVariant::fromValue(3)));

  // Check fields
  EXPECT_EQ(widget->PropertyValue("data::0").value<unsigned int>(), 3u);

  // Check new message
  retMsg = dynamic_cast<msgs::UInt64_V *>(widget->Msg());
  ASSERT_NE(nullptr, retMsg);
  ASSERT_EQ(retMsg->data_size(), 2);

  EXPECT_EQ(retMsg->data(0), 3u);
  EXPECT_EQ(retMsg->data(1), 2u);

  // Update from message - remove entries
  msg.clear_data();
  msg.add_data(4);

  widget->UpdateFromMsg(&msg);
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::0"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("data::1"));

  // Check new message
  retMsg = dynamic_cast<msgs::UInt64_V *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);

  ASSERT_EQ(retMsg->data_size(), 1);
  EXPECT_EQ(retMsg->data(0), 4u);

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
// Test repeated float fields
TEST(MessageWidgetTest, FloatVMsgWidget)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message with one field
  msgs::Float_V msg;
  msg.add_data(0.1);

  // Create widget
  auto widget = new MessageWidget(&msg);
  ASSERT_NE(nullptr, widget);
  widget->ToggleAll(true);

  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::0"));

  // Retrieve message
  auto retMsg = dynamic_cast<msgs::Float_V *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);
  ASSERT_EQ(retMsg->data_size(), 1);
  EXPECT_FLOAT_EQ(retMsg->data(0), 0.1);

  // Update from message with 2 of each repeated fiels
  msg.clear_data();
  msg.add_data(1.1);
  msg.add_data(2.1);

  widget->UpdateFromMsg(&msg);

  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::0"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::1"));

  // Check new message
  retMsg = dynamic_cast<msgs::Float_V *>(widget->Msg());
  ASSERT_NE(nullptr, retMsg);
  ASSERT_EQ(retMsg->data_size(), 2);

  EXPECT_FLOAT_EQ(retMsg->data(0), 1.1);
  EXPECT_FLOAT_EQ(retMsg->data(1), 2.1);

  // Update fields
  EXPECT_TRUE(widget->SetPropertyValue("data::0", QVariant::fromValue(3.1)));

  // Check fields
  EXPECT_FLOAT_EQ(widget->PropertyValue("data::0").value<float>(), 3.1);

  // Check new message
  retMsg = dynamic_cast<msgs::Float_V *>(widget->Msg());
  ASSERT_NE(nullptr, retMsg);
  ASSERT_EQ(retMsg->data_size(), 2);

  EXPECT_FLOAT_EQ(retMsg->data(0), 3.1);
  EXPECT_FLOAT_EQ(retMsg->data(1), 2.1);

  // Update from message - remove entries
  msg.clear_data();
  msg.add_data(4.1);

  widget->UpdateFromMsg(&msg);
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data::0"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("data::1"));

  // Check new message
  retMsg = dynamic_cast<msgs::Float_V *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);

  ASSERT_EQ(retMsg->data_size(), 1);
  EXPECT_FLOAT_EQ(retMsg->data(0), 4.1);

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
// Test repeated string, uint32 and double fields
TEST(MessageWidgetTest, TactileMsgWidget)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message with one of each repeated field
  msgs::Tactile msg;

  msg.add_collision_name("col0");
  msg.add_collision_id(0);
  msg.add_pressure(0.1);

  // Create widget
  auto widget = new MessageWidget(&msg);
  ASSERT_NE(nullptr, widget);
  widget->ToggleAll(true);

  EXPECT_NE(nullptr, widget->PropertyWidgetByName("collision_name::0"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("collision_id::0"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("pressure::0"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("collision_name::1"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("collision_id::1"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("pressure::1"));

  // Retrieve message
  auto retMsg = dynamic_cast<msgs::Tactile *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);
  ASSERT_EQ(retMsg->collision_name_size(), 1);
  ASSERT_EQ(retMsg->collision_id_size(), 1);
  ASSERT_EQ(retMsg->pressure_size(), 1);

  EXPECT_EQ(retMsg->collision_name(0), "col0");
  EXPECT_EQ(retMsg->collision_id(0), 0u);
  EXPECT_DOUBLE_EQ(retMsg->pressure(0), 0.1);

  // Update from message with 2 of each repeated fiels
  msg.clear_collision_name();
  msg.clear_collision_id();
  msg.clear_pressure();

  msg.add_collision_name("col1");
  msg.add_collision_id(1);
  msg.add_pressure(1.1);

  msg.add_collision_name("col2");
  msg.add_collision_id(2);
  msg.add_pressure(2.1);

  widget->UpdateFromMsg(&msg);

  EXPECT_NE(nullptr, widget->PropertyWidgetByName("collision_name::0"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("collision_id::0"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("pressure::0"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("collision_name::1"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("collision_id::1"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("pressure::1"));

  // Check new message
  retMsg = dynamic_cast<msgs::Tactile *>(widget->Msg());
  ASSERT_NE(nullptr, retMsg);
  ASSERT_EQ(retMsg->collision_name_size(), 2);
  ASSERT_EQ(retMsg->collision_id_size(), 2);
  ASSERT_EQ(retMsg->pressure_size(), 2);

  EXPECT_EQ(retMsg->collision_name(0), "col1");
  EXPECT_EQ(retMsg->collision_id(0), 1u);
  EXPECT_DOUBLE_EQ(retMsg->pressure(0), 1.1);

  EXPECT_EQ(retMsg->collision_name(1), "col2");
  EXPECT_EQ(retMsg->collision_id(1), 2u);
  EXPECT_DOUBLE_EQ(retMsg->pressure(1), 2.1);

  // Update fields
  EXPECT_TRUE(widget->SetPropertyValue("collision_name::0", QVariant::fromValue(
      std::string("col3"))));
  EXPECT_TRUE(widget->SetPropertyValue("collision_id::1",
      QVariant::fromValue(3)));
  EXPECT_TRUE(widget->SetPropertyValue("pressure::0",
      QVariant::fromValue(3.1)));

  // Check fields
  EXPECT_EQ(widget->PropertyValue("collision_name::0").value<std::string>(),
      "col3");
  EXPECT_EQ(widget->PropertyValue("collision_id::1").value<unsigned int>(), 3u);
  EXPECT_DOUBLE_EQ(widget->PropertyValue("pressure::0").value<double>(), 3.1);

  // Check new message
  retMsg = dynamic_cast<msgs::Tactile *>(widget->Msg());
  ASSERT_NE(nullptr, retMsg);
  ASSERT_EQ(retMsg->collision_name_size(), 2);
  ASSERT_EQ(retMsg->collision_id_size(), 2);
  ASSERT_EQ(retMsg->pressure_size(), 2);

  EXPECT_EQ(retMsg->collision_name(0), "col3");
  EXPECT_EQ(retMsg->collision_id(0), 1u);
  EXPECT_DOUBLE_EQ(retMsg->pressure(0), 3.1);

  EXPECT_EQ(retMsg->collision_name(1), "col2");
  EXPECT_EQ(retMsg->collision_id(1), 3u);
  EXPECT_DOUBLE_EQ(retMsg->pressure(1), 2.1);

  // Update from message - remove entries
  msg.clear_collision_name();
  msg.clear_collision_id();
  msg.clear_pressure();

  msg.add_collision_name("col4");
  msg.add_collision_id(4);
  msg.add_pressure(4.1);

  widget->UpdateFromMsg(&msg);
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("collision_name::0"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("collision_id::0"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("pressure::0"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("collision_name::1"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("collision_id::1"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("pressure::1"));

  // Check new message
  retMsg = dynamic_cast<msgs::Tactile *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);

  ASSERT_EQ(retMsg->collision_name_size(), 1);
  ASSERT_EQ(retMsg->collision_id_size(), 1);
  ASSERT_EQ(retMsg->pressure_size(), 1);

  EXPECT_EQ(retMsg->collision_name(0), "col4");
  EXPECT_EQ(retMsg->collision_id(0), 4u);
  EXPECT_DOUBLE_EQ(retMsg->pressure(0), 4.1);

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
TEST(MessageWidgetTest, Visible)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  msgs::Visual msg;
  auto widget = new MessageWidget(&msg);
  ASSERT_NE(widget, nullptr);
  widget->show();

  // Check that only top-level widgets are visible by default
  {
    // Inexistent widget
    EXPECT_FALSE(widget->PropertyVisible("banana"));
    // Leaf widget
    EXPECT_TRUE(widget->PropertyVisible("id"));
    // Custom nested widgets
    EXPECT_TRUE(widget->PropertyVisible("pose"));
    EXPECT_TRUE(widget->PropertyVisible("geometry"));
    // Nested message widget
    EXPECT_TRUE(widget->PropertyVisible("material"));
    // Two levels deep message
    EXPECT_FALSE(widget->PropertyVisible("material::diffuse"));
    // Two levels deep message
    EXPECT_FALSE(widget->PropertyVisible("material::script"));
    // Three levels deep leaf
    EXPECT_FALSE(widget->PropertyVisible("material::script::name"));
    // Repeated field (none yet)
    EXPECT_TRUE(widget->PropertyVisible("plugin"));
    EXPECT_FALSE(widget->PropertyVisible("plugin::header"));
    EXPECT_FALSE(widget->PropertyVisible("plugin::0::header"));
    EXPECT_FALSE(widget->PropertyVisible("plugin::1::header"));
  }

  // Expand collapsible and check immediate children become visible
  {
    auto material = widget->PropertyWidgetByName("material");
    ASSERT_NE(nullptr, material);

    auto button = material->findChild<QPushButton *>();
    ASSERT_NE(nullptr, button);

    button->click();
    QCoreApplication::processEvents();

    EXPECT_TRUE(widget->PropertyVisible("material::diffuse"));
    EXPECT_TRUE(widget->PropertyVisible("material::script"));
    EXPECT_FALSE(widget->PropertyVisible("material::script::name"));
  }

  // Inexistent widget
  {
    EXPECT_FALSE(widget->SetPropertyVisible("banana", false));
  }

  // Top-level leaf
  {
    EXPECT_TRUE(widget->SetPropertyVisible("id", false));
    EXPECT_FALSE(widget->PropertyVisible("id"));

    EXPECT_TRUE(widget->SetPropertyVisible("id", true));
    EXPECT_TRUE(widget->PropertyVisible("id"));
  }

  // Top-level special message
  {
    EXPECT_TRUE(widget->SetPropertyVisible("pose", false));
    EXPECT_FALSE(widget->PropertyVisible("pose"));

    EXPECT_TRUE(widget->SetPropertyVisible("pose", true));
    EXPECT_TRUE(widget->PropertyVisible("pose"));
  }

  // Top-level collapsed collapsible
  {
    // Check child was already hidden
    EXPECT_FALSE(widget->PropertyVisible("meta::layer"));

    // Hiding collapsible keeps child hidden
    EXPECT_TRUE(widget->SetPropertyVisible("meta", false));
    EXPECT_FALSE(widget->PropertyVisible("meta"));
    EXPECT_FALSE(widget->PropertyVisible("meta::layer"));

    // Showing collapsed collapsible doesn't show child
    EXPECT_TRUE(widget->SetPropertyVisible("meta", true));
    EXPECT_TRUE(widget->PropertyVisible("meta"));
    EXPECT_FALSE(widget->PropertyVisible("meta::layer"));
  }

  // Top-level expanded collapsible
  {
    // Check immediate children were visible
    EXPECT_TRUE(widget->PropertyVisible("material"));
    EXPECT_TRUE(widget->PropertyVisible("material::diffuse"));
    EXPECT_TRUE(widget->PropertyVisible("material::script"));
    EXPECT_FALSE(widget->PropertyVisible("material::script::name"));

    // Hiding collapsible hides children
    EXPECT_TRUE(widget->SetPropertyVisible("material", false));
    EXPECT_FALSE(widget->PropertyVisible("material"));
    EXPECT_FALSE(widget->PropertyVisible("material::diffuse"));
    EXPECT_FALSE(widget->PropertyVisible("material::script"));
    EXPECT_FALSE(widget->PropertyVisible("material::script::name"));

    // Explicitly hide a child
    EXPECT_TRUE(widget->SetPropertyVisible("material::diffuse", false));
    EXPECT_FALSE(widget->PropertyVisible("material::diffuse"));

    // Showing expanded collapsible shows children except for those
    // explicitly hidden or still collapsed
    EXPECT_TRUE(widget->SetPropertyVisible("material", true));
    EXPECT_TRUE(widget->PropertyVisible("material"));
    EXPECT_FALSE(widget->PropertyVisible("material::diffuse"));
    EXPECT_TRUE(widget->PropertyVisible("material::script"));
    EXPECT_FALSE(widget->PropertyVisible("material::script::name"));

    // Can't set property visibility if widget has never been expanded
    // (i.e. created)
    EXPECT_FALSE(widget->SetPropertyVisible("material::script::name", true));
    EXPECT_FALSE(widget->PropertyVisible("material::script::name"));

    // Toggle
    auto script = widget->PropertyWidgetByName("material::script");
    ASSERT_NE(nullptr, script);

    auto button = script->findChild<QPushButton *>();
    ASSERT_NE(nullptr, button);

    button->click();
    QCoreApplication::processEvents();

    // Now it is visible
    EXPECT_TRUE(widget->PropertyVisible("material::script"));
    EXPECT_TRUE(widget->PropertyVisible("material::script::name"));
  }

  // Repeated field (new repetitions)
  {
    // Add a plugin
    msg.add_plugin();
    widget->UpdateFromMsg(&msg);

    // Check it isn't visible yet
    EXPECT_TRUE(widget->PropertyVisible("plugin"));
    EXPECT_FALSE(widget->PropertyVisible("plugin::header"));
    EXPECT_FALSE(widget->PropertyVisible("plugin::0"));
    EXPECT_FALSE(widget->PropertyVisible("plugin::0::header"));
    EXPECT_FALSE(widget->PropertyVisible("plugin::0::name"));
    EXPECT_FALSE(widget->PropertyVisible("plugin::1"));
    EXPECT_FALSE(widget->PropertyVisible("plugin::1::header"));
    EXPECT_FALSE(widget->PropertyVisible("plugin::1::name"));

    // Expand
    auto plugin = widget->PropertyWidgetByName("plugin");
    ASSERT_NE(nullptr, plugin);

    auto button = plugin->findChild<QPushButton *>();
    ASSERT_NE(nullptr, button);

    button->click();

    auto plugin0 = widget->PropertyWidgetByName("plugin::0");
    ASSERT_NE(nullptr, plugin0);

    button = plugin0->findChild<QPushButton *>();
    ASSERT_NE(nullptr, button);

    button->click();
    QCoreApplication::processEvents();

    // Check it is now visible
    EXPECT_TRUE(widget->PropertyVisible("plugin::0"));
    EXPECT_TRUE(widget->PropertyVisible("plugin::0::header"));
    EXPECT_TRUE(widget->PropertyVisible("plugin::0::name"));

    // Hide plugin headers
    EXPECT_TRUE(widget->SetPropertyVisible("plugin::header", false));

    // Check it was hidden for the repetition
    EXPECT_FALSE(widget->PropertyVisible("plugin::0::header"));

    // Collapse it again so the next plugin fits inside the screen
    button->click();

    // Add another plugin
    msg.add_plugin();
    widget->UpdateFromMsg(&msg);
    EXPECT_FALSE(widget->PropertyVisible("plugin::1"));
    EXPECT_FALSE(widget->PropertyVisible("plugin::1::header"));
    EXPECT_FALSE(widget->PropertyVisible("plugin::1::name"));

    // Expand it
    auto plugin1 = widget->PropertyWidgetByName("plugin::1");
    ASSERT_NE(nullptr, plugin1);

    button = plugin1->findChild<QPushButton *>();
    ASSERT_NE(nullptr, button);

    button->click();
    QCoreApplication::processEvents();

    // Check the plugin is visible, but without headers
    EXPECT_TRUE(widget->PropertyVisible("plugin::1"));
    EXPECT_FALSE(widget->PropertyVisible("plugin::1::header"));
    EXPECT_TRUE(widget->PropertyVisible("plugin::1::name"));
  }

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
TEST(MessageWidgetTest, ReadOnly)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  msgs::Visual msg;
  auto widget = new MessageWidget(&msg);
  ASSERT_NE(widget, nullptr);
  widget->show();
  widget->ToggleAll(true);

  // Check that all properties are read-write by default
  {
    // Whole widget
    EXPECT_FALSE(widget->ReadOnly());
    // Inexistent widget
    EXPECT_FALSE(widget->PropertyReadOnly("banana"));
    // Leaf widget
    EXPECT_FALSE(widget->PropertyReadOnly("id"));
    // Custom nested widgets
    EXPECT_FALSE(widget->PropertyReadOnly("pose"));
    EXPECT_FALSE(widget->PropertyReadOnly("geometry"));
    // Nested message widget
    EXPECT_FALSE(widget->PropertyReadOnly("material"));
    // Two levels deep message
    EXPECT_FALSE(widget->PropertyReadOnly("material::diffuse"));
    // Two levels deep message
    EXPECT_FALSE(widget->PropertyReadOnly("material::script"));
    // Three levels deep leaf
    EXPECT_FALSE(widget->PropertyReadOnly("material::script::name"));
  }

  // The whole widget
  {
    EXPECT_TRUE(widget->SetReadOnly(true));
    EXPECT_TRUE(widget->ReadOnly());
    EXPECT_TRUE(widget->PropertyReadOnly("id"));
    EXPECT_TRUE(widget->PropertyReadOnly("pose"));
    EXPECT_TRUE(widget->PropertyReadOnly("material"));
    EXPECT_TRUE(widget->PropertyReadOnly("material::script"));
    EXPECT_TRUE(widget->PropertyReadOnly("material::script::name"));

    EXPECT_TRUE(widget->SetReadOnly(false));
    EXPECT_FALSE(widget->ReadOnly());
    EXPECT_FALSE(widget->PropertyReadOnly("id"));
    EXPECT_FALSE(widget->PropertyReadOnly("pose"));
    EXPECT_FALSE(widget->PropertyReadOnly("material"));
    EXPECT_FALSE(widget->PropertyReadOnly("material::script"));
    EXPECT_FALSE(widget->PropertyReadOnly("material::script::name"));
  }

  // Inexistent widget
  {
    EXPECT_FALSE(widget->SetPropertyReadOnly("banana", false));
  }

  // Top-level leaf
  {
    EXPECT_TRUE(widget->SetPropertyReadOnly("id", true));
    EXPECT_TRUE(widget->PropertyReadOnly("id"));

    EXPECT_TRUE(widget->SetPropertyReadOnly("id", false));
    EXPECT_FALSE(widget->PropertyReadOnly("id"));
  }

  // Top-level special message
  {
    EXPECT_TRUE(widget->SetPropertyReadOnly("pose", true));
    EXPECT_TRUE(widget->PropertyReadOnly("pose"));

    EXPECT_TRUE(widget->SetPropertyReadOnly("pose", false));
    EXPECT_FALSE(widget->PropertyReadOnly("pose"));
  }

  // Top-level collapsible
  {
    EXPECT_TRUE(widget->SetPropertyReadOnly("material", true));
    EXPECT_TRUE(widget->PropertyReadOnly("material"));
    EXPECT_TRUE(widget->PropertyReadOnly("material::script"));
    EXPECT_TRUE(widget->PropertyReadOnly("material::script::name"));

    EXPECT_TRUE(widget->SetPropertyReadOnly("material", false));
    EXPECT_FALSE(widget->PropertyReadOnly("material::script"));
    EXPECT_FALSE(widget->PropertyReadOnly("material::script::name"));
  }

  // Repeated field (new repetitions)
  {
    // Add a plugin
    msg.add_plugin();
    widget->UpdateFromMsg(&msg);
    widget->ToggleAll(true);

    // Check it was created as write
    EXPECT_FALSE(widget->PropertyReadOnly("plugin"));
    EXPECT_FALSE(widget->PropertyReadOnly("plugin::header"));
    EXPECT_FALSE(widget->PropertyReadOnly("plugin::header::stamp"));
    EXPECT_FALSE(widget->PropertyReadOnly("plugin::0"));
    EXPECT_FALSE(widget->PropertyReadOnly("plugin::0::header"));
    EXPECT_FALSE(widget->PropertyReadOnly("plugin::0::name"));

    // Set headers to read-only
    EXPECT_TRUE(widget->SetPropertyReadOnly("plugin::header", true));

    // Check it affected the repetition
    EXPECT_TRUE(widget->PropertyReadOnly("plugin::0::header"));
    EXPECT_TRUE(widget->PropertyReadOnly("plugin::0::header::stamp"));
    EXPECT_FALSE(widget->PropertyReadOnly("plugin::0::name"));

    // Add another plugin
    msg.add_plugin();
    widget->UpdateFromMsg(&msg);
    widget->ToggleAll(true);

    // Check it was affected
    EXPECT_FALSE(widget->PropertyReadOnly("plugin::1"));
    EXPECT_TRUE(widget->PropertyReadOnly("plugin::1::header"));
    EXPECT_TRUE(widget->PropertyReadOnly("plugin::1::header::stamp"));
    EXPECT_FALSE(widget->PropertyReadOnly("plugin::1::name"));

    // Set whole widget
    EXPECT_TRUE(widget->SetReadOnly(true));

    EXPECT_TRUE(widget->PropertyReadOnly("plugin::1"));

    // Add a plugin
    msg.add_plugin();
    widget->UpdateFromMsg(&msg);
    widget->ToggleAll(true);

    // Check new plugin is read only, because whole widget is
    EXPECT_TRUE(widget->PropertyReadOnly("plugin::2"));
  }

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
TEST(MessageWidgetTest, ChildStringSignal)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  auto msg = new msgs::StringMsg();
  msg->set_data("banana");

  // Create widget from message
  auto widget = new MessageWidget(msg);
  ASSERT_NE(widget, nullptr);

  // Check we got a string widget
  auto propWidget = widget->PropertyWidgetByName("data");
  ASSERT_NE(propWidget, nullptr);

  auto stringWidget = qobject_cast<StringWidget *>(propWidget);
  ASSERT_NE(stringWidget, nullptr);

  // Connect signals
  bool signalReceived = false;
  widget->connect(widget, &MessageWidget::ValueChanged,
    [&signalReceived](const std::string &_name, QVariant _var)
    {
      auto v = _var.value<std::string>();
      EXPECT_EQ(_name, "data");
      EXPECT_EQ(v, "orange");
      signalReceived = true;
    });

  // Check default
  EXPECT_EQ(widget->PropertyValue("string").value<std::string>(), "");

  // Get signal emitting widgets
  auto lineEdits = stringWidget->findChildren<QLineEdit *>();
  EXPECT_EQ(lineEdits.size(), 1);

  // Change the value and check new value at callback
  lineEdits[0]->setText("orange");
  lineEdits[0]->editingFinished();

  // Check callback was called
  EXPECT_TRUE(signalReceived);

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
TEST(MessageWidgetTest, ChildNumberSignal)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  auto msg = new msgs::Double();
  msg->set_data(-1.5);

  // Create widget from message
  auto widget = new MessageWidget(msg);
  ASSERT_NE(widget, nullptr);

  // Check we got a number widget
  auto propWidget = widget->PropertyWidgetByName("data");
  ASSERT_NE(propWidget, nullptr);

  auto numberWidget = qobject_cast<NumberWidget *>(propWidget);
  ASSERT_NE(numberWidget, nullptr);

  // Connect signals
  bool signalReceived = false;
  widget->connect(widget, &MessageWidget::ValueChanged,
    [&signalReceived](const std::string &_name, QVariant _var)
    {
      auto v = _var.value<double>();
      EXPECT_EQ(_name, "data");
      EXPECT_DOUBLE_EQ(v, 0.999);
      signalReceived = true;
    });

  // Check value before
  EXPECT_DOUBLE_EQ(widget->PropertyValue("data").value<double>(), -1.5);

  // Get signal emitting widgets
  auto spins = widget->findChildren<QDoubleSpinBox *>();
  ASSERT_EQ(spins.size(), 1);

  // Change the value and check new value at callback
  spins[0]->setValue(0.999);
  spins[0]->editingFinished();

  // Check callback was called
  EXPECT_TRUE(signalReceived);

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
TEST(MessageWidgetTest, ChildBoolSignal)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  auto msg = new msgs::Boolean();
  msg->set_data(true);

  // Create widget from message
  auto widget = new MessageWidget(msg);
  ASSERT_NE(widget, nullptr);

  // Check we got a bool widget
  auto propWidget = widget->PropertyWidgetByName("data");
  ASSERT_NE(propWidget, nullptr);

  auto boolWidget = qobject_cast<BoolWidget *>(propWidget);
  ASSERT_NE(boolWidget, nullptr);

  // Connect signals
  bool signalReceived = false;
  widget->connect(widget, &MessageWidget::ValueChanged,
    [&signalReceived](const std::string &_name, QVariant _var)
    {
      auto v = _var.value<bool>();
      EXPECT_EQ(_name, "data");
      EXPECT_FALSE(v);
      signalReceived = true;
    });

  // Check value before
  EXPECT_TRUE(widget->PropertyValue("data").value<bool>());

  // Get signal emitting widgets
  auto radios = widget->findChildren<QRadioButton *>();
  EXPECT_EQ(radios.size(), 2);

  // Change the value and check new value at callback
  radios[0]->setChecked(false);
  radios[1]->setChecked(true);

  // Check callback was called
  EXPECT_TRUE(signalReceived);

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
TEST(MessageWidgetTest, ChildVector3dSignal)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  auto msg = new msgs::Vector3d();
  msg->set_x(1);
  msg->set_y(-2);
  msg->set_z(3);

  // Create widget from message
  auto widget = new MessageWidget(msg);
  ASSERT_NE(widget, nullptr);

  // Check we got a vector 3d widget
  auto propWidget = widget->PropertyWidgetByName("");
  ASSERT_NE(propWidget, nullptr);

  auto vector3Widget = qobject_cast<Vector3dWidget *>(propWidget);
  ASSERT_NE(vector3Widget, nullptr);

  // Connect signals
  int vector3SignalCount = 0;
  widget->connect(widget, &MessageWidget::ValueChanged,
    [&vector3SignalCount](const std::string &_name, QVariant _var)
    {
      auto v = _var.value<math::Vector3d>();

      EXPECT_EQ(_name, "");

      // From spins
      // cppcheck-suppress knownConditionTrueFalse
      if (vector3SignalCount == 0)
      {
        EXPECT_EQ(v, math::Vector3d(2.5, -2, 3));
        vector3SignalCount++;
      }
      // From preset combo
      else if (vector3SignalCount == 1)
      {
        EXPECT_EQ(v, math::Vector3d(0, -1, 0));
        vector3SignalCount++;
      }
    });

  // Check value before
  EXPECT_EQ(widget->PropertyValue("").value<math::Vector3d>(),
      math::Vector3d(1, -2, 3));

  // Get axes spins
  auto spins = vector3Widget->findChildren<QDoubleSpinBox *>();
  EXPECT_EQ(spins.size(), 3);

  // Get preset combo
  auto combos = vector3Widget->findChildren<QComboBox *>();
  EXPECT_EQ(combos.size(), 1);

  // Change the X value and check new value at callback
  EXPECT_EQ(vector3SignalCount, 0);
  spins[0]->setValue(2.5);
  spins[0]->editingFinished();
  EXPECT_EQ(vector3SignalCount, 1);

  // Change the preset value and check new value at callback
  combos[0]->setCurrentIndex(4);
  EXPECT_EQ(vector3SignalCount, 2);

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
TEST(MessageWidgetTest, ChildColorSignal)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  auto msg = new msgs::Color();
  msg->set_r(0.1);
  msg->set_g(0.2);
  msg->set_b(0.3);
  msg->set_a(0.4);

  // Create widget from message
  auto widget = new MessageWidget(msg);
  ASSERT_NE(widget, nullptr);

  // Check we got a color widget
  auto propWidget = widget->PropertyWidgetByName("");
  ASSERT_NE(propWidget, nullptr);

  auto colorWidget = qobject_cast<ColorWidget *>(propWidget);
  ASSERT_NE(colorWidget, nullptr);

  // Connect signals
  bool signalReceived = false;
  widget->connect(widget, &MessageWidget::ValueChanged,
    [&signalReceived](const std::string &_name, QVariant _var)
    {
      auto v = _var.value<math::Color>();
      EXPECT_EQ(_name, "");
      EXPECT_EQ(v, math::Color(1.0, 0.2, 0.3, 0.4));
      signalReceived = true;
    });

  // Check value before
  EXPECT_EQ(widget->PropertyValue("").value<math::Color>(),
      math::Color(0.1, 0.2, 0.3, 0.4));

  // Get signal emitting widgets
  auto spins = colorWidget->findChildren<QDoubleSpinBox *>();
  EXPECT_EQ(spins.size(), 4);

  // Change the X value and check new value at callback
  spins[0]->setValue(1.0);
  spins[0]->editingFinished();

  // Check callback was called
  EXPECT_TRUE(signalReceived);

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
TEST(MessageWidgetTest, ChildPoseSignal)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  auto msg = new msgs::Pose();
  msg->mutable_position()->set_x(0.1);
  msg->mutable_position()->set_y(0.2);
  msg->mutable_position()->set_z(0.3);
  msgs::Set(msg->mutable_orientation(),
            math::Quaterniond(-0.4, -0.5, -0.6));

  // Create widget from message
  auto widget = new MessageWidget(msg);
  ASSERT_NE(widget, nullptr);

  // Check we got a pose widget
  auto propWidget = widget->PropertyWidgetByName("");
  ASSERT_NE(propWidget, nullptr);

  auto poseWidget = qobject_cast<Pose3dWidget *>(propWidget);
  ASSERT_NE(poseWidget, nullptr);

  // Connect signals
  bool signalReceived = false;
  widget->connect(widget, &MessageWidget::ValueChanged,
    [&signalReceived](const std::string &_name, QVariant _var)
    {
      auto v = _var.value<math::Pose3d>();
      EXPECT_EQ(_name, "");
      EXPECT_EQ(v, math::Pose3d(1.0, 0.2, 0.3, -0.4, -0.5, -0.6));
      signalReceived = true;
    });

  // Check value before
  EXPECT_EQ(widget->PropertyValue("").value<math::Pose3d>(),
      math::Pose3d(0.1, 0.2, 0.3, -0.4, -0.5, -0.6));

  // Get signal emitting widgets
  auto spins = poseWidget->findChildren<QDoubleSpinBox *>();
  EXPECT_EQ(spins.size(), 6);

  // Change the X value and check new value at callback
  spins[0]->setValue(1.0);
  spins[0]->editingFinished();

  // Check callback was called
  EXPECT_TRUE(signalReceived);

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
TEST(MessageWidgetTest, ChildGeometrySignal)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  auto msg = new msgs::Geometry();
  msg->set_type(msgs::Geometry::CYLINDER);
  auto cylinder = msg->mutable_cylinder();
  cylinder->set_length(10.0);
  cylinder->set_radius(0.5);

  // Create widget from message
  auto widget = new MessageWidget(msg);
  EXPECT_TRUE(widget != nullptr);

  // Check we got a geometry widget
  auto propWidget = widget->PropertyWidgetByName("");
  EXPECT_NE(propWidget, nullptr);

  auto geometryWidget = qobject_cast<GeometryWidget *>(propWidget);
  EXPECT_NE(geometryWidget, nullptr);

  // Connect signals
  bool signalReceived = false;
  widget->connect(widget, &MessageWidget::ValueChanged,
    [&signalReceived](const std::string &_name, QVariant _var)
    {
      auto v = _var.value<msgs::Geometry>();
      EXPECT_EQ(_name, "");
      EXPECT_EQ(v.type(), msgs::Geometry::CYLINDER);
      EXPECT_DOUBLE_EQ(v.cylinder().radius(), 2.0);
      signalReceived = true;
    });

  // Check value before
  auto value = widget->PropertyValue("").value<msgs::Geometry>();
  EXPECT_EQ(value.type(), msgs::Geometry::CYLINDER);
  EXPECT_DOUBLE_EQ(value.cylinder().length(), 10.0);
  EXPECT_DOUBLE_EQ(value.cylinder().radius(), 0.5);

  // Get signal emitting widgets
  auto radiusWidget = widget->findChild<NumberWidget *>("cylinderRWidget");
  ASSERT_NE(nullptr, radiusWidget);

  auto spin = radiusWidget->findChild<QDoubleSpinBox *>();
  ASSERT_NE(nullptr, spin);

  // Change the value and check new value at callback
  spin->setValue(2.0);
  spin->editingFinished();

  // Check callback was called
  EXPECT_TRUE(signalReceived);

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
TEST(MessageWidgetTest, ChildEnumSignal)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  auto msg = new msgs::Visual();
  msg->set_type(msgs::Visual::LINK);

  // Create widget from message
  auto widget = new MessageWidget(msg);
  EXPECT_TRUE(widget != nullptr);

  // Check we got an enum widget
  auto propWidget = widget->PropertyWidgetByName("type");
  EXPECT_NE(propWidget, nullptr);

  auto enumWidget = qobject_cast<EnumWidget *>(propWidget);
  EXPECT_NE(enumWidget, nullptr);

  // Connect signals
  bool signalReceived = false;
  widget->connect(widget, &MessageWidget::ValueChanged,
    [&signalReceived](const std::string &_name, QVariant _var)
    {
      auto v = _var.value<std::string>();
      EXPECT_EQ(_name, "type");
      EXPECT_EQ(v, "GUI");
      signalReceived = true;
    });

  // Check value before
  EXPECT_EQ(widget->PropertyValue("type").value<std::string>(),
      std::string("LINK"));

  auto label = enumWidget->findChild<QLabel *>();
  EXPECT_NE(label, nullptr);
  EXPECT_EQ(label->text(), "Type");

  // Get signal emitting widgets
  auto comboBoxes = enumWidget->findChildren<QComboBox *>();
  EXPECT_EQ(comboBoxes.size(), 1);
  EXPECT_EQ(comboBoxes[0]->count(), 8);

  // Change the value and check new value at callback
  comboBoxes[0]->setCurrentIndex(6);
  comboBoxes[0]->currentIndexChanged(6);

  // Check callback was called
  EXPECT_TRUE(signalReceived);

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
TEST(MessageWidgetTest, PropertyByName)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  auto msg = new msgs::StringMsg();

  // Create widget from message
  auto widget = new MessageWidget(msg);
  ASSERT_NE(widget, nullptr);
  widget->ToggleAll(true);

  // Get generated widgets by name
  for (auto name : {"header", "header::stamp", "header::stamp::sec",
      "header::stamp::nsec",  "data"})
  {
    EXPECT_NE(widget->PropertyWidgetByName(name), nullptr) << name;
  }

  // Fail with invalid names
  for (auto name : {"", "banana"})
  {
    EXPECT_EQ(widget->PropertyWidgetByName(name), nullptr) << name;
  }

  // Set value of valid properties
  EXPECT_TRUE(widget->SetPropertyValue("data",
      QVariant::fromValue(std::string("the data value"))));
  EXPECT_EQ(widget->PropertyValue("data").value<std::string>(),
      std::string("the data value"));

  // Set value of invalid properties
  EXPECT_FALSE(widget->SetPropertyValue("banana",
      QVariant::fromValue(std::string("the banana value"))));
  EXPECT_EQ(widget->PropertyValue("banana").value<std::string>(),
      std::string(""));

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
TEST(MessageWidgetTest, TopicName)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Message
  auto msg = new msgs::StringMsg();

  // Create widget from message
  auto widget = new MessageWidget(msg);
  ASSERT_NE(widget, nullptr);

  std::string topic = "aTopic";
  EXPECT_TRUE(widget->Topic().empty());
  widget->SetTopic(topic);
  EXPECT_EQ(topic, widget->Topic());

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
TEST(MessageWidgetTest, ToggleAllSimpleMsg)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  // Fill message
  auto msg = new msgs::StringMsg();
  msg->set_data("acerola");

  auto headerMsg = msg->mutable_header();
  auto stampMsg = headerMsg->mutable_stamp();
  stampMsg->set_sec(3);
  stampMsg->set_nsec(300);

  // Create widget
  auto widget = new MessageWidget(msg);
  ASSERT_NE(widget, nullptr);

  // Has only top-level widgets
  EXPECT_EQ(2u, widget->PropertyWidgetCount());
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("header"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("header::stamp"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("header::stamp::sec"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("header::stamp::nsec"));
  EXPECT_EQ(nullptr, widget->PropertyWidgetByName("header::data"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data"));

  // Check message is complete even with collapsed widgets
  auto retMsg = dynamic_cast<msgs::StringMsg *>(widget->Msg());
  ASSERT_NE(retMsg, nullptr);

  EXPECT_EQ("acerola", retMsg->data());
  EXPECT_EQ(3, retMsg->header().stamp().sec());
  EXPECT_EQ(300, retMsg->header().stamp().nsec());

  // Can get/set top-level properties
  EXPECT_EQ("acerola", widget->PropertyValue("data").value<std::string>());
  EXPECT_TRUE(widget->SetPropertyValue("data",
      QVariant::fromValue(std::string("watermelon"))));

  // Can't get/set collapsed properties
  EXPECT_FALSE(widget->PropertyValue("header::stamp::sec").isValid());
  EXPECT_FALSE(widget->PropertyValue("header::stamp::nsec").isValid());
  EXPECT_FALSE(widget->SetPropertyValue("header::stamp::sec", 4));
  EXPECT_FALSE(widget->SetPropertyValue("header::stamp::nsec", 400));

  // Expand all
  widget->ToggleAll(true);

  // Has nested messages
  EXPECT_EQ(6u, widget->PropertyWidgetCount());
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("header"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("header::stamp"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("header::stamp::sec"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("header::stamp::nsec"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("header::data"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("data"));

  // Can get/set all properties
  EXPECT_EQ("watermelon", widget->PropertyValue("data").value<std::string>());
  EXPECT_EQ(3u,
      widget->PropertyValue("header::stamp::sec").value<unsigned int>());
  EXPECT_EQ(300u,
      widget->PropertyValue("header::stamp::nsec").value<unsigned int>());

  EXPECT_TRUE(widget->SetPropertyValue("data",
      QVariant::fromValue(std::string("orange"))));
  EXPECT_TRUE(widget->SetPropertyValue("header::stamp::sec", 5));
  EXPECT_TRUE(widget->SetPropertyValue("header::stamp::nsec", 500));

  // Collapse all - widgets don't get deleted
  widget->ToggleAll(false);
  EXPECT_EQ(6u, widget->PropertyWidgetCount());
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("header"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("header::stamp"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("header::stamp::sec"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("header::stamp::nsec"));
  EXPECT_NE(nullptr, widget->PropertyWidgetByName("header::data"));

  // Update field - widgets don't get deleted
  msg->set_data("banana");
  widget->UpdateFromMsg(msg);
  EXPECT_EQ(6u, widget->PropertyWidgetCount());
  widget->ToggleAll(true);
  EXPECT_EQ(6u, widget->PropertyWidgetCount());

  delete widget;
  EXPECT_TRUE(stop());
}

/////////////////////////////////////////////////
TEST(MessageWidgetTest, ToggleAllRepeatedField)
{
  setVerbosity(4);
  EXPECT_TRUE(initApp());

  auto msg = new msgs::Plugin_V();
  msg->add_plugins();
  msg->add_plugins();

  auto widget = new MessageWidget(msg);
  ASSERT_NE(widget, nullptr);

  EXPECT_EQ(2u, widget->PropertyWidgetCount());

  widget->ToggleAll(true);

  EXPECT_EQ(24u, widget->PropertyWidgetCount());

  // Collapse all - widgets don't get deleted
  widget->ToggleAll(false);
  EXPECT_EQ(24u, widget->PropertyWidgetCount());

  // Update field - widgets don't get created because it's collapsed
  msg->add_plugins();
  widget->UpdateFromMsg(msg);
  EXPECT_EQ(24u, widget->PropertyWidgetCount());

  // New widgets are created only when expanding
  widget->ToggleAll(true);
  EXPECT_EQ(33u, widget->PropertyWidgetCount());

  delete widget;
  EXPECT_TRUE(stop());
}
