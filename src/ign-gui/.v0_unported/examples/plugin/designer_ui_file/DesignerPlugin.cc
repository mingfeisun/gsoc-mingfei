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

#include <iostream>
#include <ignition/plugin/Register.hh>

#include "DesignerPlugin.hh"
#include "ui_DesignerPlugin.h"

using namespace ignition;
using namespace gui;

/////////////////////////////////////////////////
DesignerPlugin::DesignerPlugin() : Plugin(), ui(new Ui::DesignerPlugin)
{
  ui->setupUi(this);
}

/////////////////////////////////////////////////
DesignerPlugin::~DesignerPlugin()
{
  delete ui;
}

/////////////////////////////////////////////////
void DesignerPlugin::on_helloButton_clicked()
{
  std::cout << "Hello, UI!" << std::endl;
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gui::DesignerPlugin,
                    ignition::gui::Plugin);
