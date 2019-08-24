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

#include <ignition/common/Console.hh>
#include <ignition/math/Vector3.hh>

#include "ignition/gui/Helpers.hh"
#include "ignition/gui/QtMetatypes.hh"

#include "ignition/gui/Vector3dWidget.hh"

namespace ignition
{
  namespace gui
  {
    class PropertyWidget;

    /// \brief Private data for the Vector3dWidget class.
    class Vector3dWidgetPrivate
    {
    };
  }
}

using namespace ignition;
using namespace gui;

/////////////////////////////////////////////////
Vector3dWidget::Vector3dWidget(const std::string &_key)
    : dataPtr(new Vector3dWidgetPrivate())
{
  // Presets
  auto presetsCombo = new QComboBox(this);
  presetsCombo->addItem("Custom", 0);
  presetsCombo->addItem("Unit  X", 1);
  presetsCombo->addItem("Unit -X", 2);
  presetsCombo->addItem("Unit  Y", 3);
  presetsCombo->addItem("Unit -Y", 4);
  presetsCombo->addItem("Unit  Z", 5);
  presetsCombo->addItem("Unit -Z", 6);
  presetsCombo->setMinimumWidth(80);
  presetsCombo->setToolTip("Unit vector presets");
  this->connect(presetsCombo, SIGNAL(currentIndexChanged(const int)), this,
      SLOT(OnPresetChanged(const int)));

  // Labels
  auto vecXLabel = new QLabel(tr("X"));
  auto vecYLabel = new QLabel(tr("Y"));
  auto vecZLabel = new QLabel(tr("Z"));
  vecXLabel->setToolTip(tr("x"));
  vecYLabel->setToolTip(tr("y"));
  vecZLabel->setToolTip(tr("z"));

  // Units
  auto unit = unitFromKey(_key);

  auto unitXLabel = new QLabel();
  unitXLabel->setMaximumWidth(40);
  unitXLabel->setText(QString::fromStdString(unit));

  auto unitYLabel = new QLabel();
  unitYLabel->setMaximumWidth(40);
  unitYLabel->setText(QString::fromStdString(unit));

  auto unitZLabel = new QLabel();
  unitZLabel->setMaximumWidth(40);
  unitZLabel->setText(QString::fromStdString(unit));

  // SpinBoxes
  double min = 0;
  double max = 0;
  rangeFromKey(_key, min, max);

  auto vecXSpinBox = new QDoubleSpinBox(this);
  vecXSpinBox->setRange(min, max);
  vecXSpinBox->setSingleStep(0.01);
  vecXSpinBox->setDecimals(6);
  vecXSpinBox->setAlignment(Qt::AlignRight);
  vecXSpinBox->setMaximumWidth(100);
  this->connect(vecXSpinBox, SIGNAL(valueChanged(double)), this,
      SLOT(OnSpinChanged()));
  this->connect(vecXSpinBox, SIGNAL(editingFinished()), this,
      SLOT(OnSpinFinished()));

  auto vecYSpinBox = new QDoubleSpinBox(this);
  vecYSpinBox->setRange(min, max);
  vecYSpinBox->setSingleStep(0.01);
  vecYSpinBox->setDecimals(6);
  vecYSpinBox->setAlignment(Qt::AlignRight);
  vecYSpinBox->setMaximumWidth(100);
  this->connect(vecYSpinBox, SIGNAL(valueChanged(double)), this,
      SLOT(OnSpinChanged()));
  this->connect(vecYSpinBox, SIGNAL(editingFinished()), this,
      SLOT(OnSpinFinished()));

  auto vecZSpinBox = new QDoubleSpinBox(this);
  vecZSpinBox->setRange(min, max);
  vecZSpinBox->setSingleStep(0.01);
  vecZSpinBox->setDecimals(6);
  vecZSpinBox->setAlignment(Qt::AlignRight);
  vecZSpinBox->setMaximumWidth(100);
  this->connect(vecZSpinBox, SIGNAL(valueChanged(double)), this,
      SLOT(OnSpinChanged()));
  this->connect(vecZSpinBox, SIGNAL(editingFinished()), this,
      SLOT(OnSpinFinished()));

  // Layout
  auto widgetLayout = new QHBoxLayout();
  widgetLayout->addWidget(presetsCombo);
  widgetLayout->addWidget(vecXLabel);
  widgetLayout->addWidget(vecXSpinBox);
  widgetLayout->addWidget(unitXLabel);
  widgetLayout->addWidget(vecYLabel);
  widgetLayout->addWidget(vecYSpinBox);
  widgetLayout->addWidget(unitYLabel);
  widgetLayout->addWidget(vecZLabel);
  widgetLayout->addWidget(vecZSpinBox);
  widgetLayout->addWidget(unitZLabel);

  widgetLayout->setAlignment(vecXLabel, Qt::AlignRight);
  widgetLayout->setAlignment(vecYLabel, Qt::AlignRight);
  widgetLayout->setAlignment(vecZLabel, Qt::AlignRight);

  this->setLayout(widgetLayout);
}

/////////////////////////////////////////////////
Vector3dWidget::~Vector3dWidget()
{
}

/////////////////////////////////////////////////
bool Vector3dWidget::SetValue(const QVariant _value)
{
  if (!_value.canConvert<math::Vector3d>())
  {
    ignerr << "Wrong variant type, expected [ignition::math::Vector3d]"
           << std::endl;
    return false;
  }

  auto value = _value.value<ignition::math::Vector3d>();

  auto spins = this->findChildren<QDoubleSpinBox *>();

  spins[0]->setValue(value.X());
  spins[1]->setValue(value.Y());
  spins[2]->setValue(value.Z());

  this->UpdatePreset();

  return true;
}

/////////////////////////////////////////////////
QVariant Vector3dWidget::Value() const
{
  math::Vector3d value;

  auto spins = this->findChildren<QDoubleSpinBox *>();

  value.X(spins[0]->value());
  value.Y(spins[1]->value());
  value.Z(spins[2]->value());

  QVariant v;
  v.setValue(value);

  return v;
}

/////////////////////////////////////////////////
void Vector3dWidget::OnSpinFinished()
{
  this->UpdatePreset();
  this->OnValueChanged();
}

/////////////////////////////////////////////////
void Vector3dWidget::OnSpinChanged()
{
  this->UpdatePreset();
  // Only emit a value changed signal on edit finish
}

/////////////////////////////////////////////////
void Vector3dWidget::UpdatePreset()
{
  auto value = this->Value().value<math::Vector3d>();

  int preset{0};
  if (value == math::Vector3d::UnitX)
    preset = 1;
  else if (value == -math::Vector3d::UnitX)
    preset = 2;
  else if (value == math::Vector3d::UnitY)
    preset = 3;
  else if (value == -math::Vector3d::UnitY)
    preset = 4;
  else if (value == math::Vector3d::UnitZ)
    preset = 5;
  else if (value == -math::Vector3d::UnitZ)
    preset = 6;

  auto combo = this->findChild<QComboBox *>();

  combo->blockSignals(true);
  combo->setCurrentIndex(preset);
  combo->blockSignals(false);
}

/////////////////////////////////////////////////
void Vector3dWidget::OnPresetChanged(const int _index)
{
  // Update spins
  math::Vector3d vec;
  if (_index == 1)
    vec = math::Vector3d::UnitX;
  else if (_index == 2)
    vec = -math::Vector3d::UnitX;
  else if (_index == 3)
    vec = math::Vector3d::UnitY;
  else if (_index == 4)
    vec = -math::Vector3d::UnitY;
  else if (_index == 5)
    vec = math::Vector3d::UnitZ;
  else if (_index == 6)
    vec = -math::Vector3d::UnitZ;
  else
    return;

  // Signal
  auto v = QVariant::fromValue(vec);

  this->SetValue(v);
  this->ValueChanged(v);
}
