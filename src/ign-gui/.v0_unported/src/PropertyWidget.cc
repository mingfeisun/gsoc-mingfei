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

#include <string>

#include "ignition/gui/PropertyWidget.hh"

namespace ignition
{
  namespace gui
  {
    class PropertyWidgetPrivate
    {
      /// \brief This becomes true once read-only has been explicitly set
      /// and never goes back to false.
      public: bool explicitReadOnly = false;

      /// \brief The URI used as mime data for drag and drop.
      public: std::string dragAndDropURI = "";
    };
  }
}

using namespace ignition;
using namespace gui;

/////////////////////////////////////////////////
PropertyWidget::PropertyWidget()
    : dataPtr(new PropertyWidgetPrivate)
{
}

/////////////////////////////////////////////////
PropertyWidget::~PropertyWidget()
{
}

/////////////////////////////////////////////////
void PropertyWidget::OnValueChanged()
{
  this->ValueChanged(this->Value());
}

/////////////////////////////////////////////////
void PropertyWidget::SetReadOnly(const bool _readOnly,
    const bool _explicit)
{
  if (_explicit)
    this->dataPtr->explicitReadOnly = true;

  if (_explicit || !this->dataPtr->explicitReadOnly)
    this->setEnabled(!_readOnly);
}

/////////////////////////////////////////////////
bool PropertyWidget::ReadOnly() const
{
  return !this->isEnabled();
}

/////////////////////////////////////////////////
std::string PropertyWidget::DragAndDropURI() const
{
  return this->dataPtr->dragAndDropURI;
}

/////////////////////////////////////////////////
void PropertyWidget::SetDragAndDropURI(const std::string &_uri)
{
  this->dataPtr->dragAndDropURI = _uri;
}

/////////////////////////////////////////////////
bool PropertyWidget::eventFilter(QObject *_object, QEvent *_event)
{
  QVariant value = _object->property("uri");
  std::string uri = value.toString().toStdString();

  if (_event->type() == QEvent::MouseButtonPress && !uri.empty())
  {
    std::string str = this->dataPtr->dragAndDropURI + uri;
    QString textData(str.c_str());

    QMimeData *mimeDataLocal = new QMimeData;
    mimeDataLocal->setData("application/x-item", textData.toLocal8Bit());
    mimeDataLocal->setText(textData);

    QDrag *drag = new QDrag(this);
    drag->setMimeData(mimeDataLocal);
    drag->exec(Qt::MoveAction);
  }

  return false;
}
