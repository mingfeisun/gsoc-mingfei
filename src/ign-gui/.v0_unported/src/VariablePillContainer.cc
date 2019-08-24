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

#include <map>
#include <string>

#include <ignition/common/Console.hh>

#include "ignition/gui/VariablePill.hh"
#include "ignition/gui/VariablePillContainer.hh"

using namespace ignition;
using namespace gui;

/// \internal
/// \brief VariablePillContainer private data
class ignition::gui::VariablePillContainerPrivate
{
  /// \brief Text label
  public: QLabel *label;

  /// \brief Layout that contains all the variable pills.
  public: QLayout *variableLayout;

  /// \brief Variables inside this container
  public: std::map<unsigned int, VariablePill *> variables;

  /// \brief Container size.
  public: int maxSize = -1;

  /// \brief Pointer to variable pill that is currently selected.
  public: VariablePill *selectedVariable = nullptr;
};

/////////////////////////////////////////////////
VariablePillContainer::VariablePillContainer(QWidget *_parent)
  : QWidget(_parent),
    dataPtr(new VariablePillContainerPrivate)
{
  // label
  this->dataPtr->label = new QLabel;
  QHBoxLayout *labelLayout = new QHBoxLayout;
  labelLayout->addWidget(this->dataPtr->label);
  QMargins labelMargins = labelLayout->contentsMargins();
  labelMargins.setLeft(labelMargins.left() + 10);
  labelLayout->setContentsMargins(labelMargins);

  // variable pills
  this->dataPtr->variableLayout = new QHBoxLayout;
  this->dataPtr->variableLayout->setAlignment(Qt::AlignLeft);

  QHBoxLayout *frameLayout = new QHBoxLayout;
  frameLayout->addLayout(labelLayout);
  frameLayout->addLayout(this->dataPtr->variableLayout);
  frameLayout->setAlignment(Qt::AlignLeft);
  frameLayout->setContentsMargins(8, 4, 8, 4);
  QFrame *mainFrame = new QFrame;
  mainFrame->setObjectName("variableContainerFrame");
  mainFrame->setLayout(frameLayout);
  mainFrame->setFrameShape(QFrame::NoFrame);
  mainFrame->setContentsMargins(0, 0, 0, 0);

  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addWidget(mainFrame);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setSpacing(0);

  this->setLayout(mainLayout);
  this->setAcceptDrops(true);
}

/////////////////////////////////////////////////
VariablePillContainer::~VariablePillContainer()
{
}

/////////////////////////////////////////////////
void VariablePillContainer::SetText(const std::string &_text)
{
  this->dataPtr->label->setText(QString::fromStdString(_text));
}

/////////////////////////////////////////////////
std::string VariablePillContainer::Text() const
{
  return this->dataPtr->label->text().toStdString();
}

/////////////////////////////////////////////////
void VariablePillContainer::SetVariablePillLabel(const unsigned int _id,
    const std::string &_label)
{
  gui::VariablePill *variable = nullptr;
  auto it = this->dataPtr->variables.find(_id);
  if (it == this->dataPtr->variables.end())
  {
    // look into children of multi-variable pills
    for (auto v : this->dataPtr->variables)
    {
      auto &childVariables = v.second->VariablePills();
      auto childIt = childVariables.find(_id);
      if (childIt != childVariables.end())
      {
        variable = childIt->second;
        break;
      }
    }
  }
  else
  {
    variable = it->second;
  }

  if (variable)
    variable->SetText(_label);
}

/////////////////////////////////////////////////
unsigned int VariablePillContainer::AddVariablePill(const std::string &_name,
    const unsigned int _targetId)
{
  gui::VariablePill *variable = new gui::VariablePill;
  variable->SetName(_name);
  variable->SetText(_name);

  this->AddVariablePill(variable, _targetId);

  return variable->Id();
}

/////////////////////////////////////////////////
void VariablePillContainer::AddVariablePill(gui::VariablePill *_variable,
    const unsigned int _targetId)
{
  if (!_variable)
    return;

  // add to target variable if it's not empty
  if (_targetId != VariablePill::EmptyVariable)
  {
    gui::VariablePill *targetVariable = this->VariablePill(_targetId);
    if (!targetVariable)
    {
      ignerr << "Unable to add variable. Target variable not found"
             << std::endl;
      return;
    }

    targetVariable->AddVariablePill(_variable);
    return;
  }
  else
  {
    // check if variable already exists in this container
    // check only top level variables
    if (this->dataPtr->variables.find(_variable->Id()) !=
        this->dataPtr->variables.end())
    {
      return;
    }
  }

  // Reached max size
  if (this->dataPtr->maxSize != -1 &&
      static_cast<int>(this->VariablePillCount()) >= this->dataPtr->maxSize)
  {
    ignerr << "Unable to add variable to container. Container is full"
           << std::endl;
    return;
  }

  this->connect(_variable, SIGNAL(VariableMoved(unsigned int)),
      this, SLOT(OnMoveVariable(unsigned int)));
  this->connect(_variable, SIGNAL(VariableAdded(unsigned int, std::string)),
      this, SLOT(OnAddVariable(unsigned int, std::string)));
  this->connect(_variable, SIGNAL(VariableRemoved(unsigned int)),
      this, SLOT(OnRemoveVariable(unsigned int)));
  this->connect(_variable, SIGNAL(VariableTextChanged(std::string)),
      this, SLOT(OnSetVariableLabel(std::string)));

  _variable->SetContainer(this);
  _variable->setVisible(true);
  this->dataPtr->variableLayout->addWidget(_variable);
  this->dataPtr->variables[_variable->Id()] = _variable;

  this->VariableAdded(_variable->Id(), _variable->Text(), _targetId);
}

/////////////////////////////////////////////////
void VariablePillContainer::SetMaxSize(const int _max)
{
  this->dataPtr->maxSize = _max;
}

/////////////////////////////////////////////////
int VariablePillContainer::MaxSize() const
{
  return this->dataPtr->maxSize;
}

/////////////////////////////////////////////////
void VariablePillContainer::RemoveVariablePill(const unsigned int _id)
{
  gui::VariablePill *variable = nullptr;
  auto it = this->dataPtr->variables.find(_id);
  if (it == this->dataPtr->variables.end())
  {
    // look into children of multi-variable pills
    for (auto v : this->dataPtr->variables)
    {
      auto &childVariables = v.second->VariablePills();
      auto childIt = childVariables.find(_id);
      if (childIt != childVariables.end())
      {
        variable = childIt->second;
        // remove from parent
        if (variable->Parent())
          variable->Parent()->RemoveVariablePill(variable);
        return;
      }
    }
  }
  else
    variable = it->second;

  if (!variable)
    return;

  int idx = this->dataPtr->variableLayout->indexOf(variable);
  if (idx != -1)
  {
    this->dataPtr->variableLayout->takeAt(idx);
    this->dataPtr->variables.erase(variable->Id());
    // remove from parent if any
    if (variable->Parent())
    {
      // remove and rely on callbacks to emit the VariableRemoved signal
      variable->Parent()->RemoveVariablePill(variable);
    }
    else
      this->VariableRemoved(variable->Id(), VariablePill::EmptyVariable);
  }

  // otherwise remove from container
  variable->SetContainer(nullptr);
  variable->setVisible(false);
}

/////////////////////////////////////////////////
void VariablePillContainer::RemoveVariablePill(gui::VariablePill *_variable)
{
  if (!_variable)
    return;

  this->RemoveVariablePill(_variable->Id());
}

/////////////////////////////////////////////////
unsigned int VariablePillContainer::VariablePillCount() const
{
  unsigned int count = 0;
  for (const auto v : this->dataPtr->variables)
  {
    count++;
    count += v.second->VariablePillCount();
  }
  return count;
}

/////////////////////////////////////////////////
VariablePill *VariablePillContainer::VariablePill(
    const unsigned int _id) const
{
  for (const auto &v : this->dataPtr->variables)
  {
    if (v.first == _id)
      return v.second;

    for (const auto &child : v.second->VariablePills())
    {
      if (child.first == _id)
        return child.second;
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
void VariablePillContainer::SetSelected(gui::VariablePill *_variable)
{
  if (this->dataPtr->selectedVariable)
    this->dataPtr->selectedVariable->SetSelected(false);

  this->dataPtr->selectedVariable = _variable;

  if (this->dataPtr->selectedVariable)
    this->dataPtr->selectedVariable->SetSelected(true);
}

/////////////////////////////////////////////////
void VariablePillContainer::dragEnterEvent(QDragEnterEvent *_evt)
{
  if (!this->IsDragValid(_evt))
  {
    _evt->ignore();
    return;
  }

  if (_evt->mimeData()->hasFormat("application/x-item"))
  {
    _evt->setDropAction(Qt::LinkAction);
  }
  else if (_evt->mimeData()->hasFormat("application/x-pill-item"))
  {
    _evt->setDropAction(Qt::MoveAction);
  }
  else
  {
    _evt->ignore();
    return;
  }

  _evt->acceptProposedAction();
}

/////////////////////////////////////////////////
void VariablePillContainer::dropEvent(QDropEvent *_evt)
{
  if (!this->IsDragValid(_evt))
  {
    _evt->ignore();
    return;
  }

  if (_evt->mimeData()->hasFormat("application/x-item"))
  {
    QString mimeData = _evt->mimeData()->data("application/x-item");
    std::string dataStr = mimeData.toStdString();
    this->AddVariablePill(dataStr);
  }
  else if (_evt->mimeData()->hasFormat("application/x-pill-item"))
  {
    gui::VariablePill *variable =
      qobject_cast<gui::VariablePill *>(_evt->source());
    if (!variable)
    {
      ignerr << "Variable is nullptr" << std::endl;
      return;
    }

    VariablePillContainer *container = variable->Container();

    // moved to the same container - no op
    if (!variable->Parent() && (container && container == this))
      return;

    // block signals and emit VariableMoved instead.
    gui::VariablePill *parentVariable = variable->Parent();
    if (parentVariable)
    {
      parentVariable->blockSignals(true);
      parentVariable->RemoveVariablePill(variable);
      parentVariable->blockSignals(false);
    }
    else
    {
      if (container)
      {
        container->blockSignals(true);
        container->RemoveVariablePill(variable);
        container->blockSignals(false);
      }
    }

    // case when the variable is dragged out from a muli-variable pill to
    // the container
    this->blockSignals(true);
    this->AddVariablePill(variable);
    this->blockSignals(false);

    this->VariableMoved(variable->Id(), VariablePill::EmptyVariable);
  }
}

/////////////////////////////////////////////////
bool VariablePillContainer::IsDragValid(const QDropEvent *_evt) const
{
  if (this->dataPtr->maxSize != -1 &&
      static_cast<int>(this->VariablePillCount()) >= this->dataPtr->maxSize)
  {
    return false;
  }

  std::string dataStr;
  if (_evt->mimeData()->hasFormat("application/x-item"))
  {
    QString mimeData = _evt->mimeData()->data("application/x-item");
    dataStr = mimeData.toStdString();
  }
  else if (_evt->mimeData()->hasFormat("application/x-pill-item"))
  {
    QString mimeData = _evt->mimeData()->data("application/x-pill-item");
    dataStr = mimeData.toStdString();

    gui::VariablePill *dragVariable =
      qobject_cast<gui::VariablePill *>(_evt->source());
    if (!dragVariable)
      return false;

    // limit drag and drop to same container
    if (dragVariable->Container() && dragVariable->Container() != this)
      return false;
  }
  else
    return false;

  if (dataStr.empty())
    return false;

  return true;
}

/////////////////////////////////////////////////
void VariablePillContainer::keyPressEvent(QKeyEvent *_event)
{
  if (_event->key() == Qt::Key_Delete)
  {
    if (this->dataPtr->selectedVariable)
    {
      this->RemoveVariablePill(this->dataPtr->selectedVariable);
      this->dataPtr->selectedVariable = nullptr;
    }
  }
}

/////////////////////////////////////////////////
void VariablePillContainer::mouseReleaseEvent(QMouseEvent *_event)
{
  this->SetSelected(nullptr);

  bool selected = false;
  for (const auto v : this->dataPtr->variables)
  {
    // look for the selected variable widget if not already found
    QPoint point = v.second->mapFromParent(_event->pos());
    if (!selected)
    {
      ignition::math::Vector2i pt(point.x(), point.y());
      if (v.second->ContainsPoint(pt))
      {
        this->SetSelected(v.second);
        this->setFocus();
        selected = true;
      }
      else
      {
        v.second->SetSelected(false);
      }
    }
    else
    {
      v.second->SetSelected(false);
    }


    // loop through children of multi-variable pills
    for (const auto cv : v.second->VariablePills())
    {
      gui::VariablePill *child = cv.second;

      if (!selected)
      {
        QPoint childPoint = child->mapFromParent(point);
        ignition::math::Vector2i childPt(childPoint.x(), childPoint.y());
        if (child->ContainsPoint(childPt))
        {
          this->SetSelected(child);
          this->setFocus();
          selected = true;
        }
        else
        {
          child->SetSelected(false);
        }
      }
      else
      {
        child->SetSelected(false);
      }
    }
  }
}

/////////////////////////////////////////////////
void VariablePillContainer::OnMoveVariable(const unsigned int _id)
{
  gui::VariablePill *variable =
    qobject_cast<gui::VariablePill *>(QObject::sender());
  if (!variable)
    return;

  this->VariableMoved(_id, variable->Id());
}

/////////////////////////////////////////////////
void VariablePillContainer::OnAddVariable(const unsigned int _id,
    const std::string &_label)
{
  gui::VariablePill *variable =
    qobject_cast<gui::VariablePill *>(QObject::sender());
  if (!variable)
    return;

  this->VariableAdded(_id, _label, variable->Id());
}

/////////////////////////////////////////////////
void VariablePillContainer::OnRemoveVariable(const unsigned int _id)
{
  gui::VariablePill *variable =
    qobject_cast<gui::VariablePill *>(QObject::sender());
  if (!variable)
    return;

  this->VariableRemoved(_id, variable->Id());
}

/////////////////////////////////////////////////
void VariablePillContainer::OnSetVariableLabel(const std::string &_label)
{
  gui::VariablePill *variable =
    qobject_cast<gui::VariablePill *>(QObject::sender());
  if (!variable)
    return;

  this->VariableLabelChanged(variable->Id(), _label);
}
