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
#ifndef IGNITION_GUI_PLUGINS_PLOT_TYPES_HH_
#define IGNITION_GUI_PLUGINS_PLOT_TYPES_HH_

#include <memory>
#include <set>

namespace ignition
{
namespace gui
{
namespace plugins
{
namespace plot
{
  /// \brief Plotting tool forward declarations and type defines.
  class Curve;

  /// \def CurvePtr
  /// \brief std shared pointer to a Curve object
  typedef std::shared_ptr<Curve> CurvePtr;

  /// \def CurveWeakPtr
  /// \brief std weak pointer to a Curve object
  typedef std::weak_ptr<Curve> CurveWeakPtr;

  /// \def CurveVariableSet
  /// \brief A set of unique plot curve pointers
  using CurveVariableSet = std::set<CurveWeakPtr,
    std::owner_less<CurveWeakPtr>>;
}
}
}
}

#endif
