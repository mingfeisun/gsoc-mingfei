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

#ifndef IGNITION_GUI_PLUGINS_PLOT_TRACKER_HH_
#define IGNITION_GUI_PLUGINS_PLOT_TRACKER_HH_

#include <memory>

#include "ignition/gui/qt.h"
#include "ignition/gui/qwt.h"
#include "ignition/gui/Export.hh"

namespace ignition
{
namespace gui
{
namespace plugins
{
namespace plot
{
  // Forward declarations.
  class TrackerPrivate;

  /// \brief Mouse hover tracking.
  class IGNITION_GUI_VISIBLE Tracker: public QwtPlotPicker
  {
    /// \brief Constructor.
    /// \param[in] _canvas Plot the tracker will be attached to.
    public: explicit Tracker(QWidget *_plot);

    /// \brief Overriden to update the hover line and tracker text.
    protected: virtual void updateDisplay() override;

    /// \brief Overriden to provide customized hover text.
    /// \param[in] _pos Mouse position.
    /// \return Text to display.
    protected: virtual QwtText trackerTextF(const QPointF &_pos) const override;

    /// \brief Mouse press event used to determine when to show/hide hover line.
    /// \param[in] _e Qt mouse event.
    protected: virtual void widgetMousePressEvent(QMouseEvent *_e) override;

    /// \brief Mouse release event used to determine when to show/hide hover
    /// line.
    /// \param[in] _e Qt mouse event.
    protected: virtual void widgetMouseReleaseEvent(QMouseEvent *_e) override;

    /// \brief Get curve information at a point.
    /// \param[in] _curve Plot curve instance.
    /// \param[in] _pos Mouse position.
    /// \return Curve information.
    private: QString CurveInfoAt(const QwtPlotCurve *_curve,
                                 const QPointF &_pos) const;

    /// \brief Get the curve line segment at position x.
    /// \param[in] _curve Plot curve instance.
    /// \param[in] _x X mouse position.
    /// \return Line segment.
    private: QLineF CurveLineAt(const QwtPlotCurve *_curve,
                                const double _x) const;

    /// \brief Get the index of sample that is the upper bound of value.
    /// This is ported from qwt-6.1.
    /// \param[in] _series The plot series data.
    /// \param[in] _value Value to search.
    /// \return Sample index.
    private: int UpperSampleIndex(const QwtSeriesData<QPointF> &_series,
                                  const double _value) const;

    /// \internal
    /// \brief Private data pointer.
    private: std::unique_ptr<TrackerPrivate> dataPtr;
  };
}
}
}
}

#endif
