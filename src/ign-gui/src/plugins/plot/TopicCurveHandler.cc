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

#include <google/protobuf/message.h>
#include <map>
#include <mutex>
#include <string>
#include <ignition/common/Console.hh>
#include <ignition/common/SingletonT.hh>
#include <ignition/common/Time.hh>
#include <ignition/common/URI.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "ignition/gui/Conversions.hh"
#include "ignition/gui/plugins/plot/Curve.hh"
#include "ignition/gui/plugins/plot/Types.hh"

#include "ignition/gui/plugins/plot/TopicCurveHandler.hh"

using namespace ignition;
using namespace gui;
using namespace plugins;
using namespace plot;

namespace ignition
{
namespace gui
{
namespace plugins
{
namespace plot
{
  /// \brief Helper class to update curves associated with a single topic
  class TopicCurve
  {
    /// \def CurveVariableMapIt
    /// \brief Curve variable map iterator
    public: using CurveVariableMapIt =
        std::map<std::string, CurveVariableSet>::iterator;

    /// \brief Constructor.
    /// \param[in] _topic Ignition transport topic name.
    public: explicit TopicCurve(const std::string &_topic);

    /// \brief Destructor.
    public: ~TopicCurve();

    /// \brief Add a curve to be updated
    /// \param[in] _query URI query string containing the param the curve is
    /// associated with.
    /// \param[in] _curve Pointer to the plot curve to add.
    /// \return True if successful.
    public: bool AddCurve(const std::string &_query, CurveWeakPtr _curve);

    /// \brief Remove a curve from the topic data handler
    /// \param[in] _curve Pointer to the plot curve to remove.
    /// \return True if successful.
    public: bool RemoveCurve(CurveWeakPtr _curve);

    /// \brief Get whether the topic curve has the specified plot curve.
    /// \param[in] _curve Pointer to the plot curve
    /// \return True if curve exists
    public: bool HasCurve(CurveWeakPtr _curve) const;

    /// \brief Get the number of curves managed by this topic curve class.
    /// \return Number of curves
    public: unsigned int CurveCount() const;

    /// \brief Topic data callback
    /// \param[in] _msg Message data.
    /// \param[in] _info Message info, not used.
    public: void OnTopicData(const google::protobuf::Message &_msg,
        const ignition::transport::MessageInfo &_info);

    /// \brief Update the plot curve based on message
    /// \param[in] _msg Message containing data to be added to the curve
    /// \param[in] _index Index of token in the param path string
    /// \param[in] _x X value. Only used if the topic data is not timestamped
    /// \param[in] _curvesUpdates A list of curves and values to update
    public: void UpdateCurve(google::protobuf::Message *_msg,
                const int _index, const double _x,
                std::vector<std::pair<TopicCurve::CurveVariableMapIt,
                    ignition::math::Vector2d> > &_curvesUpdates);

    /// \brief Topic name
    private: std::string topic;

    /// \brief Keep track of time when curve was created.
    private: double startTime;

    /// \brief Node for communications.
    private: transport::Node node;

    /// \brief Mutex to protect the topic data updates.
    private: mutable std::mutex mutex;

    /// \brief A map of param names to plot curves.
    private: std::map<std::string, CurveVariableSet> curves;
  };

  /// \brief Private data for the TopicCurveHandler class.
  class TopicCurveHandlerPrivate
  {
    /// \brief A map of unique topics to topic curves.
    public: std::map<std::string, std::unique_ptr<TopicCurve>> topics;
  };
}
}
}
}

/////////////////////////////////////////////////
TopicCurve::TopicCurve(const std::string &_topic)
{
  this->topic = _topic;

  this->startTime = ignition::common::Time::SystemTime().Double();

  this->node.Subscribe(this->topic, &TopicCurve::OnTopicData, this);
}

/////////////////////////////////////////////////
TopicCurve::~TopicCurve()
{
}

/////////////////////////////////////////////////
bool TopicCurve::AddCurve(const std::string &_name, CurveWeakPtr _curve)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  auto c = _curve.lock();
  if (!c)
    return false;

  common::URI topicURI(_name);
  if (!topicURI.Valid())
  {
    ignwarn << "topicURI '" << topicURI.Str() << "' is invalid" << std::endl;
    return false;
  }

  common::URIQuery topicQuery = topicURI.Query();
  std::string topicQueryStr = topicQuery.Str();

  auto it = this->curves.find(topicQueryStr);
  if (it == this->curves.end())
  {
    // create entry in map
    CurveVariableSet curveSet;
    curveSet.insert(_curve);
    this->curves[topicQueryStr] = curveSet;
  }
  else
  {
    auto cIt = it->second.find(_curve);
    if (cIt == it->second.end())
    {
      it->second.insert(_curve);
    }
  }
  return true;
}

/////////////////////////////////////////////////
bool TopicCurve::RemoveCurve(CurveWeakPtr _curve)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  auto c = _curve.lock();
  if (!c)
    return false;

  for (auto it = this->curves.begin(); it != this->curves.end(); ++it)
  {
    auto cIt = it->second.find(_curve);
    if (cIt != it->second.end())
    {
      it->second.erase(cIt);
      if (it->second.empty())
      {
        this->curves.erase(it);
      }
      return true;
    }
  }

  return false;
}

/////////////////////////////////////////////////
bool TopicCurve::HasCurve(CurveWeakPtr _curve) const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  for (const auto &it : this->curves)
  {
    if (it.second.find(_curve) != it.second.end())
      return true;
  }

  return false;
}

/////////////////////////////////////////////////
unsigned int TopicCurve::CurveCount() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  unsigned int count = 0;
  for (const auto &it : this->curves)
    count += it.second.size();
  return count;
}

/////////////////////////////////////////////////
void TopicCurve::OnTopicData(const google::protobuf::Message &_msg,
    const ignition::transport::MessageInfo &/*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (this->curves.empty())
    return;

  // stores a list of curve iterators and their new values
  std::vector<std::pair<TopicCurve::CurveVariableMapIt,
      ignition::math::Vector2d> > curvesUpdates;

  // nearest sim time - use this x value if the message is not timestamped
  // double x = TopicTime::Instance()->LastSimTime().Double();
  double x = ignition::common::Time::SystemTime().Double() - this->startTime;

  this->UpdateCurve(const_cast<google::protobuf::Message *>(&_msg), 0, x,
    curvesUpdates);

  // update curves!
  for (auto &curveUpdate : curvesUpdates)
  {
    for (auto &cIt : curveUpdate.first->second)
    {
      auto curve = cIt.lock();
      if (!curve)
        continue;

      curve->AddPoint(curveUpdate.second);
    }
  }
}

/////////////////////////////////////////////////
void TopicCurve::UpdateCurve(google::protobuf::Message *_msg,
    const int _index, const double x,
    std::vector<std::pair<TopicCurve::CurveVariableMapIt,
    ignition::math::Vector2d> > &_curvesUpdates)
{
  if (!_msg)
  {
    ignerr << "Null message" << std::endl;
    return;
  }

  auto ref = _msg->GetReflection();
  if (!ref)
  {
    ignerr << "Failed to get message reflection." << std::endl;
    return;
  }

  auto descriptor = _msg->GetDescriptor();
  if (!descriptor)
  {
    ignerr << "Failed to get message descriptor." << std::endl;
    return;
  }

  // x axis data
  double xData = x;

  for (int i = 0; i < descriptor->field_count(); ++i)
  {
    auto field = descriptor->field(i);
    if (!field)
      continue;

    std::string fieldName = field->name();

    // Check if message has timestamp and use it if it exists and is
    // a top level msg field.
    // TODO(anyone) x axis is hardcoded to be the sim time for now. Once it is
    // configurable, remove this logic for setting the x value
    if (_index == 0 &&
       (fieldName == "header" || fieldName == "stamp" || fieldName == "time") &&
        field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE &&
        !field->is_repeated())
    {
      auto valueMsg = ref->MutableMessage(_msg, field);
      if (field->message_type()->name() == "Time")
      {
        ignition::msgs::Time *msg = dynamic_cast<msgs::Time *>(valueMsg);
        if (msg)
        {
          ignition::common::Time time = convert(*msg);
          xData = time.Double();
        }
      }
      else if (field->message_type()->name() == "Header")
      {
        ignition::msgs::Header *msg = dynamic_cast<msgs::Header *>(valueMsg);
        if (msg)
        {
          auto ts = msg->mutable_stamp();
          ignition::common::Time time(ts->sec(), ts->nsec());
          xData = time.Double();
        }
      }
    }

    // loop through all the topic param name + curve pairs
    for (auto cIt = this->curves.begin(); cIt != this->curves.end(); ++cIt)
    {
      // parse param to get field at current index
      std::string topicField;

      std::string query = cIt->first;

      // tokenize query, e.g. ?p=sim_time -> [?p, sim_time]
      std::vector<std::string> queryTokens = common::split(query, "=/");

      // skip ?p
      unsigned int queryIndex = _index + 1;

      if (queryTokens.size() < 2 || queryTokens.size() <= queryIndex)
        continue;

      topicField = queryTokens[queryIndex];

      double data = 0;
      bool addData = true;
      if (topicField != fieldName)
        continue;

      // check repeated field
      // if repeated, the the next query token should indicate the index,
      // e.g. ?p=model/2/position/x
      // model is repeated and we want to get index value of 2
      bool repeated = field->is_repeated();
      int repeatedIdx = 0;
      if (repeated)
      {
        repeatedIdx = std::stoi(queryTokens[queryIndex + 1].c_str());
        int fieldSize = ref->FieldSize(*_msg, field);
        if (repeatedIdx >= fieldSize)
          continue;
      }

      switch (field->type())
      {
        case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
        {
          if (repeated)
            data = ref->GetRepeatedDouble(*_msg, field, repeatedIdx);
          else
            data = ref->GetDouble(*_msg, field);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_FLOAT:
        {
          if (repeated)
            data = ref->GetRepeatedFloat(*_msg, field, repeatedIdx);
          else
            data = ref->GetFloat(*_msg, field);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_INT64:
        {
          if (repeated)
            data = ref->GetRepeatedInt64(*_msg, field, repeatedIdx);
          else
            data = ref->GetInt64(*_msg, field);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_UINT64:
        {
          if (repeated)
            data = ref->GetRepeatedUInt64(*_msg, field, repeatedIdx);
          else
            data = ref->GetUInt64(*_msg, field);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_INT32:
        {
          if (repeated)
            data = ref->GetRepeatedInt32(*_msg, field, repeatedIdx);
          else
            data = ref->GetInt32(*_msg, field);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_UINT32:
        {
          if (repeated)
            data = ref->GetRepeatedUInt32(*_msg, field, repeatedIdx);
          else
            data = ref->GetUInt32(*_msg, field);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_BOOL:
        {
          if (repeated)
            data = ref->GetRepeatedBool(*_msg, field, repeatedIdx);
          else
            data = static_cast<int>(ref->GetBool(*_msg, field));
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_MESSAGE:
        {
          google::protobuf::Message *valueMsg = nullptr;
          if (repeated)
            valueMsg = ref->MutableRepeatedMessage(_msg, field, repeatedIdx);
          else
            valueMsg = ref->MutableMessage(_msg, field);

          if (field->message_type()->name() == "Time")
          {
            msgs::Time *msg = dynamic_cast<msgs::Time *>(valueMsg);

            if (!msg)
              continue;

             common::Time time = convert(*msg);
             data = time.Double();
          }
          else if (field->message_type()->name() == "Vector3d")
          {
            msgs::Vector3d *msg = dynamic_cast<msgs::Vector3d *>(valueMsg);

            if (!msg)
              continue;

            ignition::math::Vector3d vec = msgs::Convert(*msg);

            // parse param to get x, y, or z at leaf of query
            std::string elem = query.substr(query.size()-1);
            if (elem == "x")
            {
              data = vec.X();
            }
            else if (elem == "y")
            {
              data = vec.Y();
            }
            else if (elem == "z")
            {
              data = vec.Z();
            }
            else
              continue;
          }
          else if (field->message_type()->name() == "Quaternion")
          {
            msgs::Quaternion *msg =
                dynamic_cast<msgs::Quaternion *>(valueMsg);

            if (!msg)
              continue;

            ignition::math::Quaterniond quat = msgs::Convert(*msg);

            // parse query to get roll, pitch, or yaw at leaf of uri
            ignition::math::Vector3d rpy = quat.Euler();
            if (query.find("roll") != std::string::npos)
              data = rpy.X();
            else if (query.find("pitch") != std::string::npos)
              data = rpy.Y();
            else if (query.find("yaw") != std::string::npos)
              data = rpy.Z();
            else
              continue;
          }
          else
          {
            // increment query token index
            // if repeated field, further increment by one to skip the repeated
            // index token
            int nextIdx = _index + ((repeated) ? 2 : 1);
            this->UpdateCurve(valueMsg, nextIdx, xData, _curvesUpdates);
            addData = false;
          }
          break;
        }
        default:
        {
          continue;
        }
      // end switch
      }

      // push to tmp list and update later
      if (addData)
      {
        _curvesUpdates.push_back(
            std::make_pair(cIt, ignition::math::Vector2d(xData, data)));
      }
    }
  }
}

/////////////////////////////////////////////////
TopicCurveHandler::TopicCurveHandler()
  : dataPtr(new TopicCurveHandlerPrivate())
{
}

/////////////////////////////////////////////////
TopicCurveHandler::~TopicCurveHandler()
{
}

/////////////////////////////////////////////////
void TopicCurveHandler::AddCurve(const std::string &_name, CurveWeakPtr _curve)
{
  // append scheme to make it a valid uri so we can parse the string
  std::string uriName = "topic://" + _name;

  common::URI topicURI(uriName);
  if (!topicURI.Valid())
  {
    ignwarn << "topicURI '" << topicURI.Str().c_str() <<
               "' is invalid" << std::endl;
    return;
  }

  common::URIPath topicPath = topicURI.Path();
  // append '/' to make it a valid topic name
  std::string topicPathStr = "/" + topicPath.Str();

  auto topicCurveIt = this->dataPtr->topics.find(topicPathStr);
  if (topicCurveIt == this->dataPtr->topics.end())
  {
    std::unique_ptr<TopicCurve> topicCurve{new TopicCurve(topicPathStr)};
    bool result = topicCurve->AddCurve(uriName, _curve);
    if (result)
      this->dataPtr->topics[topicPathStr] = std::move(topicCurve);
  }
  else
  {
    topicCurveIt->second->AddCurve(uriName, _curve);
  }
}

/////////////////////////////////////////////////
void TopicCurveHandler::RemoveCurve(CurveWeakPtr _curve)
{
  for (auto it = this->dataPtr->topics.begin();
            it !=this->dataPtr->topics.end(); ++it)
  {
    if (it->second->HasCurve(_curve))
    {
      it->second->RemoveCurve(_curve);
      if (it->second->CurveCount() == 0)
      {
        this->dataPtr->topics.erase(it);
      }
      break;
    }
  }
}

/////////////////////////////////////////////////
bool TopicCurveHandler::HasCurve(CurveWeakPtr _curve) const
{
  for (const auto &it : this->dataPtr->topics)
  {
    if (it.second->HasCurve(_curve))
      return true;
  }

  return false;
}

/////////////////////////////////////////////////
unsigned int TopicCurveHandler::CurveCount() const
{
  unsigned int count = 0;
  for (const auto &it : this->dataPtr->topics)
    count += it.second->CurveCount();
  return count;
}
