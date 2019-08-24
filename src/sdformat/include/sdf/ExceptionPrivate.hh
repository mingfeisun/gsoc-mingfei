/*
 * Copyright 2015 Open Source Robotics Foundation
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

#ifndef SDF_EXCEPTION_PRIVATE_HH_
#define SDF_EXCEPTION_PRIVATE_HH_

#include <cstdint>
#include <string>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  /// \internal
  /// \brief Private data for Exception
  class ExceptionPrivate
  {
    /// \brief The error function
    public: std::string file;

    /// \brief Line the error occured on
    public: std::int64_t line;

    /// \brief The error string
    public: std::string str;
  };
  }
}
#endif

