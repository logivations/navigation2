// Copyright (c) 2022. Joshua Wallace
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef NAV2_CORE__CONTROLLER_EXCEPTIONS_HPP_
#define NAV2_CORE__CONTROLLER_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

namespace nav2_core
{

class ControllerException : public std::runtime_error
{
public:
  explicit ControllerException(const std::string & description)
  : std::runtime_error(description) {}
};

class InvalidController : public ControllerException
{
public:
  explicit InvalidController(const std::string & description)
  : ControllerException(description) {}
};

class ControllerTFError : public ControllerException
{
public:
  explicit ControllerTFError(const std::string & description)
  : ControllerException(description) {}
};

class FailedToMakeProgress : public ControllerException
{
public:
  explicit FailedToMakeProgress(const std::string & description)
  : ControllerException(description) {}
};

class PatienceExceeded : public ControllerException
{
public:
  explicit PatienceExceeded(const std::string & description)
  : ControllerException(description) {}
};

class InvalidPath : public ControllerException
{
public:
  explicit InvalidPath(const std::string & description)
  : ControllerException(description) {}
};

/**
 * @class EmptyPath
 * @brief The path handed to the controller carries no poses at all.
 *
 * Split out of InvalidPath because the two need opposite logging: an empty path is
 * never produced by the controller itself, it means the upstream path producer
 * (planner/smoother) already failed and logged the real cause, so the controller
 * must not report the same event a second time. Every other InvalidPath is a
 * diagnosis the controller makes on its own and that nobody upstream has logged.
 * Derived from InvalidPath so existing handlers and error codes keep working.
 */
class EmptyPath : public InvalidPath
{
public:
  explicit EmptyPath(const std::string & description)
  : InvalidPath(description) {}
};

class NoValidControl : public ControllerException
{
public:
  explicit NoValidControl(const std::string & description)
  : ControllerException(description) {}
};

class ControllerTimedOut : public ControllerException
{
public:
  explicit ControllerTimedOut(const std::string & description)
  : ControllerException(description) {}
};

}  // namespace nav2_core

#endif  // NAV2_CORE__CONTROLLER_EXCEPTIONS_HPP_
