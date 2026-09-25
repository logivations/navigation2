/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  Copyright (c) 2019, Intel Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NAV2_CORE__PLANNER_EXCEPTIONS_HPP_
#define NAV2_CORE__PLANNER_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include <memory>
#include <utility>

#include "nav_msgs/msg/path.hpp"

namespace nav2_core
{

class PlannerException : public std::runtime_error
{
public:
  explicit PlannerException(const std::string & description)
  : std::runtime_error(description) {}
};

class InvalidPlanner : public PlannerException
{
public:
  explicit InvalidPlanner(const std::string & description)
  : PlannerException(description) {}
};

class StartOccupied : public PlannerException
{
public:
  explicit StartOccupied(const std::string & description)
  : PlannerException(description) {}
};

class GoalOccupied : public PlannerException
{
public:
  explicit GoalOccupied(const std::string & description)
  : PlannerException(description) {}
};

class StartOutsideMapBounds : public PlannerException
{
public:
  explicit StartOutsideMapBounds(const std::string & description)
  : PlannerException(description) {}
};

class GoalOutsideMapBounds : public PlannerException
{
public:
  explicit GoalOutsideMapBounds(const std::string & description)
  : PlannerException(description) {}
};

class NoValidPathCouldBeFound : public PlannerException
{
public:
  explicit NoValidPathCouldBeFound(const std::string & description)
  : PlannerException(description) {}
};

/**
 * @brief The reachable part of the way to a goal the planner found no path to
 * (opt-in, see PlanRequestOptions::compute_partial_path)
 */
struct PartialPlan
{
  // Start -> the reachable pose closest to the goal, stopped short of the blockage
  nav_msgs::msg::Path path;
  // Distance to the goal around blocked space [m], from the start and from the end of path
  double start_cost_to_go{0.0};
  double end_cost_to_go{0.0};
};

/**
 * @brief NoValidPathCouldBeFound that carries the reachable part of the way
 */
class NoValidPathWithPartialPlan : public NoValidPathCouldBeFound
{
public:
  NoValidPathWithPartialPlan(const std::string & description, PartialPlan partial_plan)
  : NoValidPathCouldBeFound(description), partial_plan_(std::move(partial_plan)) {}

  const PartialPlan & partialPlan() const {return partial_plan_;}

private:
  PartialPlan partial_plan_;
};

class PlannerTimedOut : public PlannerException
{
public:
  explicit PlannerTimedOut(const std::string & description)
  : PlannerException(description) {}
};

class PlannerTFError : public PlannerException
{
public:
  explicit PlannerTFError(const std::string & description)
  : PlannerException(description) {}
};

class NoViapointsGiven : public PlannerException
{
public:
  explicit NoViapointsGiven(const std::string & description)
  : PlannerException(description) {}
};

class PlannerCancelled : public PlannerException
{
public:
  explicit PlannerCancelled(const std::string & description)
  : PlannerException(description) {}
};

}  // namespace nav2_core

#endif  // NAV2_CORE__PLANNER_EXCEPTIONS_HPP_
