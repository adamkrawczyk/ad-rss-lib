// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-202w Intel Corporation
//
// SPDX-License-Identifier: LGPL-2.1-only
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/rss/helpers/RssLogger.hpp"

namespace ad {
namespace rss {
namespace helpers {

void SafeDistanceComponent::addDistanceComponent(std::string description, ad::physics::Distance distance)
{
  distance_component.push_back(std::make_pair(description, static_cast<double>(distance)));
}

void SafeDistanceComponent::addDistanceComponent(std::string description, const ad::physics::Speed distance)
{
  distance_component.push_back(std::make_pair(description, static_cast<double>(distance)));
}

void RssLogger::createSafeDistanceComponent()
{
  SafeDistanceComponent component;
  safe_distance_components.push_back(component);
}

void RssLogger::addDistanceComponent(std::string description, ad::physics::Distance distance)
{
  safe_distance_components.back().addDistanceComponent(description, distance);
}

void RssLogger::appendMessage(const std::string &msg)
{
  log_message_ = log_message_ + "\n" + msg;
}

void RssLogger::appendMessage(const std::ostream &msg)
{
  std::stringstream ss;
  ss << msg.rdbuf();
  log_message_ = log_message_ + "\n" + ss.str();
}

std::string RssLogger::getMessage()
{
  return log_message_;
}

} // namespace helpers
} // namespace rss
} // namespace ad