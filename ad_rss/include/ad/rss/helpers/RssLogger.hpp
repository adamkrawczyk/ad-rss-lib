// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2022 Intel Corporation
//
// SPDX-License-Identifier: LGPL-2.1-only
//
// ----------------- END LICENSE BLOCK -----------------------------------
/**
 * @file
 */

#pragma once

#include <memory>
#include "ad/rss/situation/SituationSnapshot.hpp"
#include "ad/rss/state/ProperResponse.hpp"
#include "ad/rss/state/RssStateSnapshot.hpp"
#include "ad/rss/world/WorldModel.hpp"

/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace rss
 */
namespace rss {

/*!
 * @brief namespace helpers
 */
namespace helpers {

/*!
 * @brief struct SafeDistanceComponents
 *
 * Struct containing the components of the safe distance calculation.
 */
struct SafeDistanceComponent{
  long unsigned int id = 0; // ID referring to the ObjectID
  bool is_safe = false ;  // True if the object is safe at any direction
  double safe_distance = -1.0; // The safe distance for the object
  double current_distance = -1.0; // The current distance for the object
  std::vector<std::pair<std::string, double>> distance_component; // The description and the distance component

  void addDistanceComponent(std::string description, ad::physics::Distance distance);
  void addDistanceComponent(std::string description, const ad::physics::Speed distance);

  // Template second argument
  template <typename T>
  void addDistanceComponent(std::string description, T distance)
  {
    distance_component.push_back(std::make_pair(description, static_cast<double>(distance)));
  }
};

class RssLogger
{
public:

  /**
   * @brief SafeDistanceComponents
   */
  std::vector<SafeDistanceComponent> safe_distance_components;

  /**
   * @brief Add empty safe distance component
   */
  void createSafeDistanceComponent();

  void addDistanceComponent(std::string description, ad::physics::Distance distance);

  /**
   * @brief getMessage
   * @return return all the messages in string format.
   */
  std::string getMessage();

  /**
   * @brief appendMessage, add only to the end of the message
   * @param [in] msg - message to be appended
   */
  void appendMessage(const std::string &msg);
  /**
   * @brief appendMessage, add only to the end of the message
   * @param [in] msg - message to be appended
   */
  void appendMessage(const std::ostream &msg);

  template <typename First, typename... Rest> void appendMessage(First arg, const Rest &...rest)
  {
    std::stringstream ss;
    ss << arg;
    log_message_ = log_message_ + "\n" + ss.str();
    appendMessage(rest...);
  }

  /**
   * @brief logError, add message to the log_message and console output with error level
   * @param [in] msg - message to be logged
   */
  template <typename... Args> void logError(Args... args)
  {
    appendMessage(args...);
    spdlog::error(log_message_);
  }

  /**
   * @brief logError, add message to the log_message and console output with warning level
   * @param [in] msg - message to be logged
   */
  template <typename... Args> void logWarn(Args... args)
  {
    appendMessage(args...);
    spdlog::warn(log_message_);
  }

  /**
   * @brief logError, add message to the log_message and console output with info level
   * @param [in] msg - message to be logged
   */
  template <typename... Args> void logInfo(Args... args)
  {
    appendMessage(args...);
    spdlog::info(log_message_);
  }

  /**
   * @brief logError, add message to the log_message and console output with debug level
   * @param [in] msg - message to be logged
   */
  template <typename... Args> void logDebug(Args... args)
  {
    appendMessage(args...);
    spdlog::debug(log_message_);
  }

  /**
   * @brief logError, add message to the log_message and console output with trace level
   * @param [in] msg - message to be logged
   */
  template <typename... Args> void logTrace(Args... args)
  {
    appendMessage(args...);
    spdlog::trace(log_message_);
  }

  /**
   * @brief logError, add message to the log_message and console output with critical level
   * @param [in] msg - message to be logged
   */
  template <typename... Args> void logCritical(Args... args)
  {
    appendMessage(args...);
    spdlog::critical(log_message_);
  }

private:
  std::string log_message_;

  void appendMessage()
  {
    log_message_ = log_message_ + "\n";
  }
};

} // namespace helpers
} // namespace rss
} // namespace ad
