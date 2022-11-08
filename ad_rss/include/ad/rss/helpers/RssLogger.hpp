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

class RssLogger
{
public:
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
