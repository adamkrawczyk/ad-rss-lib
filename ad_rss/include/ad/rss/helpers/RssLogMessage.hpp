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

class RssLogMessage
{
public:
  std::string getLogMessage();
  void logMessage(const std::string &msg);
  void logMessage(const std::ostream &msg);

  template <typename First, typename... Strings> void logMessage(First arg, const Strings &... rest)
  {
    std::stringstream ss;
    ss << arg;
    log_message_ = log_message_ + "\n" + ss.str();
    logMessage(rest...);
  }

private:
  std::string log_message_;

  void logMessage()
  {
    log_message_ = log_message_ + "\n";
  }
};

} // namespace helpers
} // namespace rss
} // namespace ad
