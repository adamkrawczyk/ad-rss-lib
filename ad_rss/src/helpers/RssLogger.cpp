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