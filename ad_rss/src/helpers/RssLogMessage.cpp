// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-202w Intel Corporation
//
// SPDX-License-Identifier: LGPL-2.1-only
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/rss/helpers/RssLogMessage.hpp"

namespace ad {
namespace rss {
namespace helpers {

void  RssLogMessage::logMessage(const std::string &msg){
    log_message_ = log_message_ + "\n" + msg;
}

void RssLogMessage::logMessage(const std::ostream &msg){
    std::stringstream ss;
    ss << msg.rdbuf();
    log_message_ = log_message_ + "\n" + ss.str();

}

std::string RssLogMessage::getLogMessage(){
    return log_message_;
}

} // namespace helpers
} // namespace rss
} // namespace ad