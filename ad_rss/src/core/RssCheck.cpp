// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2021 Intel Corporation
//
// SPDX-License-Identifier: LGPL-2.1-only
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/rss/core/RssCheck.hpp"
#include "ad/rss/core/RssResponseResolving.hpp"
#include "ad/rss/core/RssSituationChecking.hpp"
#include "ad/rss/core/RssSituationExtraction.hpp"
#include "ad/rss/helpers/RssLogMessage.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

namespace ad {
namespace rss {
namespace core {

RssCheck::RssCheck()
{
  try
  {
    mResponseResolving = std::unique_ptr<RssResponseResolving>(new RssResponseResolving());
    mSituationChecking = std::unique_ptr<RssSituationChecking>(new RssSituationChecking());
    mSituationExtraction = std::unique_ptr<RssSituationExtraction>(new RssSituationExtraction());
    mLogMessage = std::unique_ptr<helpers::RssLogMessage>(new helpers::RssLogMessage());
  }
  catch (...)
  {
    spdlog::critical("RssCheck object initialization failed");
    mResponseResolving = nullptr;
    mSituationChecking = nullptr;
    mSituationExtraction = nullptr;
    mLogMessage = nullptr;
  }
}

RssCheck::~RssCheck()
{
}

bool RssCheck::calculateProperResponse(world::WorldModel const &worldModel,
                                       situation::SituationSnapshot &situationSnapshot,
                                       state::RssStateSnapshot &rssStateSnapshot,
                                       state::ProperResponse &properResponse,
                                       std::string &issueDescription)
{
  issueDescription.append("test message; ");
  bool result = false;
  // global try catch block to ensure this library call doesn't throw an exception
  try
  {
    if (!static_cast<bool>(mResponseResolving) || !static_cast<bool>(mSituationChecking)
        || !static_cast<bool>(mSituationExtraction))
    {
      const auto log = "RssCheck::calculateProperResponse>> object not properly initialized";
      mLogMessage->logMessage(log);
      spdlog::critical(log);
      return false;
    }

    result = mSituationExtraction->extractSituations(worldModel, situationSnapshot, *mLogMessage);
    issueDescription.append(mLogMessage->getLogMessage());

    if (result)
    {
      result = mSituationChecking->checkSituations(situationSnapshot, rssStateSnapshot, issueDescription);
    }

    if (result)
    {
      result = mResponseResolving->provideProperResponse(rssStateSnapshot, properResponse, issueDescription);
    }
  }
  // LCOV_EXCL_START: unreachable code, keep to be on the safe side
  catch (...)
  {
    // auto log = ;
    // mLogMessage->logMessage(log);
    spdlog::critical("RssCheck::calculateProperResponse>> exception caught");
    result = false;
  }
  // LCOV_EXCL_STOP: unreachable code, keep to be on the safe side
  if(!result && issueDescription.empty())
  {
    issueDescription = "RssCheck::calculateProperResponse>> Not described issue occurred; ";
  }
  return result;
}

bool RssCheck::calculateProperResponse(world::WorldModel const &worldModel,
                                       situation::SituationSnapshot &situationSnapshot,
                                       state::RssStateSnapshot &rssStateSnapshot,
                                       state::ProperResponse &properResponse)
{
  std::string issueDescription;
return calculateProperResponse(worldModel, situationSnapshot, rssStateSnapshot, properResponse, issueDescription);
}

bool RssCheck::calculateProperResponse(world::WorldModel const &worldModel, state::ProperResponse &properResponse)
{
  situation::SituationSnapshot situationSnapshot;
  state::RssStateSnapshot rssStateSnapshot;

  return calculateProperResponse(worldModel, situationSnapshot, rssStateSnapshot, properResponse);
}

} // namespace core
} // namespace rss
} // namespace ad
