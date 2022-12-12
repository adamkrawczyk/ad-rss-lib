// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2021 Intel Corporation
//
// SPDX-License-Identifier: LGPL-2.1-only
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/rss/situation/RssFormulas.hpp"
#include <algorithm>
#include "ad/rss/helpers/RssLogger.hpp"
#include "ad/rss/situation/Physics.hpp"
#include "ad/rss/situation/VehicleStateValidInputRange.hpp"

namespace ad {

namespace rss {
namespace situation {

// make the code more readable
using physics::Acceleration;
using physics::Distance;
using physics::Duration;
using physics::Speed;
using situation::calculateStoppingDistance;

inline bool vehicleStateWithinValidInputRange(VehicleState const &vehicleState)
{
  if (!withinValidInputRange(vehicleState))
  {
    return false;
  }
  if (vehicleState.velocity.speedLon.minimum < Speed(0.))
  {
    return false;
  }
  return true;
}

bool calculateLongitudinalDistanceOffsetAfterStatedBrakingPattern(Speed const &currentSpeed,
                                                                  Speed const &maxSpeedOnAcceleration,
                                                                  Duration const &responseTime,
                                                                  Acceleration const &acceleration,
                                                                  Acceleration const &deceleration,
                                                                  Distance &distanceOffset)
{
  Speed resultingSpeed = Speed(0.);
  Distance distanceOffsetAfterResponseTime = Distance(0.);
  bool result = calculateAcceleratedLimitedMovement(
    currentSpeed, maxSpeedOnAcceleration, acceleration, responseTime, resultingSpeed, distanceOffsetAfterResponseTime);

  Distance distanceToStop = Distance(0.);
  if (std::signbit(static_cast<double>(resultingSpeed)) != std::signbit(static_cast<double>(deceleration)))
  {
    // if speed after stated braking pattern has the same direction as the acceleration
    // further braking to full stop in that moving direction has to be added
    result = result && calculateStoppingDistance(resultingSpeed, deceleration, distanceToStop);
  }

  if (result)
  {
    distanceOffset = distanceOffsetAfterResponseTime + distanceToStop;
  }

  return result;
}

bool calculateLateralDistanceOffsetAfterStatedBrakingPattern(Speed const &currentSpeed,
                                                             Duration const &responseTime,
                                                             Acceleration const &acceleration,
                                                             Acceleration const &deceleration,
                                                             Distance &distanceOffset,
                                                             std::shared_ptr<helpers::RssLogger> &mRssLogger)
{
  Speed resultingSpeed = Speed(0.);
  Distance distanceOffsetAfterResponseTime = Distance(0.);

  bool result = calculateSpeedInAcceleratedMovement(currentSpeed, acceleration, responseTime, resultingSpeed);
  result = result
    && calculateDistanceOffsetInAcceleratedMovement(
             currentSpeed, acceleration, responseTime, distanceOffsetAfterResponseTime);

  Distance distanceToStop = Distance(0.);
  if (std::signbit(static_cast<double>(resultingSpeed)) != std::signbit(static_cast<double>(deceleration)))
  {
    // if speed after stated braking pattern has the same direction as the acceleration
    // further braking to full stop in that moving direction has to be added
    result = result && calculateStoppingDistance(resultingSpeed, deceleration, distanceToStop);
    mRssLogger->logInfo("calculateLateralDistanceOffsetAfterStatedBrakingPattern: resultingSpeed: "
                        + std::to_string(resultingSpeed) + " deceleration: " + std::to_string(deceleration)
                        + " distanceToStop: " + std::to_string(distanceToStop));
  }

  if (result)
  {
    distanceOffset = distanceOffsetAfterResponseTime + distanceToStop;
    mRssLogger->logInfo("calculateLateralDistanceOffsetAfterStatedBrakingPattern: distanceOffset: "
                        + std::to_string(distanceOffset));
  }

  return result;
}

bool calculateSafeLongitudinalDistanceSameDirection(VehicleState const &leadingVehicle,
                                                    VehicleState const &followingVehicle,
                                                    Distance &safeDistance)
{
  if (!vehicleStateWithinValidInputRange(leadingVehicle) || !vehicleStateWithinValidInputRange(followingVehicle))
  {
    return false;
  }

  Distance distanceStatedBraking = Distance(0.);

  bool result = calculateLongitudinalDistanceOffsetAfterStatedBrakingPattern( // LCOV_EXCL_LINE: wrong detection
    followingVehicle.velocity.speedLon.maximum,
    followingVehicle.dynamics.maxSpeedOnAcceleration,
    followingVehicle.dynamics.responseTime,
    followingVehicle.dynamics.alphaLon.accelMax,
    followingVehicle.dynamics.alphaLon.brakeMin,
    distanceStatedBraking);
  Distance distanceMaxBrake = Distance(0.);
  result = result
    && calculateStoppingDistance(
             leadingVehicle.velocity.speedLon.minimum, leadingVehicle.dynamics.alphaLon.brakeMax, distanceMaxBrake);

  if (result)
  {
    safeDistance = distanceStatedBraking - distanceMaxBrake;
    safeDistance = std::max(safeDistance, Distance(0.));
  }

  return result;
}

bool checkSafeLongitudinalDistanceSameDirection(VehicleState const &leadingVehicle,
                                                VehicleState const &followingVehicle,
                                                Distance const &vehicleDistance,
                                                Distance &safeDistance,
                                                bool &isDistanceSafe,
                                                std::shared_ptr<helpers::RssLogger> &mRssLogger)
{
  if (vehicleDistance < Distance(0.))
  {
    mRssLogger->logInfo("checkSafeLongitudinalDistanceSameDirection: vehicleDistance < 0, vehicleDistance: "
                        + std::to_string(vehicleDistance));
    return false;
  }

  isDistanceSafe = false;
  safeDistance = std::numeric_limits<physics::Distance>::max();

  bool const result = calculateSafeLongitudinalDistanceSameDirection(leadingVehicle, followingVehicle, safeDistance);

  if (vehicleDistance > safeDistance)
  {
    isDistanceSafe = true;
  }
  else
  {
    mRssLogger->logInfo("checkSafeLongitudinalDistanceSameDirection: vehicleDistance < safeDistance, vehicleDistance: "
                        + std::to_string(vehicleDistance) + ", safeDistance: " + std::to_string(safeDistance));
  }
  return result;
}

bool calculateSafeLongitudinalDistanceOppositeDirection(VehicleState const &correctVehicle,
                                                        VehicleState const &oppositeVehicle,
                                                        Distance &safeDistance)
{
  if (!vehicleStateWithinValidInputRange(correctVehicle) || !vehicleStateWithinValidInputRange(oppositeVehicle))
  {
    return false;
  }

  Distance distanceStatedBrakingCorrect = Distance(0.);

  bool result = calculateLongitudinalDistanceOffsetAfterStatedBrakingPattern( // LCOV_EXCL_LINE: wrong detection
    correctVehicle.velocity.speedLon.maximum,
    correctVehicle.dynamics.maxSpeedOnAcceleration,
    correctVehicle.dynamics.responseTime,
    correctVehicle.dynamics.alphaLon.accelMax,
    correctVehicle.dynamics.alphaLon.brakeMinCorrect,
    distanceStatedBrakingCorrect);

  Distance distanceStatedBrakingOpposite = Distance(0.);

  if (result)
  {
    result = calculateLongitudinalDistanceOffsetAfterStatedBrakingPattern( // LCOV_EXCL_LINE: wrong detection
      oppositeVehicle.velocity.speedLon.maximum,
      oppositeVehicle.dynamics.maxSpeedOnAcceleration,
      oppositeVehicle.dynamics.responseTime,
      oppositeVehicle.dynamics.alphaLon.accelMax,
      oppositeVehicle.dynamics.alphaLon.brakeMin,
      distanceStatedBrakingOpposite);
  }

  if (result)
  {
    safeDistance = distanceStatedBrakingCorrect + distanceStatedBrakingOpposite;
  }

  return result;
}

bool checkSafeLongitudinalDistanceOppositeDirection(VehicleState const &correctVehicle,
                                                    VehicleState const &oppositeVehicle,
                                                    Distance const &vehicleDistance,
                                                    Distance &safeDistance,
                                                    bool &isDistanceSafe)
{
  if (vehicleDistance < Distance(0.))
  {
    return false;
  }

  isDistanceSafe = false;
  safeDistance = std::numeric_limits<physics::Distance>::max();
  bool const result = calculateSafeLongitudinalDistanceOppositeDirection(correctVehicle, oppositeVehicle, safeDistance);

  if (vehicleDistance > safeDistance)
  {
    isDistanceSafe = true;
  }
  return result;
}

bool checkStopInFrontIntersection(VehicleState const &vehicle,
                                  Distance &safeDistance,
                                  bool &isDistanceSafe,
                                  std::shared_ptr<helpers::RssLogger> &mRssLogger)
{
  if (!vehicleStateWithinValidInputRange(vehicle))
  {
    return false;
  }

  isDistanceSafe = false;

  safeDistance = Distance(0.);
  bool result = calculateLongitudinalDistanceOffsetAfterStatedBrakingPattern( // LCOV_EXCL_LINE: wrong detection
    vehicle.velocity.speedLon.maximum,
    vehicle.dynamics.maxSpeedOnAcceleration,
    vehicle.dynamics.responseTime,
    vehicle.dynamics.alphaLon.accelMax,
    vehicle.dynamics.alphaLon.brakeMin,
    safeDistance);

  if (safeDistance < vehicle.distanceToEnterIntersection)
  {
    isDistanceSafe = true;
  }
  else
  {
    mRssLogger->logInfo(
      "checkStopInFrontIntersection: safeDistance < vehicle.distanceToEnterIntersection, safeDistance: "
      + std::to_string(safeDistance)
      + ", vehicle.distanceToEnterIntersection: " + std::to_string(vehicle.distanceToEnterIntersection));
  }

  return result;
}

bool calculateSafeLateralDistance(VehicleState const &leftVehicle,
                                  VehicleState const &rightVehicle,
                                  Distance &safeDistance,
                                  std::shared_ptr<helpers::RssLogger> &mRssLogger)
{
  if (!vehicleStateWithinValidInputRange(leftVehicle) || !vehicleStateWithinValidInputRange(rightVehicle))
  {
    return false;
  }

  bool result = false;
  Distance distanceOffsetStatedBrakingLeft = Distance(0.);
  Distance distanceOffsetStatedBrakingRight = Distance(0.);

  result = calculateLateralDistanceOffsetAfterStatedBrakingPattern( // LCOV_EXCL_LINE: wrong detection
    leftVehicle.velocity.speedLat.maximum,
    leftVehicle.dynamics.responseTime,
    leftVehicle.dynamics.alphaLat.accelMax,
    leftVehicle.dynamics.alphaLat.brakeMin,
    distanceOffsetStatedBrakingLeft,
    mRssLogger);

  result = result
    && calculateLateralDistanceOffsetAfterStatedBrakingPattern( // LCOV_EXCL_LINE: wrong detection
             rightVehicle.velocity.speedLat.minimum,
             rightVehicle.dynamics.responseTime,
             -rightVehicle.dynamics.alphaLat.accelMax,
             -rightVehicle.dynamics.alphaLat.brakeMin,
             distanceOffsetStatedBrakingRight,
             mRssLogger);

  if (result)
  {
    // safe distance is the difference of both distances
    safeDistance = distanceOffsetStatedBrakingLeft - distanceOffsetStatedBrakingRight;
    //  plus the lateral fluctuation margin: here we use the 0.5*my of both
    safeDistance
      += 0.5 * (leftVehicle.dynamics.lateralFluctuationMargin + rightVehicle.dynamics.lateralFluctuationMargin);
    safeDistance = std::max(safeDistance, Distance(0.));
    mRssLogger->logInfo("calculateSafeLateralDistance: safeDistance: " + std::to_string(safeDistance));
  }
  return result;
}

bool checkSafeLateralDistance(VehicleState const &leftVehicle,
                              VehicleState const &rightVehicle,
                              Distance const &vehicleDistance,
                              Distance &safeDistance,
                              bool &isDistanceSafe,
                              std::shared_ptr<helpers::RssLogger> &mRssLogger)
{
  if (vehicleDistance < Distance(0.))
  {
    mRssLogger->logInfo("checkSafeLateralDistance: vehicleDistance < 0, vehicleDistance: "
                        + std::to_string(vehicleDistance));
    return false;
  }

  isDistanceSafe = false;
  safeDistance = std::numeric_limits<physics::Distance>::max();
  bool const result = calculateSafeLateralDistance(leftVehicle, rightVehicle, safeDistance, mRssLogger);

  if (vehicleDistance > safeDistance)
  {
    isDistanceSafe = true;
  }
  else
  {
    mRssLogger->logInfo("checkSafeLateralDistance: vehicleDistance < safeDistance, vehicleDistance: "
                        + std::to_string(vehicleDistance) + ", safeDistance: " + std::to_string(safeDistance));
  }

  return result;
}

} // namespace situation
} // namespace rss
} // namespace ad
