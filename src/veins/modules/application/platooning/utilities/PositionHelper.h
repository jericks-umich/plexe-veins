//
// Copyright (c) 2012-2016 Michele Segata <segata@ccs-labs.org>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#ifndef POSITIONHELPER_H_
#define POSITIONHELPER_H_

#include "veins/modules/application/platooning/utilities/BasePositionHelper.h"

/**
 * Defines position utility functions for platoons organized in the following
 * way:
 * <-------- Travel direction --------
 * Lane 2  2 5 8 11     14 17 20 23
 * Lane 1  1 4 7 10     13 16 19 22
 * Lane 0  0 3 6  9     12 15 18 21
 *
 * In this particular example, we have 3 lanes, a platoon size of 4 vehicles
 * and 24 cars in total
 */
class PositionHelper : public BasePositionHelper {

public:
  virtual void initialize(int stage);
  virtual void finish();

  /**
   * Returns the position of this vehicle within the platoon
   */
  virtual int getPosition();

  /**
   * Returns the id of the i-th vehicle of the own platoon
   */
  virtual int getMemberId(int position);

  /**
   * Returns the position of a vehicle of the own platoon
   */
  virtual int getMemberPosition(int vehicleId);

  /**
   * Returns the id of the leader of the own platoon
   */
  virtual int getLeaderId();

  /**
   * Returns whether this vehicle is the leader of the platoon
   */
  virtual bool isLeader();

  /**
   * Returns the id of the vehicle in front of me
   */
  virtual int getFrontId();

  /**
   * Returns the id of the platoon
   */
  virtual int getPlatoonId();

  /**
   * Returns the lane the platoon is traveling on
   */
  virtual int getPlatoonLane();

  /**
   * Returns whether a vehicle is part of my platoon
   */
  virtual bool isInSamePlatoon(int vehicleId);

  /**
   * Returns size of platoon and puts array of vehicle positions for this
   * platoon in order. Positions are converted to uint8_t's for space reasons
   * as it is unlikely there will be more than 256 vehicles in one platoon.
   */
  virtual int getPlatoonOrder(uint8_t *order, int buf_size);
  // Like getPlatoonOrder, but populates order with a list starting with this
  // vehicle and containing the vehicles behind it
  virtual int getPlatoonOrderBehindMe(uint8_t *order, int buf_size);

public:
  static int getIdFromExternalId(std::string externalId);
  // static int getPlatoonColumn(int vehicleId, int nLanes, int platoonSize);
  static int getPlatoonColumn(int vehicleId, int nLanes);
  // static int getPlatoonLeader(int vehicleId, int nLanes, int platoonSize);
  static int getPlatoonLeader(int vehicleId, int nLanes);
  static int getPlatoonLane(int vehicleId, int nLanes);
  // static int getPlatoonNumber(int vehicleId, int nLanes, int platoonSize);
  static int getPlatoonNumber(int vehicleId, int nLanes);
  // static int getFrontVehicle(int vehicleId, int nLanes, int platoonSize);
  static int getFrontVehicle(int vehicleId, int nLanes);
  static int getBehindVehicle(int vehicleId, int nLanes, int platoonSize);
  // static bool isLeader(int vehicleId, int nLanes, int platoonSize);
  static bool isLeader(int vehicleId, int nLanes);
  // static int getPositionInPlatoon(int vehicleId, int nLanes, int
  // platoonSize);
  static int getPositionInPlatoon(int vehicleId, int nLanes);
  // static bool isFrontVehicle(int vehicleId, int myId, int nLanes,
  //                           int platoonSize);
  static bool isFrontVehicle(int vehicleId, int myId, int nLanes);
  // static bool isInSamePlatoon(int vehicleId, int myId, int nLanes,
  //                            int platoonSize);
  static bool isInSamePlatoon(int vehicleId, int myId, int nLanes);

public:
  PositionHelper() : BasePositionHelper() {}
};

#endif
