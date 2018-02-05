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

#include "veins/modules/application/platooning/utilities/PositionHelper.h"

Define_Module(PositionHelper);

void PositionHelper::initialize(int stage) {

  BasePositionHelper::initialize(stage);

  if (stage == 0) {
    nCars = par("nCars").longValue();
    myId = getIdFromExternalId(getExternalId());
    // leaderId = getPlatoonLeader(myId, nLanes, platoonSize);
    leaderId = getPlatoonLeader(myId, nLanes);
    leader = myId == leaderId;
    // frontId = getFrontVehicle(myId, nLanes, platoonSize);
    frontId = getFrontVehicle(myId, nLanes);
    // position = getPositionInPlatoon(myId, nLanes, platoonSize);
    position = getPositionInPlatoon(myId, nLanes);
    // platoonId = getPlatoonNumber(myId, nLanes, platoonSize);
    platoonId = getPlatoonNumber(myId, nLanes);
    platoonLane = getPlatoonLane(myId, nLanes);
  }
}

// Note from Jeremy Erickson:
// I substantially reworked this class, based on inferring the role of lanes and
// columns. I think much of the previous arithmetic was wrong.

//         |<------VehicleId------>|
// --------+--+--+--+--+--+--+--+--+-----------------------
//  Lane 0 |28|24|20|16|12| 8| 4| 0| -> direction of travel
// --------+--+--+--+--+--+--+--+--+-----------------------
//  Lane 1 |29|25|21|17|13| 9| 5| 1| -> direction of travel
// --------+--+--+--+--+--+--+--+--+-----------------------
//  Lane 2 |30|26|22|18|14|10| 6| 2| -> direction of travel
// --------+--+--+--+--+--+--+--+--+-----------------------
//  Lane 3 |31|27|23|19|15|11| 7| 3| -> direction of travel
// --------+--+--+--+--+--+--+--+--+-----------------------
//         |C |C |C |C |C |C |C |C |
//         |o |o |o |o |o |o |o |o |
//         |l |l |l |l |l |l |l |l |
//         | 7| 6| 5| 4| 3| 2| 1| 0|

// This is a graphical representation of what the new positioning system is

void PositionHelper::finish() { BasePositionHelper::finish(); }

int PositionHelper::getPosition() { return position; }

int PositionHelper::getMemberId(int position) {
  return leaderId + position * nLanes;
}

int PositionHelper::getMemberPosition(int vehicleId) {
  return (vehicleId - leaderId) / nLanes;
}

int PositionHelper::getLeaderId() { return leaderId; }

bool PositionHelper::isLeader() { return leader; }

int PositionHelper::getFrontId() { return frontId; }

int PositionHelper::getPlatoonId() { return platoonId; }

int PositionHelper::getPlatoonLane() { return platoonLane; }

bool PositionHelper::isInSamePlatoon(int vehicleId) {
  return platoonId == getPlatoonNumber(vehicleId, nLanes, platoonSize);
}

int PositionHelper::getPlatoonOrder(uint8_t *order, int buf_size) {
  if (buf_size < platoonSize) {
    return -1; // can't store platoon order in this buffer
  }
  int behind_vehicle = leaderId;
  for (int pos = 0; behind_vehicle != -1 && pos < platoonSize; pos++) {
    order[pos] = (uint8_t)behind_vehicle;
    behind_vehicle = getBehindVehicle(behind_vehicle);
  }
  return pos;
}

int PositionHelper::getPlatoonOrderBehindMePlusLeader(uint8_t *order,
                                                      int buf_size) {
  if (buf_size < platoonSize) {
    return -1; // can't store platoon order in this buffer
  }
  int behind_vehicle = myId;
  for (int pos = 0; behind_vehicle != -1 && pos < platoonSize; pos++) {
    order[pos] = (uint8_t)behind_vehicle;
    behind_vehicle = getBehindVehicle(behind_vehicle);
  }
  return pos;
}

int PositionHelper::getIdFromExternalId(std::string externalId) {
  int dotIndex = externalId.find_last_of('.');
  std::string strId = externalId.substr(dotIndex + 1);
  return strtol(strId.c_str(), 0, 10);
}

// bool PositionHelper::isLeader(int vehicleId, int nLanes, int platoonSize) {
//  return (vehicleId / nLanes) % platoonSize == 0;
//}
bool PositionHelper::isLeader(int vehicleId, int nLanes) {
  return (vehicleId / nLanes) == 0;
}
// int PositionHelper::getPlatoonNumber(int vehicleId, int nLanes,
//                                     int platoonSize) {
//  return getPlatoonColumn(vehicleId, nLanes) * nLanes +
//         getPlatoonLane(vehicleId, nLanes);
//}
int PositionHelper::getPlatoonNumber(int vehicleId, int nLanes) {
  return getPlatoonLane(vehicleId, nLanes);
}
int PositionHelper::getPlatoonLane(int vehicleId, int nLanes) {
  return vehicleId % nLanes;
}
// int PositionHelper::getPlatoonColumn(int vehicleId, int nLanes,
//                                     int platoonSize) {
//   return vehicleId / (nLanes * platoonSize);
//}
int PositionHelper::getPlatoonColumn(int vehicleId, int nLanes) {
  return vehicleId / nLanes;
}

// int PositionHelper::getPlatoonLeader(int vehicleId, int nLanes,
//                                     int platoonSize) {
//  return getPlatoonColumn(vehicleId, nLanes) * nLanes * platoonSize +
//         getPlatoonLane(getPlatoonNumber(vehicleId, nLanes, platoonSize),
//                        nLanes);
//}
int PositionHelper::getPlatoonLeader(int vehicleId, int nLanes) {
  return getPlatoonLane(vehicleId, nLanes);
}
// int PositionHelper::getFrontVehicle(int vehicleId, int nLanes,
//                                    int platoonSize) {
//  if (getPlatoonLeader(vehicleId, nLanes, platoonSize) == vehicleId)
//    return -1;
//  else
//    return vehicleId - nLanes;
//}
int PositionHelper::getFrontVehicle(int vehicleId, int nLanes) {
  if (getPlatoonLeader(vehicleId, nLanes) == vehicleId) {
    return -1;
  } else {
    return vehicleId - nLanes;
  }
}
int PositionHelper::getBehindVehicle(int vehicleId, int nLanes,
                                     int platoonSize) {
  if (vehicleId + nLanes > platoonSize) {
    return -1;
  } else {
    return vehicleId - nLanes;
  }
}
// bool PositionHelper::isInSamePlatoon(int vehicleId, int myId, int nLanes,
//                                     int platoonSize) {
//  return getPlatoonNumber(vehicleId, nLanes, platoonSize) ==
//         getPlatoonNumber(myId, nLanes, platoonSize);
//}
bool PositionHelper::isInSamePlatoon(int vehicleId, int myId, int nLanes) {
  return (vehicleId % nLanes) == (myId % nLanes);
}
// bool PositionHelper::isFrontVehicle(int vehicleId, int myId, int nLanes,
//                                    int platoonSize) {
//  return getFrontVehicle(myId, nLanes, platoonSize) == vehicleId;
//}
bool PositionHelper::isFrontVehicle(int vehicleId, int myId, int nLanes) {
  return getFrontVehicle(myId, nLanes) == vehicleId;
}
// int PositionHelper::getPositionInPlatoon(int vehicleId, int nLanes,
//                                         int platoonSize) {
//  return (vehicleId - getPlatoonLeader(vehicleId, nLanes, platoonSize)) /
//         nLanes;
//}
int PositionHelper::getPositionInPlatoon(int vehicleId, int nLanes) {
  return getPlatoonColumn(vehicleId, nLanes);
}
