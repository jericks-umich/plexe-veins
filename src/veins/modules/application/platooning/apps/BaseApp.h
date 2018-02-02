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

#ifndef BASEAPP_H_
#define BASEAPP_H_

#include "veins/base/modules/BaseApplLayer.h"

#include "veins/modules/application/platooning/UnicastProtocol.h"
#include "veins/modules/application/platooning/messages/PlatooningBeacon_m.h"

#include "veins/modules/mobility/traci/TraCIMobility.h"

#include "veins/modules/application/platooning/CC_Const.h"

#include "veins/modules/application/platooning/utilities/BasePositionHelper.h"

class BaseProtocol;

class BaseApp : public BaseApplLayer {

public:
  virtual void initialize(int stage);
  virtual void finish();

protected:
  virtual void onBeacon(WaveShortMessage *wsm);
  virtual void onData(WaveShortMessage *wsm);

protected:
  // id of this vehicle
  int myId;

  Veins::TraCIMobility *mobility;
  Veins::TraCICommandInterface *traci;
  Veins::TraCICommandInterface::Vehicle *traciVehicle;

  // determines position and role of each vehicle
  BasePositionHelper *positionHelper;

  // lower layer protocol
  BaseProtocol *protocol;

  // time at which simulation should stop
  SimTime simulationDuration;
  // determine whether there has been a vehicle collision in the simulation.
  // shared by all
  static bool crashHappened;
  // determine whether simulation correctly terminated
  static bool simulationCompleted;

  /**
   * Log data about vehicle
   */
  virtual void logVehicleData(bool crashed = false);

  // output vectors for mobility stats
  // id of the vehicle
  cOutVector nodeIdOut;
  // distance and relative speed
  cOutVector distanceOut, relSpeedOut;
  // speed and position
  cOutVector speedOut, posxOut, posyOut;
  // real acceleration and controller acceleration
  cOutVector accelerationOut, controllerAccelerationOut;

  // messages for scheduleAt
  cMessage *recordData;
  cMessage *contractChain;

public:
  BaseApp() { recordData = 0; }

  /**
   * Sends a unicast message
   *
   * @param msg message to be encapsulated into the unicast message
   * @param destination id of the destination
   */
  void sendUnicast(cPacket *msg, int destination);

  /**
   * Stops the simulation. Can be invoked by other classes
   */
  void stopSimulation();

protected:
  virtual void handleLowerMsg(cMessage *msg);
  virtual void handleSelfMsg(cMessage *msg);
  virtual void handleLowerControl(cMessage *msg);
  virtual void startNewContractChain();
};

#endif /* BASEAPP_H_ */
