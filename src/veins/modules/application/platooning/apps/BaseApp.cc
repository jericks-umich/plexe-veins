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

#include "veins/modules/application/platooning/apps/BaseApp.h"

#include "veins/base/messages/MacPkt_m.h"
#include "veins/modules/mac/ieee80211p/Mac1609_4.h"
#include "veins/modules/messages/WaveShortMessage_m.h"

#include "veins/modules/application/platooning/protocols/BaseProtocol.h"

#include "commpact_types.h"

bool BaseApp::crashHappened = false;
bool BaseApp::simulationCompleted = false;

Define_Module(BaseApp);

void BaseApp::initialize(int stage) {

  BaseApplLayer::initialize(stage);

  if (stage == 0) {
    // when to stop simulation (after communications started)
    simulationDuration = SimTime(par("simulationDuration").longValue());
    // set names for output vectors
    // distance from front vehicle
    distanceOut.setName("distance");
    // relative speed w.r.t. front vehicle
    relSpeedOut.setName("relativeSpeed");
    // vehicle id
    nodeIdOut.setName("nodeId");
    // current speed
    speedOut.setName("speed");
    // vehicle position
    posxOut.setName("posx");
    posyOut.setName("posy");
    // vehicle acceleration
    accelerationOut.setName("acceleration");
    controllerAccelerationOut.setName("controllerAcceleration");
  }

  if (stage == 1) {
    mobility = Veins::TraCIMobilityAccess().get(getParentModule());
    traci = mobility->getCommandInterface();
    traciVehicle = mobility->getVehicleCommandInterface();
    positionHelper =
        FindModule<BasePositionHelper *>::findSubModule(getParentModule());
    protocol = FindModule<BaseProtocol *>::findSubModule(getParentModule());
    myId = positionHelper->getId();

    // connect application to protocol
    protocol->registerApplication(BaseProtocol::BEACON_TYPE,
                                  gate("lowerLayerIn"), gate("lowerLayerOut"));

    recordData = new cMessage("recordData");
    // init statistics collection. round to 0.1 seconds
    SimTime rounded = SimTime(floor(simTime().dbl() * 1000 + 100), SIMTIME_MS);
    scheduleAt(rounded, recordData);

    // Set up periodic contract chains.
    // The "first" contract will take effect during initialization, not set up
    // here leader should initiate a new contract chain every time it receives
    // a completed chain, or every "recoveryChainCompletionDeadline"
    // milliseconds if the previous chain is late or fails.
    // Chains are considered "late" if they do not arrive by the
    // recoveryChainCompletionDeadline and then will be considered untrusted
    // and ignored.
    if (positionHelper->isLeader()) {
      contractChain = new cMessage("contractChain");
      SimTime startContractChain =
          SimTime(floor(simTime().dbl() * 1000) +
                      par("recoveryPhaseDuration").longValue(),
                  SIMTIME_MS);
      scheduleAt(startContractChain, contractChain);
    }

    // register to also receive messages having to do with contract chains
    protocol->registerApplication(BaseProtocol::CONTRACT_TYPE,
                                  gate("lowerLayerIn"), gate("lowerLayerOut"));
  }
}

void BaseApp::finish() {
  BaseApplLayer::finish();
  if (recordData) {
    if (recordData->isScheduled()) {
      cancelEvent(recordData);
    }
    delete recordData;
    recordData = 0;
  }
  if (!crashHappened && !simulationCompleted) {
    if (traciVehicle->isCrashed()) {
      crashHappened = true;
      logVehicleData(true);
      endSimulation();
    }
  }
}

void BaseApp::handleLowerMsg(cMessage *msg) {

  UnicastMessage *unicast = dynamic_cast<UnicastMessage *>(msg);
  ASSERT2(unicast, "received a frame not of type UnicastMessage");

  cPacket *enc = unicast->decapsulate();
  ASSERT2(enc, "received a UnicastMessage with nothing inside");

  if (enc->getKind() == BaseProtocol::BEACON_TYPE) {

    PlatooningBeacon *epkt = dynamic_cast<PlatooningBeacon *>(enc);
    ASSERT2(epkt,
            "received UnicastMessage does not contain a PlatooningBeacon");

    if (positionHelper->isInSamePlatoon(epkt->getVehicleId())) {

      // if the message comes from the leader
      if (epkt->getVehicleId() == positionHelper->getLeaderId()) {
        traciVehicle->setPlatoonLeaderData(
            epkt->getSpeed(), epkt->getAcceleration(), epkt->getPositionX(),
            epkt->getPositionY(), epkt->getTime());
      }
      // if the message comes from the vehicle in front
      if (epkt->getVehicleId() == positionHelper->getFrontId()) {
        traciVehicle->setPrecedingVehicleData(
            epkt->getSpeed(), epkt->getAcceleration(), epkt->getPositionX(),
            epkt->getPositionY(), epkt->getTime());
      }
      // send data about every vehicle to the CACC. this is needed by the
      // consensus controller
      struct Plexe::VEHICLE_DATA vehicleData;
      vehicleData.index =
          positionHelper->getMemberPosition(epkt->getVehicleId());
      vehicleData.acceleration = epkt->getAcceleration();
      // for now length is fixed to 4 meters. TODO: take it from sumo
      vehicleData.length = 4;
      vehicleData.positionX = epkt->getPositionX();
      vehicleData.positionY = epkt->getPositionY();
      vehicleData.speed = epkt->getSpeed();
      vehicleData.time = epkt->getTime();
      // send information to CACC
      traciVehicle->setGenericInformation(CC_SET_VEHICLE_DATA, &vehicleData,
                                          sizeof(struct Plexe::VEHICLE_DATA));
    }
  } else if (enc->getKind() == BaseProtocol::CONTRACT_TYPE) {
    // TODO Jeremy
  }
}

void BaseApp::handleLowerControl(cMessage *msg) { delete msg; }

void BaseApp::onData(WaveShortMessage *wsm) {}

void BaseApp::sendUnicast(cPacket *msg, int destination) {
  UnicastMessage *unicast = new UnicastMessage();
  unicast->setDestination(destination);
  unicast->encapsulate(msg);
  sendDown(unicast);
}

void BaseApp::handleSelfMsg(cMessage *msg) {
  if (msg == recordData) {
    // check for simulation end. let the first vehicle check
    if (myId == 0 && simTime() > simulationDuration)
      stopSimulation();
    // log mobility data
    logVehicleData();
    // re-schedule next event
    scheduleAt(simTime() + SimTime(100, SIMTIME_MS), recordData);
  } else if (msg == contractChain) {
    // if we get here, then it's either the first contract chain, or the last
    // contract chain failed to complete in time
    // TODO: Log a failure
    // Start a new contract chain
    startNewContractChain();
  }
}

void BaseApp::startNewContractChain() {
  // create an instance of new cPacket class
  // TODO Jeremy
}

void BaseApp::stopSimulation() {
  simulationCompleted = true;
  endSimulation();
}

void BaseApp::onBeacon(WaveShortMessage *wsm) {}
