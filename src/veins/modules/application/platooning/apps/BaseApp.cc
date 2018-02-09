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
    // enclave defaults
    contract_changed = false; // first contract will use default values, id of 0
    recovery_phase_duration = par("recoveryPhaseDuration").longValue();
    recovery_chain_completion_deadline =
        par("recoveryChainCompletionDeadline").longValue();
    upper_speed = par("upperSpeed").doubleValue();
    lower_speed = par("lowerSpeed").doubleValue();
    upper_accel = par("upperAccel").doubleValue();
    lower_accel = par("lowerAccel").doubleValue();
    max_decel = par("maxDecel").doubleValue();
    contract_id = 0;
    seq_num = 0;
    contract_type = COMMPACT_NORMAL;
  }

  if (stage == 1) {
    mobility = Veins::TraCIMobilityAccess().get(getParentModule());
    traci = mobility->getCommandInterface();
    traciVehicle = mobility->getVehicleCommandInterface();
    positionHelper =
        FindModule<PositionHelper *>::findSubModule(getParentModule());
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
      SimTime startContractChain = SimTime(
          floor(simTime().dbl() * 1000) + recovery_chain_completion_deadline,
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
    // unpack ContractChain
    ContractChain *epkt = dynamic_cast<ContractChain *>(enc);
    ASSERT2(epkt, "received UnicastMessage does not contain a ContractChain");
    contract_chain_t contract_chain = epkt->getContract_chain();
    // check whether we are the intended recipient
    if (epkt->getRecipient() == myId) {
      printf("Vehicle %d received Contract Packet!\n", myId);
      // make sure this message is still valid, otherwise ignore it
      if (SimTime().dbl() < contract_chain.valid_time) {

        // make sure this message is something this vehicle will accept (i.e. is
        // safe).

        // Pass the completed contract chain to the enclave for verification and
        // to do any updates necessary

        // Collect timing information TODO

        // if we're the leader and just received a completed normal chain, we
        // can start the next chain immediately
        if (positionHelper->isLeader() &&
            contract_chain.contract_type == COMMPACT_NORMAL &&
            contract_chain.chain_order[contract_chain.chain_length - 1] ==
                myId) {
          // cancel any upcoming contractChain event
          cancelEvent(contractChain);
          // start a new contract chain
          startNewContractChain();
          // create a new contractChain event in case the chain doesn't complete
          // in time
          scheduleAt(simTime() + SimTime(recovery_chain_completion_deadline,
                                         SIMTIME_MS),
                     contractChain);
        } else if (contract_chain
                       .chain_order[contract_chain.chain_length - 1] == myId) {
          // if we're not the leader ending a normal chain, but the contract
          // ended with us (i.e. leave/split procedure)
          printf("Check how we got here! Why is a non-leader %d the last "
                 "recipient of this contract chain?",
                 myId);
        } else { // the chain has not ended yet, so we should pass it on
                 // find who is next
          int idx, next = -1;
          for (idx = 0; idx < contract_chain.chain_length - 1; idx++) {
            if (contract_chain.chain_order[idx] == myId) {
              next = contract_chain.chain_order[idx + 1];
              break;
            }
          }
          if (next == -1) {
            printf("Error: We did not find myId in chain_order! How did we get "
                   "this packet?\n");
          } else {
            // create new ContractChain with them as recipient
            ContractChain *new_contract_chain = new ContractChain();
            new_contract_chain->setRecipient((unsigned char)next);
            new_contract_chain->setContract_chain(contract_chain);
            // send the new ContractChain
            sendContractChain(new_contract_chain);
          }
        }
      }
    }
  }

  delete enc;
  delete unicast;
}

void BaseApp::logVehicleData(bool crashed) {
  // get distance and relative speed w.r.t. front vehicle
  double distance, relSpeed, acceleration, speed, controllerAcceleration, posX,
      posY, time;
  traciVehicle->getRadarMeasurements(distance, relSpeed);
  traciVehicle->getVehicleData(speed, acceleration, controllerAcceleration,
                               posX, posY, time);
  if (crashed)
    distance = 0;
  // write data to output files
  distanceOut.record(distance);
  relSpeedOut.record(relSpeed);
  nodeIdOut.record(myId);
  accelerationOut.record(acceleration);
  controllerAccelerationOut.record(controllerAcceleration);
  speedOut.record(mobility->getCurrentSpeed().x);
  Coord pos = mobility->getPositionAt(simTime());
  posxOut.record(pos.x);
  posyOut.record(pos.y);
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
    // Set a new event so we send out the next contract chain too
    scheduleAt(simTime() +
                   SimTime(recovery_chain_completion_deadline, SIMTIME_MS),
               contractChain);
  }
}

void BaseApp::startNewContractChain() {
  // check if we have a change to the contract, or just another chain in the
  // existing contract
  // NOTE: the enclave itself will check whether the contract has changed and
  // whether the new parameters are valid or not
  if (contract_changed) {
    contract_id++;
    seq_num = 0;
  } else {
    seq_num++;
  }
  contract_chain_t params;
  params.contract_id = contract_id;
  params.seq_num = seq_num;
  params.sent_time = simTime().dbl();
  params.valid_time =
      params.sent_time +
      SimTime(recovery_chain_completion_deadline, SIMTIME_MS).dbl();
  params.contract_type = contract_type;
  // set contract type-specific parameters
  switch ((int)contract_type) {
  case COMMPACT_NORMAL:
    // extend the timeout
    params.recovery_phase_timeout =
        params.sent_time + SimTime(recovery_phase_duration, SIMTIME_MS).dbl();
    // set chain order (for normal, front to back)
    params.chain_length = positionHelper->getPlatoonOrder(params.chain_order,
                                                          MAX_PLATOON_VEHICLES);
    // append leader to the chain order
    params.chain_order[params.chain_length] = myId;
    params.chain_length++;
    // set speed and accel bounds
    params.upper_speed = upper_speed;
    params.lower_speed = lower_speed;
    params.upper_accel = upper_accel;
    params.lower_accel = lower_accel;
    params.max_decel = max_decel;
    break;
  case COMMPACT_JOIN: // TODO: these types are not supported yet
  case COMMPACT_LEAVE:
  case COMMPACT_SPLIT:
    params.recovery_phase_timeout = 0; // do not extend timeout
    break;
  }

  // get signature from enclave
  cp_ec256_signature_t signature;
  cp_ec256_signature_t empty_signature;
  memset((void *)&signature, 0, sizeof(cp_ec256_signature_t)); // set to zero
  memset((void *)&empty_signature, 0,
         sizeof(cp_ec256_signature_t)); // set to zero
  traciVehicle->sendVehicleContractChainGetSignature(
      params, &signature, 0, NULL); // signature gets populated
  // if signature was not set (memcmp returns zero if matches empty_sig)
  if (!memcmp((void *)&signature, (void *)&empty_signature,
              sizeof(cp_ec256_signature_t))) {
    printf("Could not get enclave to sign new contract chain!\n");
    return;
  }

  // create an instance of new cPacket class
  ContractChain *contract_chain = new ContractChain();
  contract_chain->setKind(BaseProtocol::CONTRACT_TYPE);
  contract_chain->setContract_chain(params);
  int position = positionHelper->getPosition();
  char sig_name[6];
  snprintf(sig_name, 6, "sig_%1d", position);
  cMsgPar par = contract_chain->addPar(sig_name);
  // create a new instance of signature and attach it to the chain
  ContractSignature *sig_message = new ContractSignature;
  sig_message->setSignature(signature);
  par.setObjectValue(sig_message);

  // set the recipient of this packet based on the chain_order
  if (params.chain_length < 2) {
    printf("Not enough vehicles in chain_order!\n");
    return;
  }
  contract_chain->setRecipient(params.chain_order[1]);

  // send contract chain down
  sendContractChain(contract_chain);
}

void BaseApp::sendContractChain(ContractChain *contract_chain) {
  UnicastMessage *unicast = new UnicastMessage();
  unicast->setDestination(-1); // broadcast
  unicast->encapsulate(contract_chain);
  unicast->setKind(BaseProtocol::CONTRACT_TYPE);
  sendDown(unicast);
}

void BaseApp::stopSimulation() {
  simulationCompleted = true;
  endSimulation();
}

void BaseApp::onBeacon(WaveShortMessage *wsm) {}
