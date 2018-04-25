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
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
bool BaseApp::crashHappened = false;
bool BaseApp::simulationCompleted = false;
bool initialized = false;

/* clang-format off */
/*
																|- 1hop - (data)
																|- 2 hops - (data)
						|-- one signature --|
						|                   |- 7hops - (data)
delay log - |
						|-- two signatures
						|
						|-- three signatures
						|     ....
						|--   ....


*/
/* clang-format on */
vector<vector<vector<double>>> delay_data;
vector<vector<unsigned int>> pointer;

// invariant: put log in an increasing order: one signature, two signatures etc.
//           put short delay log first and long the second
vector<vector<string>> DSRC_LOGS = {
    {string(ONE_SIGNATURE_ONE_HOP_DELAY_LOG),
     string(ONE_SIGNATURE_ONE_HOP_DELAY_LOG)},
    {string(TWO_SIGNATURES_ONE_HOP_DELAY_LOG),
     string(TWO_SIGNATURES_ONE_HOP_DELAY_LOG)},
    {string(THREE_SIGNATURES_ONE_HOP_DELAY_LOG),
     string(THREE_SIGNATURES_TWO_HOP_DELAY_LOG)},
    {string(FOUR_SIGNATURES_ONE_HOP_DELAY_LOG),
     string(FOUR_SIGNATURES_THREE_HOP_DELAY_LOG)},
    {string(FIVE_SIGNATURES_ONE_HOP_DELAY_LOG),
     string(FIVE_SIGNATURES_FOUR_HOP_DELAY_LOG)},
    {string(SIX_SIGNATURES_ONE_HOP_DELAY_LOG),
     string(SIX_SIGNATURES_FIVE_HOP_DELAY_LOG)},
    {string(SEVEN_SIGNATURES_ONE_HOP_DELAY_LOG),
     string(SEVEN_SIGNATURES_SIX_HOP_DELAY_LOG)},
    {string(EIGHT_SIGNATURES_SEVEN_HOP_DELAY_LOG),
     string(EIGHT_SIGNATURES_SEVEN_HOP_DELAY_LOG)}};

void initialize() {
  for (unsigned int i = 0; i < DSRC_LOGS.size(); ++i) {
    delay_data.push_back(vector<vector<double>>());
    pointer.push_back(vector<unsigned int>());
    for (unsigned int j = 0; j < DSRC_LOGS[i].size(); ++j) {
      pointer[i].push_back(0);
      vector<double> data;
      ifstream infile;
      string file_name = string(DSRC_LOG_FOLDER) + "/" + DSRC_LOGS[i][j];
      infile.open(file_name.c_str());
      if (!infile) {
        printf("Failed to read file\n");
        exit(1);
      }

      double data_point = 0;
      while (infile >> data_point) {
        infile >> data_point;
        data.push_back(data_point);
      }

      delay_data[i].push_back(move(data));
    }
  }
  initialized = true;
}

double getDsrcDelayTimeBySigNumAndRange(unsigned int signature_num,
                                        unsigned int range) {
  unsigned int idx = pointer[signature_num - 1][range];
  double delay = delay_data[signature_num - 1][range][idx];
  pointer[signature_num - 1][range] =
      (idx + 1) % delay_data[signature_num - 1][range].size();
  if (delay > TIMEOUT_THRESHOLD) {
    return PACKET_LOSS;
  }
  return delay;
}

double getDsrcDelayTime(unsigned int sender, unsigned int targetReceiver) {
  if (!initialized) {
    initialize();
  }
  if (targetReceiver != 0) {
    return getDsrcDelayTimeBySigNumAndRange(sender + 1, SHORT_RANGE);
  } else {
    return getDsrcDelayTimeBySigNumAndRange(sender + 1, LONG_RANGE);
  }
}

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
    contractChainDelayOut.setName("contractChainDelay");
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

  bool delete_enc = true;
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
    // printf("epkt recipient: %d\n", epkt->getRecipient());
    // printf("contract id: %u\n", contract_chain.contract_id);
    // printf("seq_num: %u\n", contract_chain.seq_num);
    // printf("sent_time: %f\n", contract_chain.sent_time);
    // printf("valid_time: %f\n", contract_chain.valid_time);
    // printf("recovery_phase_timeout: %f\n",
    //       contract_chain.recovery_phase_timeout);
    // printf("chain_length: %u\n", contract_chain.chain_length);
    // uint8_t *co = contract_chain.chain_order;
    // printf("order: %u %u %u %u %u %u %u %u %u\n", co[0], co[1], co[2], co[3],
    //       co[4], co[5], co[6], co[7], co[8]);
    // printf("speeds, accels, max decel: %f %f, %f %f, %f\n",
    //       contract_chain.upper_speed, contract_chain.lower_speed,
    //       contract_chain.upper_accel, contract_chain.lower_accel,
    //       contract_chain.max_decel);
    // check whether we are the intended recipient
    if (epkt->getRecipient() == myId) {
      // printf("Vehicle %d received Contract Packet at time %f\n", myId,
      //       simTime().dbl());
      // make sure this message is still valid, otherwise ignore it
      if (simTime().dbl() < contract_chain.valid_time) {
        // printf("contract is still valid\n");

        // make sure this message is something this vehicle will accept (i.e. is
        // safe).
        // This is not something we are currently doing, but an AI would need to
        // do.

        // Collect any signatures that are on this contract already
        cArray arr = epkt->getParList();
        cObject *obj_ref;
        cOwnedObject *oobj_ref;
        cMsgPar *par_ref;
        char sig_name[6];
        cp_ec256_signature_t signatures[MAX_PLATOON_VEHICLES];
        memset((void *)signatures, 0,
               MAX_PLATOON_VEHICLES * sizeof(cp_ec256_signature_t));
        ContractSignature *sig_ref;
        for (int i = 0; i < MAX_PLATOON_VEHICLES; i++) {
          snprintf(sig_name, 6, "sig_%1d", i);
          // try to get sig_%1d from epkt
          obj_ref = arr.get(sig_name);
          if (obj_ref != NULL) { // then we got this signature
            par_ref = dynamic_cast<cMsgPar *>(obj_ref);
            ASSERT2(par_ref, "Attached cObject is not of type cMsgPar");
            oobj_ref = par_ref->getObjectValue();
            sig_ref = dynamic_cast<ContractSignature *>(oobj_ref);
            if (sig_ref == NULL) {
              printf(
                  "Attached signature '%s' is not of type ContractSignature\n",
                  sig_name);
              ASSERT2(false,
                      "Attached signature is not of type ContractSignature");
            }
            signatures[i] = sig_ref->getSignature();
            // printf("Got %s: %x\n", sig_name, *(unsigned int
            // *)&signatures[i]);
          }
        }

        // Pass the completed contract chain to the enclave for verification
        // and to do any updates necessary
        cp_ec256_signature_t new_signature;
        cp_ec256_signature_t empty_signature;
        memset((void *)&new_signature, 0, sizeof(cp_ec256_signature_t));
        memset((void *)&empty_signature, 0, sizeof(cp_ec256_signature_t));
        double compute_time;
        traciVehicle->sendVehicleContractChainGetSignature(
            contract_chain, &new_signature, MAX_PLATOON_VEHICLES, signatures,
            &compute_time); // new_signature gets populated
        // if signature was not set (memcmp returns zero if matches empty_sig)
        if (!memcmp((void *)&new_signature, (void *)&empty_signature,
                    sizeof(cp_ec256_signature_t))) {
          printf("Could not get enclave to sign new contract chain!\n");
          return;
        }
        printf("Compute time: %f\n", compute_time);
        // printf("Got signature: 0x%x\n", *(unsigned int *)&new_signature);

        // if we're the leader and just received a completed normal chain, we
        // can start the next chain immediately
        if (positionHelper->isLeader() &&
            contract_chain.contract_type == COMMPACT_NORMAL &&
            contract_chain.chain_order[contract_chain.chain_length - 1] ==
                myId) {
          // record how long the last contract chain took
          contractChainDelayOut.record(simTime().dbl() -
                                       contract_chain.sent_time + compute_time);
          printf("Completed contract chain took %f simulation time seconds\n",
                 simTime().dbl() - contract_chain.sent_time + compute_time);
          // cancel any upcoming contractChain event
          cancelEvent(contractChain);
          // start a new contract chain
          startNewContractChain();
          // create a new contractChain event in case the chain doesn't
          // complete in time
          scheduleAt(
              simTime() + compute_time +
                  SimTime(recovery_chain_completion_deadline, SIMTIME_MS),
              contractChain);
        }
        // if we're not the leader ending a normal chain, but the contract
        // ended with us (i.e. leave/split procedure)
        else if (contract_chain.chain_order[contract_chain.chain_length - 1] ==
                 myId) {
          printf("Check how we got here! Why is a non-leader %d the last "
                 "recipient of this contract chain?",
                 myId);
        }
        // the chain has not ended yet, so we should pass it on
        else {
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
            // reuse existing ContractChain after appending the new signature
            int position = positionHelper->getPosition();
            snprintf(sig_name, 6, "sig_%1d", position);
            epkt->addPar(sig_name);
            // create a new instance of signature and attach it to the chain
            ContractSignature *sig_message = new ContractSignature;
            sig_message->setSignature(new_signature);
            epkt->par(sig_name).setObjectValue(sig_message);
            // send the new ContractChain
            epkt->setRecipient((unsigned char)next);
            double DSRC_delay_time =
                getDsrcDelayTime(position, (unsigned int)next);
            compute_time = compute_time + DSRC_delay_time;
            if (DSRC_delay_time > 0) {
              // printf("DSRC delay: %f\n", DSRC_delay_time);
              sendContractChain(epkt, compute_time);
            } else {
              // record that we're dropping a contract chain
              printf("\n\nPacket dropped by vehicle %d\n\n\n", position);
              contractChainDelayOut.record(-10 * position - 1);
            }
            delete_enc = false;
          }
        }
      }
    }
  }

  if (delete_enc) {
    delete enc;
  }
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
  } else if (msg->getKind() == BaseProtocol::CONTRACT_TYPE) {
    // if we got a contract_type message that's not contractChain, then it's a
    // message we want to send down, but had to delay, so send it down now
    sendContractChain(dynamic_cast<ContractChain *>(msg), 0.0);
  }
}

void BaseApp::startNewContractChain() {
  // printf("Starting new contract chain from vehicle %d\n", myId);
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
  cp_ec256_signature_t new_signature;
  cp_ec256_signature_t empty_signature;
  memset((void *)&new_signature, 0,
         sizeof(cp_ec256_signature_t)); // set to zero
  memset((void *)&empty_signature, 0,
         sizeof(cp_ec256_signature_t)); // set to zero
  // printf("Sending new contract to vehicle %d enclave for signature\n", myId);
  double compute_time;
  traciVehicle->sendVehicleContractChainGetSignature(
      params, &new_signature, 0, NULL,
      &compute_time); // new_signature gets populated
  // if signature was not set (memcmp returns zero if matches empty_sig)
  if (!memcmp((void *)&new_signature, (void *)&empty_signature,
              sizeof(cp_ec256_signature_t))) {
    printf("Could not get enclave to sign new contract chain!\n");
    return;
  }
  // printf("Got signature: 0x%x\n", *(unsigned int *)&new_signature);

  // create an instance of new cPacket class
  ContractChain *contract_chain = new ContractChain();
  contract_chain->setKind(BaseProtocol::CONTRACT_TYPE);
  contract_chain->setContract_chain(params);
  int position = positionHelper->getPosition();
  char sig_name[6];
  snprintf(sig_name, 6, "sig_%1d", position);
  contract_chain->addPar(sig_name);
  // create a new instance of signature and attach it to the chain
  ContractSignature *sig_message = new ContractSignature;
  sig_message->setSignature(new_signature);
  contract_chain->par(sig_name).setObjectValue(sig_message);

  // set the recipient of this packet based on the chain_order
  if (params.chain_length < 2) {
    printf("Not enough vehicles in chain_order!\n");
    return;
  }
  contract_chain->setRecipient(params.chain_order[1]);
  double DSRC_delay_time = getDsrcDelayTime(position, params.chain_order[1]);
  compute_time = compute_time + DSRC_delay_time;
  // send contract chain down
  if (DSRC_delay_time > 0) {
    sendContractChain(contract_chain, compute_time);
  } else {
    // record that we're dropping a contract chain
    printf("\n\nPacket dropped by vehicle %d\n\n\n", position);
    contractChainDelayOut.record(-10 * position - 1);
  }
}

void BaseApp::sendContractChain(ContractChain *contract_chain, double delay) {
  if (delay == 0.0) {
    UnicastMessage *unicast = new UnicastMessage();
    unicast->setDestination(-1); // broadcast
    unicast->encapsulate(contract_chain);
    unicast->setKind(BaseProtocol::CONTRACT_TYPE);
    sendDown(unicast);
  } else {
    scheduleAt(simTime().dbl() + delay, contract_chain);
  }
}

void BaseApp::stopSimulation() {
  simulationCompleted = true;
  endSimulation();
}

void BaseApp::onBeacon(WaveShortMessage *wsm) {}
