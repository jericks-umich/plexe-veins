//
// Created by chshibo on 3/30/18.
//

#include "DSRC_delay.h"
#include "commpact_types.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
using namespace std;

const char *DSRC_LOGS[] = {
    ONE_SIGNATURE_SHORT_DELAY_LOG,    ONE_SIGNATURE_LONG_DELAY_LOG,
    TWO_SIGNATURES_SHORT_DELAY_LOG,   THREE_SIGNATURES_SHORT_DELAY_LOG,
    FOUR_SIGNATURES_SHORT_DELAY_LOG,  FIVE_SIGNATURES_SHORT_DELAY_LOG,
    SIX_SIGNATURES_SHORT_DELAY_LOG,   SEVEN_SIGNATURES_SHORT_DELAY_LOG,
    EIGHT_SIGNATURES_SHORT_DELAY_LOG, EIGHT_SIGNATURES_LONG_DELAY_LOG};

bool initialized = false;
unordered_map<string, vector<double>> delay_data;
unordered_map<string, unsigned int> pointer;

double get_delay_time(string log) {
  double delay_time = delay_data[log][pointer[log]];
  pointer[log] = (pointer[log] + 1) % delay_data[log].size();
  return delay_time / 2;
}
void initialize() {
  for (int i = 0; i < DSRC_LOG_FILE_NUM; ++i) {
    vector<double> data;
    ifstream infile;
    string file_name = string(DSRC_LOG_FOLDER) + "/" + string(DSRC_LOGS[i]);
    infile.open(file_name.c_str());
    double dummy;
    if (!infile) {
      cerr << "Read dsrc error: " << file_name;
      exit(1);
    }
    while (infile >> dummy) {
      double data_point = 0;
      infile >> data_point;
      data.push_back(data_point);
    }

    pointer[DSRC_LOGS[i]] = 0;
    delay_data[DSRC_LOGS[i]] = move(data);
  }
  initialized = true;
}

double getDsrcDelayTime(int CONTRACT_TYPE, unsigned int sender,
                        unsigned int targetReceiver) {
  if (!initialized) {
    initialize();
  }
  switch (CONTRACT_TYPE) {
  case COMMPACT_NORMAL:
    if (sender == 0 && targetReceiver == 1) {
      return get_delay_time(ONE_SIGNATURE_SHORT_DELAY_LOG);
    } else if (sender == 1 && targetReceiver == 2) {
      return get_delay_time(TWO_SIGNATURES_SHORT_DELAY_LOG);
    } else if (sender == 2 && targetReceiver == 3) {
      return get_delay_time(THREE_SIGNATURES_SHORT_DELAY_LOG);
    } else if (sender == 3 && targetReceiver == 4) {
      return get_delay_time(FOUR_SIGNATURES_SHORT_DELAY_LOG);
    } else if (sender == 4 && targetReceiver == 5) {
      return get_delay_time(FIVE_SIGNATURES_SHORT_DELAY_LOG);
    } else if (sender == 5 && targetReceiver == 6) {
      return get_delay_time(SIX_SIGNATURES_SHORT_DELAY_LOG);
    } else if (sender == 6 && targetReceiver == 7) {
      return get_delay_time(SEVEN_SIGNATURES_SHORT_DELAY_LOG);
    } else if (sender == 7 && targetReceiver == 0) {
      return get_delay_time(EIGHT_SIGNATURES_LONG_DELAY_LOG);
    } else if (sender == 6 && targetReceiver == 0) {
      return get_delay_time(EIGHT_SIGNATURES_LONG_DELAY_LOG);
    } else if (sender == 5 && targetReceiver == 0) {
      return get_delay_time(EIGHT_SIGNATURES_LONG_DELAY_LOG);
    } else if (sender == 4 && targetReceiver == 0) {
      return get_delay_time(EIGHT_SIGNATURES_LONG_DELAY_LOG);
    } else if (sender == 3 && targetReceiver == 0) {
      return get_delay_time(EIGHT_SIGNATURES_LONG_DELAY_LOG);
    } else if (sender == 2 && targetReceiver == 0) {
      return get_delay_time(EIGHT_SIGNATURES_SHORT_DELAY_LOG);
    } else if (sender == 1 && targetReceiver == 0) {
      return get_delay_time(EIGHT_SIGNATURES_SHORT_DELAY_LOG);
    } else {
      cerr << "Invalid communication: sender: " << sender
           << " target receiver: " << targetReceiver;
      exit(1);
    }
  case COMMPACT_JOIN:
    break;
  case COMMPACT_LEAVE:
    break;
  case COMMPACT_SPLIT:
    break;
  default:
    break;
  }
}
