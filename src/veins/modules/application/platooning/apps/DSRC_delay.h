//
// Created by chshibo on 3/30/18.
//
#include <string>
#ifndef DSRC_DELAY_H
#define DSRC_DELAY_H

#define DSRC_LOG_FOLDER "/tmp/results"
#define DSRC_LOG_FILE_NUM 10
#define ONE_SIGNATURE_SHORT_DELAY_LOG "1000_128_short.log"
#define ONE_SIGNATURE_LONG_DELAY_LOG "1000_128_long.log"
#define TWO_SIGNATURES_SHORT_DELAY_LOG "1000_192_short.log"
#define THREE_SIGNATURES_SHORT_DELAY_LOG "1000_256_short.log"
#define FOUR_SIGNATURES_SHORT_DELAY_LOG "1000_320_short.log"
#define FIVE_SIGNATURES_SHORT_DELAY_LOG "1000_384_short.log"
#define SIX_SIGNATURES_SHORT_DELAY_LOG "1000_448_short.log"
#define SEVEN_SIGNATURES_SHORT_DELAY_LOG "1000_512_short.log"
#define EIGHT_SIGNATURES_SHORT_DELAY_LOG "1000_576_short.log"
#define EIGHT_SIGNATURES_LONG_DELAY_LOG "1000_576_long.log"

void initialize();

double getDsrcDelayTime(int CONTRACT_TYPE, unsigned int sender,
                        unsigned int targetReceiver);

#endif // UNTITLED_DSRC_DELAY_H
