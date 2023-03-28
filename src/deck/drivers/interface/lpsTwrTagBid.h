#ifndef __LPS_TWR_TAG_BID_H__
#define __LPS_TWR_TAG_BID_H__

#include "locodeck.h"
#include "libdw1000.h"

#include "mac.h"

#define LPS_TWR_POLL    0x01   
#define LPS_TWR_ANSWER  0x02
#define LPS_TWR_FINAL   0x03
#define LPS_TWR_REPORT  0x04 
#define LPS_TWR_DYNAMIC 0x05

#define LPS_TWR_TYPE  0
#define LPS_TWR_SEQ   1
#define LPS_TWR_ENABLE

// Number of UWB decks in the swarm
#define NumUWB 3 

extern uwbAlgorithm_t uwbTwrBidTagAlgorithm;

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];
  uint16_t reciprocalDistance;
  float_t selfVx;
  float_t selfVy;
  float_t selfGz;
  float_t selfh;
  bool keep_flying;
} __attribute__((packed)) lpsTwrTagBidReportPayload_t;

bool twrGetSwarmInfo(int robNum, uint16_t* range, float* vx, float* vy, float* gyroZ, float* height);
bool command_share(int RobIDfromControl, bool keep_flying);

typedef struct {
  const uint64_t antennaDelay;
  locoAddress_t tagAddress;
} lpsTwrBidAlgoOptions_t;


#endif // __LPS_TWR_TAG_BID_H__