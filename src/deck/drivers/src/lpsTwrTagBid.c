
#include <string.h>
#include <math.h>

#include "lpsTwrTagBid.h"

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "crtp_localization_service.h"

#include "stabilizer_types.h"
#include "estimator.h"
#include "cf_math.h"

#include "physicalConstants.h"
#include "configblock.h"
#include "static_mem.h"
#include "debug.h"

#define DEBUG_MODULE "TWRBID"

// This will only store the tag address and the antenna delay
static lpsTwrBidAlgoOptions_t twrBidOptions = {
    .tagAddress = 0xbccf000000000000,
    .antennaDelay = LOCODECK_ANTENNA_DELAY
};

// Defining the state of the tag
typedef struct {
    uint16_t distance[NumUWB];
} swarmInfo_t;

// Median filter for distance ranging (size=3)
typedef struct {
    uint16_t distance_history[3];
    uint8_t index_inserting;
} median_data_t;

static uint8_t selfID;
static locoAddress_t selfAddress;
static swarmInfo_t state;
static lpsTwrBidAlgoOptions_t* options = &twrBidOptions;

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static packet_t txPacket;
static bool rangingOk;

// Communication logic between each UWB
static bool current_mode_trans;
static uint8_t current_receiveID;

static bool checkTurn; // check if the receiving UWB turns into transmitting mode
static uint32_t checkTurnTick = 0;

// Median data
static median_data_t median_data[NumUWB];
#define ABS(a) ((a) > 0 ? (a) : -(a))

static uint16_t median_filter_3(uint16_t *data)
{
  uint16_t middle;
  if ((data[0] <= data[1]) && (data[0] <= data[2])){
    middle = (data[1] <= data[2]) ? data[1] : data[2];
  }
  else if((data[1] <= data[0]) && (data[1] <= data[2])){
    middle = (data[0] <= data[2]) ? data[0] : data[2];
  }
  else{
    middle = (data[0] <= data[1]) ? data[0] : data[1];
  }
  return middle;
}

static void twrTagInit(dwDevice_t *dev)
{
    // Initialize the packet in the TX buffer
    memset(&txPacket, 0, sizeof(txPacket));
    MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
    txPacket.pan = 0xbccf;

    memset(&poll_tx, 0, sizeof(poll_tx));
    memset(&poll_rx, 0, sizeof(poll_rx));
    memset(&answer_tx, 0, sizeof(answer_tx));
    memset(&answer_rx, 0, sizeof(answer_rx));
    memset(&final_tx, 0, sizeof(final_tx));
    memset(&final_rx, 0, sizeof(final_rx));

    // selfID goes from 0 to NumUWB-1
    selfID = (uint8_t)(((configblockGetRadioAddress()) & 0x000000000f) - 1);
    selfAddress = options->tagAddress + selfID;

    if (selfID==0) // Start the cf Num 1 as sender
    {
        current_mode_trans = true;
        current_receiveID = NumUWB-1; // the First one in receiving will be the last tag
        dwSetReceiveWaitTimeout(dev, 1000);
    }
    else
    {
        current_mode_trans = false;
        dwSetReceiveWaitTimeout(dev, 10000);    
    }

    // Initialize the state of the swarm
    for (int i = 0; i < NumUWB; i++) {
        median_data[i].index_inserting = 0;
    }

    checkTurn = false;
    rangingOk = false;
}

static void txcallback(dwDevice_t *dev)
{
    // time measurement
    dwTime_t departure;
    dwGetTransmitTimestamp(dev, &departure);
    departure.full += (options->antennaDelay / 2);

    if (current_mode_trans) // sender mode
    {
        switch (txPacket.payload[LPS_TWR_TYPE])
        {
            case LPS_TWR_POLL:
                poll_tx = departure;
                break;
            
            case LPS_TWR_FINAL:
                final_tx = departure;
                break;
        
            case LPS_TWR_DYNAMIC:
                if( (current_receiveID == 0) || (current_receiveID-1 == selfID) ){
                        current_mode_trans = false;
                        dwIdle(dev);
                        dwSetReceiveWaitTimeout(dev, 10000);
                        dwNewReceive(dev);
                        dwSetDefaults(dev);
                        dwStartReceive(dev);
                        checkTurn = true;
                        checkTurnTick = xTaskGetTickCount();
                }
                else
                {
                        current_receiveID = current_receiveID - 1;
                }
                break;
        }
    }
    else // receiver mode
    {
        switch (txPacket.payload[LPS_TWR_TYPE])
        {
            case LPS_TWR_ANSWER:
                answer_tx = departure;
                break;
            
            case LPS_TWR_REPORT:
            break;
        }
    }
}

static void rxcallback(dwDevice_t *dev) 
{
    // creating time stamp
    dwTime_t arival = { .full=0 };

    int dataLength = dwGetDataLength(dev);
    if (dataLength == 0) return;

    // initializing rxPacket
    packet_t rxPacket;
    memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);
    dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

    // if the package is not for me. We set to receive mode
    if (rxPacket.destAddress != selfAddress) 
    {
        if(current_mode_trans)
        {
            dwIdle(dev);
            dwSetReceiveWaitTimeout(dev, 10000);
        }
        dwNewReceive(dev);
        dwSetDefaults(dev);
        dwStartReceive(dev);
        return;
    }

    // we change the address to send it back to its owner
    txPacket.destAddress = rxPacket.sourceAddress;
    txPacket.sourceAddress = rxPacket.destAddress;

    if (current_mode_trans)
    {
        switch(rxPacket.payload[LPS_TWR_TYPE]) 
        {
            case LPS_TWR_ANSWER:
            {
                // change the label to the next step FINAL
                txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_FINAL;
                txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];

                // get the time stamp
                dwGetReceiveTimestamp(dev, &arival);
                arival.full -= (options->antennaDelay / 2);

                // store the timne stamp
                answer_rx = arival;

                // send back the message with FINAL tag
                dwNewTransmit(dev);
                dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
                dwWaitForResponse(dev, true);
                dwStartTransmit(dev);
                break;
            }
            case LPS_TWR_REPORT:
            {   
                // change the label to the next step DYNAMIC
                txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_DYNAMIC;
                txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];

                // extract payload to get the time stamps
                lpsTwrTagBidReportPayload_t *report = (lpsTwrTagBidReportPayload_t *)(rxPacket.payload + 2);
                double tround1, treply1, treply2, tround2, tprop_ctn, tprop;

                memcpy(&poll_rx, &report->pollRx, 5);
                memcpy(&answer_tx, &report->answerTx, 5);
                memcpy(&final_rx, &report->finalRx, 5);
                
                // Calculating the time of flight
                tround1 = answer_rx.low32 - poll_tx.low32;
                treply1 = answer_tx.low32 - poll_rx.low32;
                tround2 = final_rx.low32 - answer_tx.low32;
                treply2 = final_tx.low32 - answer_rx.low32;

                tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);
                
                tprop = tprop_ctn / LOCODECK_TS_FREQ;
                // Calculating distance
                uint16_t calcDist = (uint16_t)(1000 * (SPEED_OF_LIGHT * tprop + 1));

                if(calcDist!=0)
                {
                    // Basic filtering
                    uint16_t medianDist = median_filter_3(median_data[current_receiveID].distance_history);
                    if (ABS(medianDist-calcDist)>500)
                        state.distance[current_receiveID] = medianDist;
                    else
                        state.distance[current_receiveID] = calcDist;
                    
                    // Circular insertion 0 --> 1 --> 2 --> 0
                    median_data[current_receiveID].index_inserting++;
                    if(median_data[current_receiveID].index_inserting==3)
                        median_data[current_receiveID].index_inserting = 0;
                    median_data[current_receiveID].distance_history[median_data[current_receiveID].index_inserting] = calcDist;        
                    rangingOk = true;  
                }

                // make a new report and send it back with the dynamic tag
                lpsTwrTagBidReportPayload_t *dynamic = (lpsTwrTagBidReportPayload_t *)(txPacket.payload+2);                
                dynamic->reciprocalDistance = calcDist;
                // estimatorKalmanGetEstimatedZ(&dynamic->selfHeight);

                // Transmision
                dwNewTransmit(dev);
                dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2 + sizeof(lpsTwrTagBidReportPayload_t));
                dwWaitForResponse(dev, true);
                dwStartTransmit(dev);
                break;
            }
        }
    }
    else
    {
        switch(rxPacket.payload[LPS_TWR_TYPE]) 
        {
            case LPS_TWR_POLL:
            {
                // change the label to the next step ANSWER
                txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_ANSWER;
                txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];

                // get the time stamp
                dwGetReceiveTimestamp(dev, &arival);
                arival.full -= (options->antennaDelay / 2);

                // store the timne stamp
                poll_rx = arival;

                // send back the message with ANSWER tag
                dwNewTransmit(dev);
                dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
                dwWaitForResponse(dev, true);
                dwStartTransmit(dev);
                break;
            }
            case LPS_TWR_FINAL:
            {
                // change the label to the next step REPORT
                txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT;
                txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];

                // get the time stamp
                dwGetReceiveTimestamp(dev, &arival);
                arival.full -= (options->antennaDelay / 2);

                // store the timne stamp
                final_rx = arival;

                // make a new report and send it back with the REPORT tag
                lpsTwrTagBidReportPayload_t *report = (lpsTwrTagBidReportPayload_t *)(txPacket.payload+2);
                memcpy(&report->pollRx, &poll_rx, 5);
                memcpy(&report->answerTx, &answer_tx, 5);
                memcpy(&report->finalRx, &final_rx, 5);

                // fill the packet
                // estimatorKalmanGetEstimatedZ(&report->selfHeight);

                // Transmision
                dwNewTransmit(dev);
                dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2 + sizeof(lpsTwrTagBidReportPayload_t));
                dwWaitForResponse(dev, true);
                dwStartTransmit(dev);
                break;
            }
            case LPS_TWR_DYNAMIC:
            {
                // This is the tricky one. In this case, you are receiver
                // Receiving dynamic means the communication with the previous sender is done

                lpsTwrTagBidReportPayload_t *report2 = (lpsTwrTagBidReportPayload_t *)(rxPacket.payload+2);
                uint8_t rangingID = (uint8_t)(rxPacket.sourceAddress & 0xFF);
                
                if((report2->reciprocalDistance)!=0)
                {
                    // median filter
                    uint16_t calcDist = report2->reciprocalDistance;
                    uint16_t medianDist = median_filter_3(median_data[rangingID].distance_history);

                    if (ABS(medianDist-calcDist)>500)
                        state.distance[rangingID] = medianDist;
                    else
                        state.distance[rangingID] = calcDist;
                    median_data[rangingID].index_inserting++;
                    
                    // Circular insertion 0 --> 1 --> 2 --> 0
                    if(median_data[rangingID].index_inserting==3)
                        median_data[rangingID].index_inserting = 0;
                    median_data[rangingID].distance_history[median_data[rangingID].index_inserting] = calcDist;
                }
                rangingOk = true;

                uint8_t fromID = (uint8_t)(rxPacket.sourceAddress & 0xFF);
                           
                if( selfID == fromID + 1 || selfID == 0 )
                {
                    current_mode_trans = true;
                    dwIdle(dev);
                    dwSetReceiveWaitTimeout(dev, 1000);
                    if(selfID == NumUWB - 1) // CF num 3
                        current_receiveID = 0;
                    else
                        current_receiveID = NumUWB - 1;
                    if(selfID == 0)
                        current_receiveID = NumUWB - 2; // immediate problem
                    
                    txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
                    txPacket.payload[LPS_TWR_SEQ] = 0;
                    txPacket.sourceAddress = selfAddress;
                    txPacket.destAddress = options->tagAddress + current_receiveID;

                    dwNewTransmit(dev);
                    dwSetDefaults(dev);
                    dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
                    dwWaitForResponse(dev, true);
                    dwStartTransmit(dev);
                }
                else
                {
                    dwNewReceive(dev);
                    dwSetDefaults(dev);
                    dwStartReceive(dev);
                }
                break;
            }
        }
    }
}


static uint32_t twrTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
    switch (event)
    {
    case eventPacketReceived:
        rxcallback(dev);
        checkTurn = false;
        break;
    
    case eventPacketSent:
        txcallback(dev);
        break;  
    
    case eventTimeout:  // Comes back to timeout after each ranging attempt
    case eventReceiveTimeout:
    case eventReceiveFailed:
        // if received failed and its sender, send again   
        if (current_mode_trans==true)
        {
            txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
            txPacket.payload[LPS_TWR_SEQ] = 0;
            txPacket.sourceAddress = selfAddress;
            txPacket.destAddress = options->tagAddress + current_receiveID;
            dwNewTransmit(dev);
            dwSetDefaults(dev);
            dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
            dwWaitForResponse(dev, true);
            dwStartTransmit(dev);
        }

        // if not, check if its your turn or receive again
        else
        {
            if(xTaskGetTickCount() > checkTurnTick + 20) // > 20ms
            {
                if(checkTurn == true)
                {
                    current_mode_trans = true;
                    dwIdle(dev);
                    dwSetReceiveWaitTimeout(dev, 1000);
                    txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
                    txPacket.payload[LPS_TWR_SEQ] = 0;
                    txPacket.sourceAddress = selfAddress;
                    txPacket.destAddress = options->tagAddress + current_receiveID;
                    dwNewTransmit(dev);
                    dwSetDefaults(dev);
                    dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
                    dwWaitForResponse(dev, true);
                    dwStartTransmit(dev);
                    checkTurn = false;
                    break;
                }
            }
        
            dwNewReceive(dev);
            dwSetDefaults(dev);
            dwStartReceive(dev);      
        }
        break;

    default:
      configASSERT(false);
    }

    return MAX_TIMEOUT;

}

static bool isRangingOk()
{
  return rangingOk;
}

static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
  if (anchorId < NumUWB-1)
    return true;
  else
    return false;
}

static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  for (int i = 0; i < NumUWB-1; i++) {
    unorderedAnchorList[i] = i;
  }
  return NumUWB-1;
}

static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  uint8_t count = 0;
  for (int i = 0; i < NumUWB-1; i++) {
      unorderedAnchorList[count] = i;
      count++;
  }
  return count;
}

uwbAlgorithm_t uwbTwrBidTagAlgorithm = {
  .init = twrTagInit,
  .onEvent = twrTagOnEvent,
  .isRangingOk = isRangingOk,
  .getAnchorPosition = getAnchorPosition,
  .getAnchorIdList = getAnchorIdList,
  .getActiveAnchorIdList = getActiveAnchorIdList,
};

LOG_GROUP_START(ranging)
LOG_ADD(LOG_UINT16, distance0, &state.distance[0])
LOG_ADD(LOG_UINT16, distance1, &state.distance[1])
LOG_ADD(LOG_UINT16, distance2, &state.distance[2])
LOG_ADD(LOG_UINT16, distance3, &state.distance[3])
LOG_ADD(LOG_UINT16, distance4, &state.distance[4])
LOG_GROUP_STOP(ranging)