#ifndef _PARSERPC_H_
#define _PARSERPC_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "crcANSI.h"
#include "LogsSST26.h"

//-----config
#define REQUEST_PC_HEADER 0x33
#define REQUEST_PC_HEADER_SIZE 4
#define REQUEST_PC_CRC_SIZE 2
#define ANSWER_TO_PC_HEADER_SIZE 4
#define ANSWER_TO_PC_CRC_SIZE 2
#define PROTOCOL_USB_FULLSIZE (REQUEST_PC_CRC_SIZE + REQUEST_PC_HEADER_SIZE)

#define MAX_PAYLOAD_SIZE SIZE_LOG

//----------CMD
#define WriteTimePeriodEventLog 0x03
#define ReadEventLog 0x04
#define ClearEventLog 0x05
#define StartWriteDataToLog 0x08
#define StopWriteDataToLog 0x09
#define ModeBridgeMeteoblock 0x10
#define ModeBridgeGNSS 0x11
#define ModeBridgeAccumulator 0x12

//--debug cmd
#define AnswerData 0x40

typedef struct
{
    uint8_t headerConst;
    uint8_t cmd;
    uint16_t lenghtPayload;
    uint8_t payloadAndCRC[SIZE_LOG + REQUEST_PC_CRC_SIZE]; //maxPayloadData
} usb_protocol_t;

// typedef struct
// {
//     uint8_t *payload;
//     uint16_t crcANSI16;
// } usb_protocol_data_t;

// void parserRequestPC(uint8_t *messageRequestPC);

#endif /* _PARSERPC_H_ */