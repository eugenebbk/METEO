
#include "METEO_Parser.h"
#include "crc/crc16.h"

void txCmdMeteo(uint8_t cmd, uint8_t *payload)
{
    messageMETEO_t messageTxMETEO;
    messageTxMETEO.hat = 0x24;
    messageTxMETEO.func = cmd;
    memcpy(&messageTxMETEO.payloadCRC[0], payload, 1);
    Append_CRC16((uint8_t*)&messageTxMETEO, 5); //3+2
}

void rxCmdMeteo(uint8_t *payload)
{
    messageMETEO_t messageRxMETEO;











    messageRxMETEO.hat = 0x24;
    messageRxMETEO.func = cmd;
    memcpy(&messageRxMETEO.payloadCRC[0], payload, 1);
    Append_CRC16((uint8_t*)&messageRxMETEO, 5); //3+2
}



// uint8_t Ubx_putData(ubx_s *self, uint8_t in_data)
// {
//   static uint8_t lenghtTemp = 0;
//   static uint8_t StateMachinePars = 1;
//   self->Variable.StateMachineParsDebugMonitor = StateMachinePars;

//   if (self)
//   {
//     switch (StateMachinePars)
//     {
//     default:
//       StateMachinePars = 1;
//       // break;
//       return 0x00;

//     // "Waiting for startByte1"
//     case 1:
//       if (in_data == startByteConst1) //
//       {
//         StateMachinePars++;
//         self->Header.startByte1 = in_data; 
//         self->Variable.crc_ACalculate = 0;
//         self->Variable.crc_BCalculate = 0;
//         self->Variable.readyPackage = 0;
//         self->Variable.lenghtPayloadCurrent = 0;
//         self->Variable.errorMallocc = 0;
//         self->Payload = NULL;
//         lenghtTemp = 0;
//         free(self->Payload);
//       }
//       return 0x00; 

//     //"Waiting for startByte2"
//     case 2:
//       if (in_data == startByteConst2) //
//       {
//         self->Header.startByte2 = in_data; 
//         StateMachinePars++;
//       }
//       else
//       {
//         StateMachinePars = 1;
//       }
//       return 0x00;

//     //"Waiting for CLASS"
//     case 3:
//       if (in_data <= 0x28)
//       { // 0x28
//         self->Header.classType = in_data;
//         StateMachinePars++;
//         break;
//       }
//       else
//       {
//         StateMachinePars = 1;
//         return 0x00;
//       }

//     // Wait ID
//     case 4:
//       self->Header.ID = in_data;
//       StateMachinePars++;
//       break;

//     // Wait lenghtPayload
//     case 5:
//       if (lenghtTemp == 0) //lenghtPayload little
//       {
//         lenghtTemp = 1;
//         self->Header.Length = in_data;
//       }
//       else if (lenghtTemp == 1) 
//       {
//         (self->Header.Length) |= ((uint16_t)in_data) << 8;
//         lenghtTemp = 0;
//         if(!(self->Payload = (uint8_t *)malloc(self->Header.Length))){
//           StateMachinePars=1;
//           self->Variable.errorMallocc = 1;
//         }
//         else{
//           StateMachinePars++;
//         }
//       }
//       break;

//     // Wait Payload
//     case 6:
//       self->Payload[self->Variable.lenghtPayloadCurrent] = in_data;
//       self->Variable.lenghtPayloadCurrent++;
//       if (self->Header.Length <= (self->Variable.lenghtPayloadCurrent))
//       {
//         StateMachinePars++;
//       }
//       break;

//     // Wait CRC_A
//     case 7:
//       if (in_data == self->Variable.crc_ACalculate)
//       {
//         self->Header.CRC_A = in_data;
//         StateMachinePars++;
//         return 0x00; //
//       }
//       else
//       {
//         StateMachinePars = 1;
//         return 0x00; //
//       }

//     // Wait CRC_B
//     case 8:
//       if (in_data == self->Variable.crc_BCalculate)
//       {
//         self->Header.CRC_B = in_data;
//         StateMachinePars = 1;
//         self->Variable.readyPackage = 1;
//         return 0x01; //
//       }
//       else
//       {
//         StateMachinePars = 1;
//         return 0x00; //
//       }
//     }

//     // Calculation CRC
//     Fletcher8(in_data, &(self->Variable.crc_ACalculate), &(self->Variable.crc_BCalculate));
//   }
//   return 0x00;
// }