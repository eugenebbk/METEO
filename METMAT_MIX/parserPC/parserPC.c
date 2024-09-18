
#include "parserPC.h"

// extern configureModeMK_t configureModeMK;
// extern log3_t log3;

// //parser and answer usb
// void parserRequestPC(uint8_t *messageRequestPC) 
// {
//     uint8_t messageAnswerPC[261] = {0};

//     usb_protocol_t usb_protocol = {0};

//     usb_protocol.headerConst = REQUEST_PC_HEADER;
//     usb_protocol.cmd = messageRequestPC[1];
//     usb_protocol.lenghtPayload = 1;

//     static uint8_t stateMachineParserPC = 0;
//     uint8_t errorParserPC = 0;

//     uint8_t payloadParserPC = messageRequestPC[2];
//     uint16_t resultCompareCRC_temp = func_compare_crc16(messageRequestPC, REQUEST_PC_HEADER_SIZE + payloadParserPC + REQUEST_PC_CRC_SIZE); //

//     uint8_t *payload_ptr = messageRequestPC + REQUEST_PC_HEADER_SIZE - 1;
//     if (messageRequestPC[0] == REQUEST_PC_HEADER && resultCompareCRC_temp == 0) // ���� �������� ���� ��������� � ������
//     {
//         switch (messageRequestPC[1]) // ������ ������������ ������ �������
//         {
//             if (stateMachineParserPC == 0)
//             {
//             case WriteTimePeriodEventLog: // ������� ������ ���������
//                 configureModeMK.periodWritingDataToLog = (messageRequestPC[3] << 8) && (messageRequestPC[4]);

//                 // ����� ��������

//                 usb_protocol.payloadAndCRC[0] = 0;
//                 break; // ����� �� ����� �������

//             case ReadEventLog: //-
//                 usb_protocol.lenghtPayload = SIZE_LOG;

//                 // ����������
//                 // memcpy(&usb_protocol.payloadAndCRC[0], ,SIZE_LOG); //����������� ����
//                 break; // ����� �� ����� �������

//             case ClearEventLog:

//                 // ����������
//                 usb_protocol.payloadAndCRC[0] = 0;
//                 break; // ����� �� ����� �������

//             case ModeBridgeMeteoblock:
//                 configureModeMK.currentModeMK = ModeBridgeMeteoblock;
//                 // ����������
//                 usb_protocol.payloadAndCRC[0] = 0;
//                 break; // ����� �� ����� �������

//             case ModeBridgeGNSS:
//                 configureModeMK.currentModeMK = ModeBridgeGNSS;

//                 // ����������
//                 usb_protocol.payloadAndCRC[0] = 0;
//                 break; // ����� �� ����� �������

//             case ModeBridgeAccumulator:
//                 configureModeMK.currentModeMK = ModeBridgeAccumulator;

//                 // ����������
//                 usb_protocol.payloadAndCRC[0] = 0;
//                 break; // ����� �� ����� �������
//             }

//         case StopWriteDataToLog:
//             configureModeMK.currentModeMK = StopWriteDataToLog;
//             usb_protocol.lenghtPayload = 1;
//             stateMachineParserPC = 0;

//             // ����������
//             usb_protocol.payloadAndCRC[0] = 0;

//             break; // ����� �� ����� �������

//         case StartWriteDataToLog:
//             configureModeMK.currentModeMK = StartWriteDataToLog;
//             stateMachineParserPC = 1;

//             // ����������
//             usb_protocol.payloadAndCRC[0] = 0;
//             break; // ����� �� ����� �������

//         // case AnswerData:
//         //     configureModeMK.currentModeMK = StartWriteDataToLog;
//         //     stateMachineParserPC = 1;

//         //     // ����������
//         //     usb_protocol.payloadAndCRC[0] = 0;
//         //     break; // ����� �� ����� �������

//         default:
//             usb_protocol.payloadAndCRC[0] = 2; //data

//             uint16_t calcCRC16 = func_calc_crc16(usb_protocol, REQUEST_PC_HEADER_SIZE + usb_protocol.lenghtPayload);
//             usb_protocol.payloadAndCRC[usb_protocol.lenghtPayload + REQUEST_PC_HEADER_SIZE - 1] = (uint8_t)(calcCRC16 >> 8);
//             usb_protocol.payloadAndCRC[usb_protocol.lenghtPayload + REQUEST_PC_HEADER_SIZE] = (uint8_t)(calcCRC16 & 0xFF);
//             errorParserPC = 2; // error cmd
//             break;             // �����

//         } // ����� ������������ ������ �������
//         uint16_t calcCRC16 = func_calc_crc16(usb_protocol, REQUEST_PC_HEADER_SIZE + usb_protocol.lenghtPayload);
//         usb_protocol.payloadAndCRC[usb_protocol.lenghtPayload + REQUEST_PC_HEADER_SIZE - 1] = (uint8_t)(calcCRC16 >> 8);
//         usb_protocol.payloadAndCRC[usb_protocol.lenghtPayload + REQUEST_PC_HEADER_SIZE] = (uint8_t)(calcCRC16 & 0xFF);
//     }
//     else
//     {
//         usb_protocol.payloadAndCRC[0] = 1;

//         uint16_t calcCRC16 = func_calc_crc16(usb_protocol, REQUEST_PC_HEADER_SIZE + usb_protocol.lenghtPayload);
//         usb_protocol.payloadAndCRC[usb_protocol.lenghtPayload + REQUEST_PC_HEADER_SIZE - 1] = (uint8_t)(calcCRC16 >> 8);
//         usb_protocol.payloadAndCRC[usb_protocol.lenghtPayload + REQUEST_PC_HEADER_SIZE] = (uint8_t)(calcCRC16 & 0xFF);

//         errorParserPC = 1; // error crc and header
//     }

//     CDC_Transmit_FS(usb_protocol, PROTOCOL_USB_FULLSIZE + usb_protocol.lenghtPayload);
//     return errorParserPC; // no error or other error
// }



// //This function has hardware TIMER
// 	uint8_t ReadADS1232(void){   //

//     switch(StM_Ads){
//         default:
//             StM_Ads = 1;

// 				// "Waiting for readiness measurements from ADC"
//         case 1: 
// 						if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)  //miso spi
// 						{
//                 StM_Ads++;                                                       
//                 CntBit = 0;                                                      
//                 DataAds = 0;                                                     
//                 CntClk = 0;                                                      
//             }
//             break;
        	
// 				//"Setting CLK output to high"		
//         case 2:     
//             CntClk++;
//             if(CntClk >= 10) //If the duration of the CLK pulse has passed
// 						{
//                 CntClk = 0;
// 								HAL_GPIO_WritePin (GPIOA,GPIO_PIN_5,GPIO_PIN_SET); //clk spi
//                 StM_Ads++;
//             }
//             break;		

//         case 4: 
// 						StM_Ads = 1;					
// 						Average_Weight = float_filter(DataAds); //function float_filter
// 						Average_Weight += 0x800000;
// 						rx_Buffer2[0] = Average_Weight>>24; //MSB
// 						rx_Buffer2[1] = Average_Weight>>16;
// 						rx_Buffer2[2] = Average_Weight>>8;
// 						rx_Buffer2[3] = Average_Weight; //LSB	
				
// 						return 0xFF; //ready
// 	}
// 		    return 0x00; //busy
// }