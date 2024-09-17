
#include <stdio.h>
#include <string.h>

#include "parserNMEA.h"
#include "minmea.h"
#include "usart.h"
#include "stm32f4xx_hal_uart.h"
#include <stdlib.h> // ??? ??????????? ??????? malloc
// #include <malloc.h>

#define INDENT_SPACES "  "

extern struct minmea_sentence_zda frame_zda;
extern struct minmea_sentence_gll frame_gll;
extern uint8_t *massiveParser_ptr;

void clearMassiveForNMEA(UART_HandleTypeDef *huart, uint8_t *buffer, uint8_t sizeBuffer)
{
    // __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE | UART_IT_RXNE);
    memset(&buffer, '$', sizeBuffer);
    // __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE | UART_IT_RXNE);
}

uint8_t parserNMEA(uint8_t *sentenceNMEA)
{
    {
        switch (minmea_sentence_id((const char *)sentenceNMEA, false))
        {
            // case MINMEA_SENTENCE_RMC:
            // {
            //     struct minmea_sentence_rmc frame;
            //     if (minmea_parse_rmc(&frame, line))
            //     {
            //     }
            //     else
            //     {
            //     }
            // }
            // break;

            // case MINMEA_SENTENCE_GGA:
            // {
            //     struct minmea_sentence_gga frame;
            //     if (minmea_parse_gga(&frame, line))
            //     {
            //     }
            //     else
            //     {
            //     }
            // }
            // break;

            // case MINMEA_SENTENCE_GST:
            // {
            //     struct minmea_sentence_gst frame;
            //     if (minmea_parse_gst(&frame, line))
            //     {
            //     }
            //     else
            //     {
            //     }
            // }
            // break;

            // case MINMEA_SENTENCE_GSV:
            // {
            //     struct minmea_sentence_gsv frame;
            //     if (minmea_parse_gsv(&frame, line))
            //     {
            //     }
            //     else
            //     {
            //     }
            // }
            // break;
            // case MINMEA_SENTENCE_VTG:
            // {
            //     struct minmea_sentence_vtg frame;
            //     if (minmea_parse_vtg(&frame, line))
            //     {
            //     }
            //     else
            //     {
            //     }
            // }
            // break;

        case MINMEA_SENTENCE_ZDA:
        {
            if (minmea_parse_zda(&frame_zda, (const char *)sentenceNMEA))
            {
                // printf(INDENT_SPACES "$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n",
                //        frame.time.hours,
                //        frame.time.minutes,
                //        frame.time.seconds,
                //        frame.date.day,
                //        frame.date.month,
                //        frame.date.year,
                //        frame.hour_offset,
                //        frame.minute_offset);
                volatile uint8_t nop_tempZDA = 1;
            }
            else
            {
                volatile uint8_t nop_tempZDA = 1;
                // printf(INDENT_SPACES "$xxZDA sentence is not parsed\n");
                return 1;
            }
        }
        break;

        case MINMEA_SENTENCE_GLL:
        {
            if (minmea_parse_gll(&frame_gll, (const char *)sentenceNMEA))
            {
                volatile uint8_t nop_tempGLL = 1;
                // printf(INDENT_SPACES "$xxGLL: latitude: %d:%d, longitude:%d:%d, Time:%d:%d:%d:, status:%d, mode:%d \n",
                //        frame.latitude.value,
                //        frame.latitude.scale,
                //        frame.longitude.value,
                //        frame.longitude.scale,
                //        frame.time.hours,
                //        frame.time.minutes,
                //        frame.time.seconds,
                //        frame.status,
                //        frame.mode);
                // printf(INDENT_SPACES "$xxGLL floating point degree coordinates and speed: (%f,%f) %f\n",
                //        minmea_tocoord(&frame.latitude),
                //        minmea_tocoord(&frame.longitude));
            }
            else
            {
                volatile uint8_t nop_tempGLL = 1;
                // printf(INDENT_SPACES "$xxGLL sentence is not parsed\n");
                return 1;
            }
        }
        break;

        case MINMEA_INVALID:
        {
            // printf(INDENT_SPACES "$xxxxx sentence is not valid\n");
            return 1;
        }
        break;

        default:
        {
            // printf(INDENT_SPACES "$xxxxx sentence is not parsed\n");
            return 1;
        }
        break;
        }
    }

    return 0;
}
// 0- end of message
uint8_t readData(const uint8_t *massiveSourceData_ptr)
{
    const uint8_t sizeEndOfSentence = 2;
    uint8_t *lastSymbolData_ptr = (uint8_t *)massiveSourceData_ptr;
    for (uint8_t counterMessages = 0; counterMessages < CNT_NMEA_SENTENCE; counterMessages++)
    {
        // check start data for parsing
        uint8_t *lastSymbolDataPast_ptr = lastSymbolData_ptr;
        uint8_t *bla_ptr = lastSymbolData_ptr;
        lastSymbolData_ptr = memchr((lastSymbolData_ptr + 1), '$', (size_t)NAX_SIZE_NMEA_SENTENCE); // ma

        if (lastSymbolData_ptr == NULL)
        {
            return 0;
        }

        // check size data for parsing
        size_t sentenceSize = (size_t)(lastSymbolData_ptr - lastSymbolDataPast_ptr - sizeEndOfSentence);

        // uint8_t *sentenceNMEA = (uint8_t *)malloc((unsigned)sentenceSize+1);
        // if (sentenceNMEA == NULL)
        //     return 2; // dopisat
        // massiveParser_ptr = sentenceNMEA;
        // memset(sentenceNMEA, 0, sentenceSize+1);
        // memcpy(sentenceNMEA, lastSymbolDataPast_ptr, sentenceSize);
        // // parsing
        // uint8_t succesparserNMEA = parserNMEA(&sentenceNMEA[0]);
        // free(sentenceNMEA);

        uint8_t sentenceNMEA[sentenceSize+1];
        massiveParser_ptr = sentenceNMEA; //for debugging
        memset(sentenceNMEA, 0, sentenceSize + 1);
        memcpy(sentenceNMEA, lastSymbolDataPast_ptr, sentenceSize);
        // parsing
        uint8_t succesparserNMEA = parserNMEA(&sentenceNMEA[0]);
    }
    return 1;
}
