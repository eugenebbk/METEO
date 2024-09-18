
#ifndef METEOSTATION_H_
#define METEOSTATION_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "formCMD.h"

typedef struct
{
    uint8_t sensTemperature;
    uint8_t sensTemperaturePressureHumidity;
    uint8_t sensAnemometer;
} errorMeteostation_t;

//21b
typedef struct
{
    uint8_t  stateHeater;
    int16_t  Tmperature_Heater;
    int16_t  Tmperature_Air;
    uint16_t PressureAtmosphere;
    uint16_t Humidity;
    int16_t  SpeedAir_oX;
    int16_t  SpeedAir_oY;
    uint16_t SpeedAir_Result;
    int16_t  DirectAir;
    int16_t  SpeedUltrasound_oX;
    int16_t  SpeedUltrasound_oY;
} dataMeteostation_t;

#endif /* METEOSTATION_H_ */