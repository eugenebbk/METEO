#ifndef DS18B20_INIT_H
#define DS18B20_INIT_H

#define numbSensorsDS18B20 5

void initDS18B20(DS18B20 *temperatureSensor, DS18B20_Status *errorDS18B20, uint8_t numbSensors);
void requestToCalculationTemperature(DS18B20 *temperatureSensor, DS18B20_Status *errorDS18B20, uint8_t numbSensors);
void receiveTemperature(DS18B20 *temperatureSensor, DS18B20_Status *errorDS18B20, uint8_t numbSensors);

#endif // #ifndef DS18B20_INIT_H
