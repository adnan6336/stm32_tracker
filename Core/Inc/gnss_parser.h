/*
 * gnss_parser.h
 *
 *  Created on: Dec 29, 2022
 *      Author: Muhammad Adnan
 */

#ifndef INC_GNSS_PARSER_H_
#define INC_GNSS_PARSER_H_



#endif /* INC_GNSS_PARSER_H_ */
/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"



uint8_t* nmea_parser(char *nmeaResponse,uint8_t responseLenght,uint8_t *checkSum,uint8_t *rCheckSum);
