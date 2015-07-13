/*
 * bmm500_util.h
 *
 *  Created on: Oct 2, 2014
 *      Author: Aytac
 */

#ifndef STM32F10X_UTIL_H_
#define STM32F10X_UTIL_H_

// INCLUDES
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// DEFINES

#define STRING_BUF_SIZE		50	//Maximum size of the string
#define DECIMAL_PART_SIZE	4

#define CONSTANT_PI			3.141592654


/**
 * Global String Buffer
 */
char string_buffer[STRING_BUF_SIZE];


/**
 * Number of decimal places for float
 * factor = 100 -> 2 decimal places, formatting -> "%d.%02d"
 * factor = 1000 -> 3 decimal places, formatting -> "%d.%03d"
 * and so on
 */
#define STRING_FLOAT_FACTOR 1000		//1000 = 3 decimal places
#define STRING_FLOAT_FORMAT	"%d.04d"	// formatting

/**
 * GLOBAL FUNCTIONS
 */
// String manipulation functions
//char* intToString(uint16_t int_number);
char* util_floatToString(float float_number);
//void doubleToString(double data);
//float stringToFloat(const char* str);
//double stringToDouble(const char* str);
//int16_t stringToInt(const char* str);
//void copySubString(const char* str, uint16_t startIndex, uint16_t length);
//void copyLeftSubString(const char* str, uint16_t length);
//void copyRightSubString(const char* str, uint16_t length);
//char* stringCopy(char* destination, char* source);
//uint16_t messageLength(char *serial_number, char *sequence_number, char* transmission_reason, char *temp, char *humidity, char *co_ppm, char *co2_ppm);
//bool stringFind(char* source, char* key);
//bool isNumeric(char inputChar);
//
//// Mathematical Operations
int16_t util_exponention(int16_t base, int16_t exponent);


void util_doubleToString(double source, char* destination, uint8_t destination_size);


/** Mathematical Methods */
float util_arctan(float const number);


int16_t util_powerInt(int16_t base, uint16_t exponent);
float util_powerDouble(float base, uint16_t exponent);



#endif /* STM32F10X_UTIL_H_ */
