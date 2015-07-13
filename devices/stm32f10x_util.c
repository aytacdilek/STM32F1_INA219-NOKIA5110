/*
 * File: 	stm32f10x_util.c
 * Datum:	02.10.2014
 * Version:	1.0
 * Author:	Aytac Dilek
 * email:	adilek@bimetri.com
 * web:		www.bimetri.com
 * cpu:		efm32gg332f1024
 * IDE:		Simplicity Studio
 * GCC:		GNU-ARM
 *
 * NOTES:	for the conversion of integers, the function "sprintf ()" are used
 *
 * Example for Integer :
 * 		int16_t data=-123;
 * 		sprintf(STRING_BUF,"%d",data);
 *
 * 		Output as a decimal :
 * 			8/16bit unsigned 	= %d or %i 	: uint8_t, uint16_t
 * 			8/16bit signed   	= %d or %i 	: int8_t, int16_t
 * 			32bit unsigned   	= %u         	: unsigned int
 * 		Output as a hex number:
 * 			8/16bit unsigned 	= %x or %X 	: uint8_t, uint16_t
 * 			32bit unsigned   	= %x or %X 	: unsigned int
 * 		Output as a string:
 * 								= %s     	: char[], unsigned char[]
 * 		Output as a character:
 * 								= %c      	: char, unsigned char
 * 		additions:
 * 			% 5d: edition of 5 points
 * 			% 05d: edition of 5 digits with leading zeros
 * 	Example for float:
 * 		float data=-123.4567;
 * 		UB_String_FloatToDezStr(data);
 *
 */

/**
 * INCLUDES
 */
#include "stm32f10x_util.h"



///**
// * @Definition:	Converts a integer number to a string
// * @Parameters:	integer number
// */
//char* intToString(uint16_t int_number){
//	/* Find the length of the integer number */
//	int length = 0;
//	int temp_number = int_number;
//	do {
//		temp_number = temp_number / 10;
//		length++;
//	} while (temp_number != 0);
//
//	/* Create a char array */
//	char *ret_string;
//	ret_string = (char *)malloc(length);
//	if(ret_string == NULL)
//		exit(1);
//
//	/* Copy integer decimals into to the char array */
//	uint16_t i;
//	temp_number = int_number;
//	for(i=0 ; i<length ; i++){
//		ret_string[i] = (0x30 + (temp_number / util_exponention(10, length-1-i)));
//		temp_number = temp_number  % util_exponention(10, length-1-i);
//	}
//	ret_string[i] = '\0';
//
//	return ret_string;
//}



/*******************************************************************************
* Function Name  : 	floatToString
* Description    : 	Converts a floating point number to a string as number decimal
* 					result is then in STRING_BUFFER
* Input          : 	float must be between -32767 and 32767
* Output         : 	None
* Return         : 	None
* Attention		 : 	Number = 123.4567 is to string "123.457"
*******************************************************************************/
char* util_floatToString(float float_number){
	uint16_t exponent;
	uint16_t fraction;

	exponent = (uint16_t) float_number;
	fraction = ((uint16_t)(float_number * 1000)) % 1000;

	/* Find the length of the integer number */
	int length = 0;
	int temp_number = exponent;
	do {
		temp_number = temp_number / 10;
		length++;
	} while (temp_number != 0);

	/* Copy exponent part into to the char array */
	uint16_t i = length;
	temp_number = exponent;
	for(i=0 ; i<length ; i++){
		string_buffer[i] = (0x30 + (temp_number / util_exponention(10, length-1-i)));
		temp_number = temp_number  % util_exponention(10, length-1-i);
	}

	/* Place dot sign into array */
	string_buffer[length] = '.';

	/* Place fraction part into the char array */
	temp_number = fraction;
	for(i=0 ; i<3 ; i++){
		string_buffer[length+1+i] = (0x30 + (temp_number / util_exponention(10, 2-i)));
		temp_number = temp_number  % util_exponention(10, 2-i);
	}
	string_buffer[length+4] = '\0';

	return string_buffer;
}



///**
// * @Definition:	Converts a floating point number to a string as number decimal result is then in STRING_BUFFER
// * @Parameters:	float must be between -32767 and 32767
// */
//void doubleToString(double data){
//	int16_t exponent;
//	uint16_t fraction;
//	double rest;
//
//	exponent=(int16_t)(data);
//	if(data>=0) {
//		rest = data-(float)(exponent);
//	}
//	else {
//		rest = (float)(exponent)-data;
//	}
//	fraction = (uint16_t)(rest * (float)(STRING_FLOAT_FACTOR) + 0.5);
//
//	sprintf(string_buffer, STRING_FLOAT_FORMAT, exponent, fraction);
//}
//
//
//
///**
// * @Definition:	Converts a string to floating point number.
// * @Parameters:	C-string beginning with the representation of a floating-point number.
// * @Returns:	Converted floating point number
// */
//float stringToFloat(const char* str){
//	float data;
//	data = (float) atof(str);	// atof() converts a c string to double variable
//	return data;
//}
//
///**
// * @Definition:	Converts a string to floating point number.
// * @Parameters:	C-string beginning with the representation of a floating-point number.
// * @Returns:	Converted floating point number
// */
//double stringToDouble(const char* str){
//	double data;
//	data = atof(str);	// atof() converts a c string to double variable
//	return data;
//}
//
///**
// * @Definition:	Converts a string to floating point number.
// * @Parameters:	C-string beginning with the representation of a floating-point number.
// * @Returns:	Converted integer number
// */
//int16_t stringToInt(const char* str){
//	int16_t data;
//	data = atoi(str);	// atof() converts a c string to double variable
//	return data;
//}
//
///**
// * @Definition:	Copies a substring
// * @Parameters: str - source string which will be copied
// * 				startIndex - first character which will be copied
// * 				lentgh - length of the substring
// * @Returns:
// * @Example:	string "Hello people", 3, 6 becomes "lo peo"
// */
//void copySubString(const char* str, uint16_t startIndex, uint16_t length){
//	uint16_t i, m;
//	uint16_t cnt = 0;
//
//	if(length == 0)
//		return;
//	m = startIndex + length;
//	if(m > strlen(str))
//		m = strlen(str);
//	for(i = startIndex; i < m; i++){
//		string_buffer[cnt] = str[i];
//		cnt++;
//	}
//	string_buffer[cnt] = "\0";		// 0x00
//}
//
///**
// * @Definition:	Copies a substring from the left
// * @Parameters: str - source string which will be copied
// * 				lentgh - length of the substring
// * @Returns:
// * @Example:	string "Hello people", 3 becomes "Hel"
// */
//void copyLeftSubString(const char* str, uint16_t length){
//	uint16_t i, m, start;
//	uint16_t cnt = 0;
//
//	if(length == 0)
//		return;
//	if(length > strlen(str))
//		length = strlen(str);
//	strncpy(string_buffer, str, length);
//	string_buffer[cnt] = "\0";		// 0x00
//}
//
///**
// * @Definition:	Copies right part of a string
// * @Parameters: str - source string which will be copied
// * 				lentgh - length of the substring
// * @Returns:
// * @Example:	string "Hello people", 3 becomes "Hel"
// */
//void copyRightSubString(const char* str, uint16_t length){
//	uint16_t i, m, start;
//	uint16_t cnt = 0;
//
//	if(length == 0)
//		return;
//	m = strlen(str);
//	if(length > m)
//		length = m;
//	start = m - length;
//
//	for(i = start; i < m; i++){
//		string_buffer[cnt] = str[i];
//		cnt++;
//	}
//	string_buffer[cnt] = "\0";		// 0x00
//}
//
//char* stringCopy(char* destination, char *source){
//	uint8_t size, i;
//
//	size = strlen(source);
//
//	destination = (char*) malloc(size+1);
//	if(destination == NULL)
//		return NULL;
//
//	for(i = 0 ; i < size ; i++)
//	{
//		destination[i] = source[i];
//	}
//	destination[size] = '\0';
//
//	return destination;
//}
//
//uint16_t messageLength(char *serial_number, char *sequence_number, char* transmission_reason, char *temp, char *humidity, char *co_ppm, char *co2_ppm){
//	uint8_t length;
//	length = strlen(serial_number) + 1;
//	length = length + strlen(sequence_number) + 1;
//	length = length + strlen(transmission_reason) + 1;
//	length = length + strlen(temp) + 1;
//	length = length + strlen(humidity) + 1;
//	length = length + strlen(co_ppm) + 1;
//	length = length + strlen(co2_ppm);
//
//	return length;
//}
//
//bool stringFind(char* source, char* key){
////	int lengthKey;
////	int lengthSource;
////	int i, j;
////	int token = 0;
////
////	lengthKey = strlen(key);
////	lengthSource = strlen(source);
////
////
////	for(i=0; i<lengthKey; i++){
////		for(j=0; j<lengthSource; j++){
////			if(key[i] == source[j]){
////				token++;
////			}
////		}
////	}
////	if(token == lengthKey)
////		return true;
////	else
////		return false;
//
//	/* Second Method */
//	  int c, d, e, text_length, pattern_length, position = -1;
//
//	  text_length    = strlen(source);
//	  pattern_length = strlen(key);
//
//	  if (pattern_length > text_length) {
//		return false;
//	  }
//
//	  for (c = 0; c <= text_length - pattern_length; c++) {
//		position = e = c;
//
//		for (d = 0; d < pattern_length; d++) {
//		  if (key[d] == source[e]) {
//			e++;
//		  }
//		  else {
//			break;
//		  }
//		}
//		if (d == pattern_length) {
//		  return true;
//		}
//	  }
//
//return false;
//}
//
//bool isNumeric(char inputChar){
//	if((inputChar >= 48) && (inputChar <= 57))
//		return true;
//	else
//		return false;
//}



/*
 * @Definition:	Exponention operation
 * @Parameters:	base and exponent values
 * @Returns:	integer number
 */
int16_t util_exponention(int16_t base, int16_t exponent){
	int16_t ret_value = 1;
	uint8_t i;

	for(i=0 ; i<exponent ; i++){
		ret_value = ret_value * base;
	}

	return ret_value;
}







void util_doubleToString(double source, char* destination, uint8_t destination_size){
	uint8_t index = 0;
	uint16_t tenth_part;
	uint8_t tenth_size = 0;
	uint16_t decimal_part;
//	utin8_t decimal_size = 0;
	uint8_t i;
	double temp;

	/* Clear the elements of destination string */
	for (i = 0; i < destination_size; ++i) {
		destination[i] = 0;
	}

	/* If the number is lower than zero, place '-' sign at the beginning of the string */
	if(source < 0){
		destination[index] = '-';
		source *= -1;
		index++;
	}

	/* Calculate the size of the tenth part */
	tenth_part = (uint16_t) source;
	do {
		tenth_part /= 10;
		tenth_size++;
	} while (tenth_part != 0);

	/* Store the tenth part of the source double number */
	tenth_part = (uint16_t) source;
	for (i = 0; i < tenth_size; ++i) {
		destination[index] = 0x30 + (tenth_part / util_exponention(10, tenth_size - i - 1));
		tenth_part = tenth_part % util_exponention(10, tenth_size - i - 1);
		index++;
	}

	/* Store point sign to the string */
	destination[index] = '.';
	index++;

	/* Store the decimal part of the source double number */
	decimal_part = (uint16_t) source;
	temp = source - (float) decimal_part;
	decimal_part = (uint16_t) (temp * util_exponention(10, DECIMAL_PART_SIZE));

	for (i = 0; i < DECIMAL_PART_SIZE; ++i) {
		destination[index] = 0x30 + (decimal_part / util_exponention(10, DECIMAL_PART_SIZE - i - 1));
		decimal_part = decimal_part % util_exponention(10, DECIMAL_PART_SIZE - i - 1);
		index++;
	}
}



float util_powerDouble(float base, uint16_t exponent){
	float result = 1.00;
	uint8_t i;

	for (i = 0; i < exponent; ++i) {
		result = result * base;
	}

	return result;
}

int16_t util_powerInt(int16_t base, uint16_t exponent){
	int16_t result = 1;
	uint8_t i;

	for (i = 0; i < exponent; ++i) {
		result = result * base;
	}

	return result;
}


float util_arctan(float const number){
	float result = 0.00;
	float x1;
	float x2;
	uint8_t i;

	if(number <= 1.00){
		for (i = 0; i < 100; ++i) {
			x1 = (-1.00 * util_powerDouble(-1.00, i+1 % 2));
			x2 = (util_powerDouble(number, 2 * i + 1) / (2 * i + 1));
			result = result		+	x1	*	x2;
		}
	}
	else{
		result = CONSTANT_PI / 2 - util_arctan(1.00 / number);
	}

	return result;
}
