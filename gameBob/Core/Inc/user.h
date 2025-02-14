/*
 * user.h
 *
 *  Created on: Jan 20, 2025
 *      Author: Eryk
 */

#ifndef INC_USER_H_
#define INC_USER_H_





typedef struct{
	uint16_t leftXAxis;
	uint16_t leftYAxis;
	uint8_t leftAnalogKey;

	uint16_t rightXAxis;
	uint16_t rightYAxis;
	uint8_t rightAnalogKey;

	uint8_t keyLeft;
	uint8_t keyRight;
	uint8_t keyUp;
	uint8_t keyDown;
	uint8_t keyA;
	uint8_t keyB;

	uint16_t seed; //get noise value from flaoting adc input

} USER_INPUT;


#endif /* INC_USER_H_ */
