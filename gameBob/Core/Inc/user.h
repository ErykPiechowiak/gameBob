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

} USER_INPUT;


#endif /* INC_USER_H_ */
