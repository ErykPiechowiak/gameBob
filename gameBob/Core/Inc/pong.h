/*
 * Pong.h
 *
 *  Created on: Jan 22, 2025
 *      Author: Eryk
 */

#ifndef INC_PONG_H_
#define INC_PONG_H_

#include "stdint.h"
#include "user.h"
#include "stm32f1xx_hal.h"
#include "ugui_fonts.h"

#define PWM_FREQ 64000000
#define PWM_CHANNEL TIM_CHANNEL_1

typedef struct{
	uint16_t oldx1;
	uint16_t oldx2;
	uint16_t x1;
	uint16_t x2;
	uint16_t y;
	int16_t acc;
	uint16_t score;
}PLAYER;

typedef struct{
	int16_t oldx;
	int16_t oldy;
	int16_t x;
	int16_t y;
	uint16_t r;
	int16_t accx;
	int16_t accy;
} BALL ;


void initGame(TIM_HandleTypeDef *htimer);
void gameInput(USER_INPUT uInput);
void gameLogic();
static void updateDisplay();
static void updatePlayersPosition();
static void updatePlayerPosition(PLAYER *player, uint16_t xaxis, uint16_t yaxis);
static void updateBallPosition();
static void drawPlayer(PLAYER *player);
static void drawDashedLine(uint16_t x, uint16_t y, uint16_t space);
static void drawScore(uint16_t color);
static void buzzer(int action);
static void winAnimation(const PLAYER player);


/* BUZZER */
static int presForFrequency (int frequency);
static void playTone (int *tone, int *duration, int *pause, int size);
static void noTone (void);



#endif /* INC_PONG_H_ */
