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
#include "FreeRTOS.h"
#include "task.h"

#define PWM_FREQ 64000000
#define PWM_CHANNEL TIM_CHANNEL_1

typedef struct{
	char name[8];
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

enum Screen{
	MAIN_MENU_SCREEN,
	GAME_SCREEN,
	SETTINGS_SCREEN
};

enum GamePause{
	NONE,
	PLAYER1,
	PLAYER2,
	GAME_OVER
};

enum ActiveMenuOption{
	START_GAME,
	SETTINGS,
	EXIT_GAME
};

enum SettingsMenuOption{
	DIFFICULTY,
	GAME_MODE
};

enum Difficulty{
	EASY,
	NORMAL,
	HARD
};

enum GameMode{
	VERSUS,
	COOPERATION
};

void initGame(TaskHandle_t h_parent_task,TIM_HandleTypeDef *htimer, USER_INPUT uInput);
void gameInput(USER_INPUT uInput);
void gameLogic();
static void reInitGame();
static void updateDisplay();
static void updatePlayersPosition();
static void updatePlayerPosition(PLAYER *player, uint16_t xaxis, uint16_t yaxis);
static void updateBallPosition();
static void updateBallAngle();
static void drawPlayer(PLAYER *player);
static void drawDashedLine(uint16_t x, uint16_t y, uint16_t space);
static void drawScore(uint16_t color);
static void buzzer(int action);
static void winAnimation(PLAYER *player);
static void clearDisplay();
static void drawMainMenu();
static void drawSettingsMenu();


/* BUZZER */
static int presForFrequency (int frequency);
static void playTone (int *tone, int *duration, int *pause, int size);
static void noTone (void);



#endif /* INC_PONG_H_ */
