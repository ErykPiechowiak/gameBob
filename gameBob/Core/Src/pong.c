/*
 * pong.c
 *
 *  Created on: Jan 22, 2025
 *      Author: Eryk
 */

#include "pong.h"
#include "lcd.h"
#include "user.h"
#include "stdio.h"
#include "pitches.h"
#include "ugui_fonts.h"
#include <inttypes.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* GAME VARIABLES */
static PLAYER player1, player2;
static BALL ball;
static USER_INPUT uInput;
static USER_INPUT uInputOld;
static uint32_t seed;
static uint8_t first_ball;
static int16_t ball_speed = 10;
static int16_t ball_speed_slow = 5;
static uint16_t coop_score = 0;



/* MAIN MENU */
//static uint8_t game_pause = 0; //1 -> player 1 scored; 2 -> player
static enum GamePause game_pause = NONE;
static enum Screen screen = MAIN_MENU_SCREEN;
static enum ActiveMenuOption menu_option = START_GAME;
static enum SettingsMenuOption settings_option = DIFFICULTY;
static enum Difficulty difficulty = NORMAL;
static enum GameMode game_mode = VERSUS;
uint8_t option_changed_flag = 1;
static uint8_t game_start = 0;


/* HANDLERS ETC */
TIM_HandleTypeDef *htim;
uint32_t pwm_freq;
TaskHandle_t h_main_menu_task;


void initGame(TaskHandle_t h_parent_task, TIM_HandleTypeDef *htimer, USER_INPUT uInput){
	player1.x1 = LCD_WIDTH /2 -30;
	player1.x2 = LCD_WIDTH /2 +30;
	player1.oldx1 = 0;
	player1.oldx2 = 0;
	player1.y = LCD_HEIGHT;
	player1.acc = 0;
	player1.score = 0;
	strcpy(player1.name, "Player1");

	player2.x1 = LCD_WIDTH /2 -30;
	player2.x2 = LCD_WIDTH /2 +30;
	player2.y = 2;
	player2.acc = 0;
	player2.score = 0;
	strcpy(player2.name, "Player2");

	ball.x = LCD_HEIGHT/2;
	ball.y = LCD_WIDTH/2;
	ball.oldx = 0;
	ball.oldy = 0;
	ball.r = 5;
	ball.accy = ball_speed;
	//ball.accx = 0;
	srand(uInput.seed);
	first_ball =1;
	updateBallAngle();//rand()%7;

	h_main_menu_task = h_parent_task;
	htim = htimer;

	//presForFrequency(10);
	__HAL_TIM_SET_PRESCALER(htim, presForFrequency(500));

	/* INIT DISPLAY */
	UG_FillScreen(C_BLACK);
	//drawDashedLine(0, LCD_HEIGHT/2, 10);
	UG_Update();

}

static void reInitGame(){
	player1.x1 = LCD_WIDTH /2 -30;
	player1.x2 = LCD_WIDTH /2 +30;
	player1.oldx1 = 0;
	player1.oldx2 = 0;
	player1.y = LCD_HEIGHT;
	player1.acc = 0;
	player1.score = 0;
	strcpy(player1.name, "Player1");

	player2.x1 = LCD_WIDTH /2 -30;
	player2.x2 = LCD_WIDTH /2 +30;
	player2.y = 2;
	player2.acc = 0;
	player2.score = 0;
	strcpy(player2.name, "Player2");

	ball.x = LCD_HEIGHT/2;
	ball.y = LCD_WIDTH/2;
	ball.oldx = 0;
	ball.oldy = 0;
	ball.r = 5;

	coop_score = 0;

	switch(difficulty){
	case(EASY):
		ball_speed = 5;
		ball.accy = 5;
		break;

	case(NORMAL):
		ball_speed = 10;
		ball.accy = 10;
		break;
	case(HARD):
		ball_speed = 15;
		ball.accy = 15;
		break;
	default:
		ball_speed = 10;
		ball.accy = 10;
	}

	first_ball = 1;
	updateBallAngle();
	__HAL_TIM_SET_PRESCALER(htim, presForFrequency(500));

	game_pause = NONE;
	/* INIT DISPLAY */
	UG_FillScreen(C_BLACK);
	//drawDashedLine(0, LCD_HEIGHT/2, 10);
	UG_Update();
}

void gameInput(USER_INPUT input){
	uInputOld = uInput;
	uInput = input;
}


static void updatePlayerPosition(PLAYER *player, uint16_t xaxis, uint16_t yaxis){

	if(xaxis < 1800 || xaxis > 2200){
		player->acc = (xaxis - 2000)/100;
		if(player->x1+player->acc > 0 && player->x2+player->acc < LCD_WIDTH){
			player->oldx1 = player->x1;
			player->oldx2 = player->x2;
			player->x1+=player->acc;
			player->x2+=player->acc;
		}
		else{
			if(player->x1+player->acc < 0){
				player->acc = 0 - player->x1;
			}
			else{
				player->acc = LCD_WIDTH - player->x2;
			}
			player->oldx1 = player->x1;
			player->oldx2 = player->x2;
			player->x1+=player->acc;
			player->x2+=player->acc;
		}
	}
}


static void updatePlayersPosition(){

	updatePlayerPosition(&player1, uInput.leftXAxis, uInput.leftYAxis);
	updatePlayerPosition(&player2, uInput.rightXAxis, uInput.rightYAxis);

}

static void updateBallAngle(){
	int16_t random_value = rand()%7;
	ball.accx = random_value;
	random_value = rand()%2;
	ball.accx = random_value == 0 ? ball.accx : ball.accx *-1;
}

static void updateBallPosition(){
	if(first_ball)
	{
		ball.accy= ball.accy > 0 ? ball_speed_slow : ball_speed_slow *-1;
	}

	ball.oldx = ball.x;
	ball.oldy = ball.y;
	ball.y += ball.accy;
	ball.x += ball.accx;

	buzzer(0);

	// Player 1 check boundaries
	if(ball.y+ball.r >= player1.y && (ball.x+ball.r >= player1.x1 && ball.x-ball.r <= player1.x2)){
		//ball.y = player1.y;
		ball.accy= ball.accy > 0 ? ball_speed : ball_speed*-1;
		ball.accy *= -1;
		ball.y += ball.accy;
		ball.accx = (ball.x - (player1.x2-30))/2;
		first_ball = 0;

		if(game_mode == COOPERATION){
			coop_score++;
			/* add little accx to avoid staying still and scoring */
			if(ball.accx == 0){
				int16_t random_value = rand()%3;
				int16_t temp_random = rand()%2;
				ball.accx = temp_random == 0 ? random_value : random_value*-1;
			}
		}
		//ball.r = ball.r;
	}
	// Player 2 check boundaries
	else if(ball.y-ball.r <= player2.y && (ball.x+ball.r >= player2.x1 && ball.x-ball.r <= player2.x2)){
		//ball.y = player2.y;
		ball.accy= ball.accy > 0 ? ball_speed : ball_speed*-1;
		ball.accy *= -1;
		ball.y += ball.accy;
		ball.accx = (ball.x - (player2.x2-30))/2;
		first_ball = 0;

		if(game_mode == COOPERATION){
			coop_score++;
			/* add little accx to avoid staying still */
			if(ball.accx == 0){
				int16_t random_value = rand()%3;
				int16_t temp_random = rand()%2;
				ball.accx = temp_random == 0 ? random_value : random_value*-1;
			}
		}
	}
	else if(ball.y+ball.r > LCD_HEIGHT || ball.y - ball.r < 0){
		if(ball.y+ball.r > LCD_HEIGHT){
			first_ball = 1;
			player2.score++;
			game_pause = PLAYER2;
		}
		else{
			first_ball = 1;
			player1.score++;
			game_pause = PLAYER1;
		}
		if(game_mode == COOPERATION){
			game_pause = GAME_OVER;
		}

		ball.x = LCD_HEIGHT/2;
		ball.y = LCD_WIDTH/2;


	}
	else if(ball.x-ball.r <= 0 || ball.x+ball.r >= LCD_WIDTH){
		ball.accx *= -1;
		ball.x += ball.accx;
		buzzer(1);
	}
	else if(LCD_HEIGHT/2 > ball.y-ball.r && LCD_HEIGHT/2 < ball.y+ball.r){
		//drawDashedLine(0, LCD_HEIGHT/2, 10);
	}



}

static void drawPlayer(PLAYER *player){
	if(player->acc>0){
		UG_DrawLine(player->x1-player->acc, player->y-2, player->x1-1, player->y-2, C_BLACK);
		UG_DrawLine(player->x1-player->acc, player->y-1, player->x1-1, player->y-1, C_BLACK);
		UG_DrawLine(player->x1-player->acc, player->y, player->x1-1, player->y, C_BLACK);
	}
	else{
		UG_DrawLine(player->x2+1, player->y-2, player->x2-player->acc, player->y-2, C_BLACK);
		UG_DrawLine(player->x2+1, player->y-1, player->x2-player->acc, player->y-1, C_BLACK);
		UG_DrawLine(player->x2+1, player->y, player->x2-player->acc, player->y, C_BLACK);
	}
	UG_DrawLine(player->x1, player->y-2, player->x2, player->y-2, C_WHITE);
	UG_DrawLine(player->x1, player->y-1, player->x2, player->y-1, C_WHITE);
	UG_DrawLine(player->x1, player->y, player->x2, player->y, C_WHITE);
}

static void updateDisplay(){

	/* DRAW BALL */
	UG_FillCircle(ball.oldx, ball.oldy, ball.r, C_BLACK);
	UG_FillCircle(ball.x, ball.y, ball.r, C_WHITE);

	/* DRAW PLAYERS */
	drawPlayer(&player1);
	drawPlayer(&player2);

	UG_Update();
}


static void drawDashedLine(uint16_t x, uint16_t y, uint16_t space){

	uint16_t tempx1 = x;
	uint16_t tempy1 = y;
	uint16_t tempx2 = x + space;
	uint16_t tempy2 = tempy1;
	while(tempx2 < LCD_WIDTH){
		LCD_DrawLine(tempx1, tempy1, tempx2, tempy2, C_WHITE);
		tempx1 = tempx2 + space;
		tempx2 = tempx1 + space;
	}
}



static void drawScore(uint16_t color){
	char score[2];
	if(game_mode == VERSUS){
		sprintf(score,"%" PRIu16,player1.score);
		LCD_PutStr(LCD_WIDTH/2, (LCD_HEIGHT/2)+(LCD_HEIGHT/4), score, FONT_16X26, color, C_BLACK);
		sprintf(score,"%" PRIu16,player2.score);
		LCD_PutStr(LCD_WIDTH/2, (LCD_HEIGHT/2)-(LCD_HEIGHT/4), score, FONT_16X26, color, C_BLACK);
	}
	else{
		char text[30];
		sprintf(text,"GAME OVER!");
		LCD_PutStr(LCD_WIDTH/4, (LCD_HEIGHT/2)-(LCD_HEIGHT/4), text, FONT_16X26, C_RED, C_BLACK);
		sprintf(text,"SCORE: %" PRIu16,coop_score);
		LCD_PutStr(LCD_WIDTH/4, (LCD_HEIGHT/2)-13, text, FONT_16X26, C_GREEN, C_BLACK);
		sprintf(text,"Press B key to continue");
		LCD_PutStr((LCD_WIDTH/2)-80, (LCD_HEIGHT/2)+30, text, FONT_7X12, C_WHITE,C_BLACK);
	}

}


static int presForFrequency(int frequency)
{
	if (frequency == 0) return 0;
	return ((PWM_FREQ/(1000*frequency))-1);  // 1 is added in the register
}

static void playTone (int *tone, int *duration, int *pause, int size)
{
	int tempo = 160;

	for (int i=0; i<size; i++)
	{
		int pres = presForFrequency(tone[i]*3);  // calculate prescaler
		int dur = 1000/(duration[i]);  // calculate duration
		int pauseBetweenTones = 0;
		if (pause != NULL) pauseBetweenTones = pause[i] - duration[i];

		__HAL_TIM_SET_PRESCALER(htim, pres);
		HAL_Delay(dur);   // how long the tone will play
		noTone();  // pause
		HAL_Delay(pauseBetweenTones);  // no tone for this duration
	}
}

static void noTone (void)
{
	__HAL_TIM_SET_PRESCALER(htim, 0);
}

static void buzzer(int action){
	switch(action){
	case 1:
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
		break;
	case 2:
		HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
		break;
	default:
		HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
	}

}

static void winAnimation(PLAYER *player){
	/* DRAW STICKMAN */
	UG_DrawCircle(10, 10, 5, C_BLUE);
	UG_DrawCircle(LCD_WIDTH-10, LCD_HEIGHT-10, 5, C_RED);
	uint16_t head_x,head_y, head_r,core_length = 0;
	uint16_t left_hand_x1,left_hand_x2,left_hand_y1,left_hand_y2,right_hand_x1,right_hand_x2,right_hand_y1,right_hand_y2 = 0;
	uint16_t left_leg_x1, left_leg_x2, left_leg_y1,left_leg_y2, right_leg_x1,right_leg_x2,right_leg_y1,right_leg_y2 = 0;

	if(player->y < LCD_HEIGHT){

		head_x = LCD_WIDTH/2;
		head_y = (LCD_HEIGHT/2)/2;
		head_r = 5;

		core_length = 20;

		left_hand_x1 = head_x - 20;
		left_hand_x2 = head_x;
		left_hand_y1 = head_y-head_r-core_length/2;
		left_hand_y2 = head_y-head_r-5;

		right_hand_x1 = head_x;
		right_hand_x2 = head_x + 20;
		right_hand_y1 = head_y-head_r-5;
		right_hand_y2 = head_y-head_r-core_length/2;

		left_leg_x1 = left_hand_x1;
		left_leg_x2 = left_hand_x2;
		left_leg_y1 = left_hand_y1-core_length-10;
		left_leg_y2 = left_hand_y2 -core_length;

		right_leg_x1 = right_hand_x1;
		right_leg_x2 = right_hand_x2;
		right_leg_y1 = right_hand_y1-core_length;
		right_leg_y2 = right_hand_y2 -core_length-10;
	}
	else{
		head_x = LCD_WIDTH/2;
		head_y = LCD_HEIGHT/2 + (LCD_HEIGHT/2)/2;
		head_r = 5;

		core_length = 20;

		left_hand_x1 = head_x - 20;
		left_hand_x2 = head_x;
		left_hand_y1 = head_y+head_r+core_length/2;
		left_hand_y2 = head_y+head_r+5;

		right_hand_x1 = head_x;
		right_hand_x2 = head_x + 20;
		right_hand_y1 = head_y+head_r+5;
		right_hand_y2 = head_y+head_r+core_length/2;

		left_leg_x1 = left_hand_x1;
		left_leg_x2 = left_hand_x2;
		left_leg_y1 = left_hand_y1+core_length+10;
		left_leg_y2 = left_hand_y2 +core_length;

		right_leg_x1 = right_hand_x1;
		right_leg_x2 = right_hand_x2;
		right_leg_y1 = right_hand_y1+core_length;
		right_leg_y2 = right_hand_y2 +core_length+10;
	}

	UG_DrawCircle(head_x, head_y, head_r, C_WHITE); //head
	if(player->y < LCD_HEIGHT){
		UG_DrawLine(head_x, head_y-head_r, head_x, head_y-head_r-core_length-5, C_WHITE); //core
	}
	else{
		UG_DrawLine(head_x, head_y+head_r, head_x, head_y+head_r+core_length+5, C_WHITE); //core
	}
	UG_DrawLine(left_hand_x1, left_hand_y1, left_hand_x2, left_hand_y2, C_WHITE); //left hand
	UG_DrawLine(right_hand_x1, right_hand_y1, right_hand_x2, right_hand_y2, C_WHITE); //right hand
	UG_DrawLine(left_leg_x1, left_leg_y1, left_leg_x2, left_leg_y2, C_WHITE); //left leg
	UG_DrawLine(right_leg_x1, right_leg_y1, right_leg_x2, right_leg_y2, C_WHITE); //right leg

	int STmelody[] = {
			  NOTE_C5, NOTE_GS4, NOTE_A4, NOTE_C5, NOTE_D5, NOTE_F5, NOTE_F5,
			  NOTE_A5, NOTE_AS5, NOTE_A5, NOTE_FS5, NOTE_D5, NOTE_A5,
			  NOTE_A5, NOTE_A5, NOTE_G5, NOTE_A5, NOTE_C6, NOTE_A5,
			  NOTE_C5, NOTE_A5, NOTE_F5
	};
	int STnoteDurations[] = {
			  6, 6, 6, 6, 6, 6, 2,
			  6, 6, 6, 6, 6, 2,
			  6, 6, 6, 6, 6, 3,
			  6, 6, 1
	};
	int pres = 0;
	int dur = 0;
	int pauseBetweenTones = 0;
	int size = sizeof(STmelody)/sizeof(STmelody[0]);
	int *pause = NULL;

	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);

	for(int i=0; i<size;i++){
		if(i%1==0){
			UG_DrawLine(left_hand_x1, left_hand_y1, left_hand_x2, left_hand_y2, C_BLACK); //left hand
			UG_DrawLine(right_hand_x1, right_hand_y1, right_hand_x2, right_hand_y2, C_BLACK); //right hand
		}
		if(i%2==0){
			left_hand_y1 +=10;
			right_hand_y2 +=10;
		}
		else{
			left_hand_y1 -=10;
			right_hand_y2 -=10;
		}
		if(i%1==0){
			UG_DrawLine(left_hand_x1, left_hand_y1, left_hand_x2, left_hand_y2, C_WHITE); //left hand
			UG_DrawLine(right_hand_x1, right_hand_y1, right_hand_x2, right_hand_y2, C_WHITE); //right hand
		}

		/* PLAY MUSIC */
		pres = presForFrequency(STmelody[i]*3);  // calculate prescaler
		dur = 1000/(STnoteDurations[i]);  // calculate duration
		if (pause != NULL) pauseBetweenTones = pause[i] - STnoteDurations[i];
		__HAL_TIM_SET_PRESCALER(htim, pres);
		HAL_Delay(dur);   // how long the tone will play
		noTone();  // pause
		HAL_Delay(pauseBetweenTones);  // no tone for this duration

	}
	/* STOP BUZZER */
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
	__HAL_TIM_SET_PRESCALER(htim, presForFrequency(500)); //go back to nominal freq (side screen hit sound)

	/* DISPLAY GAME OVER SCREEN */
	char text[30];
	sprintf(text,"%s won!",player->name);
	LCD_PutStr((LCD_WIDTH/2)-66, LCD_HEIGHT/2, text, FONT_12X16, C_WHITE,C_BLACK);
	sprintf(text,"Press B key to continue");
	LCD_PutStr((LCD_WIDTH/2)-80, (LCD_HEIGHT/2)+30, text, FONT_7X12, C_WHITE,C_BLACK);


}

static void clearDisplay(){
	UG_FillScreen(C_BLACK);
	UG_Update();
}


/* MAIN MENU */
static void drawMainMenu(){
	/* Mini version of the game on top */
	UG_DrawLine(0,((LCD_HEIGHT/2)/2)-20,0,((LCD_HEIGHT/2)/2)+20,C_WHITE);
	UG_DrawLine(1,((LCD_HEIGHT/2)/2)-20,1,((LCD_HEIGHT/2)/2)+20,C_WHITE);
	UG_DrawLine(LCD_WIDTH,((LCD_HEIGHT/2)/2)-40,LCD_WIDTH,((LCD_HEIGHT/2)/2),C_WHITE);
	UG_DrawLine(LCD_WIDTH-1,((LCD_HEIGHT/2)/2)-40,LCD_WIDTH-1,((LCD_HEIGHT/2)/2),C_WHITE);
	UG_FillCircle(LCD_WIDTH/4, LCD_HEIGHT/4, 5, C_WHITE);
	UG_DrawLine(0,LCD_HEIGHT/2-10,LCD_WIDTH,LCD_HEIGHT/2-10,C_WHITE);


	/* OPTIONS */

	if(uInput.keyB == GPIO_PIN_RESET && uInput.keyB != uInputOld.keyB){
		if(menu_option == START_GAME){
			screen = GAME_SCREEN;
			clearDisplay();
			reInitGame();
			return;
		}
		else if(menu_option == SETTINGS){
			screen = SETTINGS_SCREEN;
			clearDisplay();
			return;
		}
		else if(menu_option == EXIT_GAME){
			clearDisplay();
			vTaskResume(h_main_menu_task);
			vTaskDelete(NULL);
		}
	}

	if(uInput.keyDown == GPIO_PIN_RESET && uInput.keyDown != uInputOld.keyDown){
		if(menu_option < EXIT_GAME){
			menu_option++;
		}
	}
	else if(uInput.keyUp == GPIO_PIN_RESET && uInput.keyUp != uInputOld.keyUp){
		if(menu_option > START_GAME){
			menu_option--;
		}
	}

	uint16_t option_colors[3] = {C_WHITE, C_WHITE, C_WHITE};
	for(int i=0; i<3; i++){
		menu_option == i ? option_colors[i] = C_YELLOW : C_WHITE;
	}

	char buffer[20];
	sprintf(buffer, "START GAME");
	LCD_PutStr(LCD_WIDTH/4, LCD_HEIGHT/2+10, buffer, FONT_12X16, option_colors[0],C_BLACK);
	sprintf(buffer, "OPTIONS");
	LCD_PutStr(LCD_WIDTH/4, LCD_HEIGHT/2+30, buffer, FONT_12X16, option_colors[1], C_BLACK);
	sprintf(buffer, "EXIT GAME");
	LCD_PutStr(LCD_WIDTH/4, LCD_HEIGHT/2+50, buffer, FONT_12X16, option_colors[2], C_BLACK);
}

/* SETTINGS MENU */
static void drawSettingsMenu(){

	if(uInput.keyDown == GPIO_PIN_RESET && uInput.keyDown != uInputOld.keyDown){
		if(settings_option < GAME_MODE){
			settings_option++;
			option_changed_flag = 1;
		}
	}
	else if(uInput.keyUp == GPIO_PIN_RESET && uInput.keyUp != uInputOld.keyUp){
		if(settings_option > DIFFICULTY){
			settings_option--;
			option_changed_flag = 1;
		}
	}

	if(uInput.keyB == GPIO_PIN_RESET && uInput.keyB != uInputOld.keyB){
		if(settings_option == DIFFICULTY){
			option_changed_flag = 1;
			if(difficulty == HARD){
				difficulty = EASY;
			}
			else{
				difficulty++;
			}
		}
		else if(settings_option == GAME_MODE){
			option_changed_flag = 1;
			if(game_mode == COOPERATION)
				game_mode = VERSUS;
			else
				game_mode++;
		}
	}

	else if(uInput.keyA == GPIO_PIN_RESET && uInput.keyA != uInputOld.keyA){
		option_changed_flag = 1;
		screen = MAIN_MENU_SCREEN;
		clearDisplay();
		return;
	}

if(option_changed_flag){
		uint16_t option_colors[2] = {C_WHITE, C_WHITE};
		for(int i=0; i<2; i++){
			settings_option == i ? option_colors[i] = C_YELLOW : C_WHITE;
		}

		char buffer[30];
		sprintf(buffer, "SETTINGS: ");
		LCD_PutStr(LCD_WIDTH/4, 10, buffer, FONT_12X20, C_WHITE,C_BLACK);
		sprintf(buffer, "DIFFICULTY: ");
		LCD_PutStr(LCD_WIDTH/4, LCD_HEIGHT/4+20, buffer, FONT_12X16, option_colors[0],C_BLACK);
		/* DRAW SELECTED DIFFICULTY */
		uint16_t difficulty_color = C_YELLOW;
		switch(difficulty){
		case(EASY):
				sprintf(buffer, "EASY");
			difficulty_color = C_GREEN;
			break;
		case(NORMAL):
				sprintf(buffer, "NORMAL");
			difficulty_color = C_BLUE;
			break;
		case(HARD):
				sprintf(buffer, "HARD");
			difficulty_color = C_RED;
			break;
		default:
			sprintf(buffer,"NOMRAL");
			difficulty_color = C_BLUE;
		}
		char temp_buffer[20];
		sprintf(temp_buffer, "SSSSSSSSSSSS");
		LCD_PutStr(LCD_WIDTH/4+24, LCD_HEIGHT/4+50, temp_buffer, FONT_12X16, C_BLACK,C_BLACK); //ERASER
		LCD_PutStr(LCD_WIDTH/4+24, LCD_HEIGHT/4+50, buffer, FONT_12X16, difficulty_color,C_BLACK);

		sprintf(buffer, "GAME MODE: ");
		LCD_PutStr(LCD_WIDTH/4, LCD_HEIGHT/4+80, buffer, FONT_12X16, option_colors[1], C_BLACK);
		/* DRAW SELECTED GAME MODE */
		uint16_t game_mode_color = C_RED;
		switch(game_mode){
		case(VERSUS):
			sprintf(buffer, "VERSUS");
			game_mode_color = C_RED;
			break;
		case(COOPERATION):
			sprintf(buffer, "CO-OP");
			game_mode_color = C_GREEN;
			break;
		default:
			sprintf(buffer, "VERSUS");
			game_mode_color = C_RED;
		}
		LCD_PutStr(LCD_WIDTH/4+24, LCD_HEIGHT/4+110, temp_buffer, FONT_12X16, C_BLACK,C_BLACK); //ERASER
		LCD_PutStr(LCD_WIDTH/4+24, LCD_HEIGHT/4+110, buffer, FONT_12X16, game_mode_color, C_BLACK);

		option_changed_flag = 0; //reset flag
}

}

void gameLogic(){
	switch(screen){

	case MAIN_MENU_SCREEN:
		drawMainMenu();
		break;
	case GAME_SCREEN:
		if (game_pause == NONE){
			/* GAME LOGIC */
			updatePlayersPosition();
			updateBallPosition();
			updateDisplay();
		}
		else if(game_pause == PLAYER1 || game_pause == PLAYER2){
			drawScore(C_WHITE);
			if((player1.score >= 5 || player2.score >= 5) && game_pause != GAME_OVER){
				clearDisplay();
				PLAYER *winner;
				winner = player1.score >= 5 ? &player1 : &player2;
				winAnimation(winner);
				game_pause = GAME_OVER;
			}
			if((game_pause == PLAYER2 && uInput.leftAnalogKey == GPIO_PIN_RESET) || (game_pause == PLAYER1 && uInput.rightAnalogKey == GPIO_PIN_RESET)){
				game_pause = NONE;
				updateBallAngle();; //Set random x-angle before resuming game
				drawScore(C_BLACK);
			}

		}
		else{
			if(game_mode == COOPERATION){
				drawScore(C_WHITE);
			}
			if(uInput.keyB == GPIO_PIN_RESET){
				clearDisplay();
				screen = MAIN_MENU_SCREEN;
			}
		}
		break;
	case SETTINGS_SCREEN:
		drawSettingsMenu();
		break;
	default:
		drawMainMenu();
	}

}

