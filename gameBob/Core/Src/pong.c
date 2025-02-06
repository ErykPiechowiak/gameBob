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

static PLAYER player1, player2;
static BALL ball;
static USER_INPUT uInput;
static uint32_t seed = 12342;
static uint8_t game_pause = 0;
TIM_HandleTypeDef *htim;
uint32_t pwm_freq;

/*
void initGame(PLAYER *player1, PLAYER *player2, BALL *ball){
	player1->x1 = LCD_WIDTH /2 -30;
	player1->x2 = LCD_WIDTH /2 +30;
	player1->oldx1 = 0;
	player1->oldx2 = 0;
	player1->y = 1;
	player1->acc = 0;

	player2->x1 = LCD_WIDTH /2 -30;
	player2->x2 = LCD_WIDTH /2 +30;
	player2->y = LCD_HEIGHT;
	player2->acc = 0;

	ball->x = LCD_HEIGHT/2;
	ball->y = LCD_WIDTH/2;
	ball->r = 5;


}
*/
void initGame(TIM_HandleTypeDef *htimer){
	player1.x1 = LCD_WIDTH /2 -30;
	player1.x2 = LCD_WIDTH /2 +30;
	player1.oldx1 = 0;
	player1.oldx2 = 0;
	player1.y = LCD_HEIGHT;
	player1.acc = 0;
	player1.score = 0;

	player2.x1 = LCD_WIDTH /2 -30;
	player2.x2 = LCD_WIDTH /2 +30;
	player2.y = 2;
	player2.acc = 0;
	player2.score = 0;

	ball.x = LCD_HEIGHT/2;
	ball.y = LCD_WIDTH/2;
	ball.oldx = 0;
	ball.oldy = 0;
	ball.r = 5;
	ball.accy = 10;
	//ball.accx = 0;
	srand(seed);
	ball.accx = rand()%7;

	htim = htimer;

	//presForFrequency(10);
	__HAL_TIM_SET_PRESCALER(htim, presForFrequency(500));

	/* INIT DISPLAY */
	UG_FillScreen(C_BLACK);
	//drawDashedLine(0, LCD_HEIGHT/2, 10);
	UG_Update();

}

void gameInput(USER_INPUT input){
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
	/*
	if(uInput.leftXAxis < 1800 || uInput.leftXAxis > 2200){
		player1.acc = (uInput.leftXAxis - 2000)/100;
		if(player1.x1+player1.acc > 0 && player1.x2+player1.acc < LCD_WIDTH){
			player1.oldx1 = player1.x1;
			player1.oldx2 = player1.x2;
			player1.x1+=player1.acc;
			player1.x2+=player1.acc;
		}
		else{
			if(player1.x1+player1.acc < 0){
				player1.acc = 0 - player1.x1;
			}
			else{
				player1.acc = LCD_WIDTH - player1.x2;
			}
			player1.oldx1 = player1.x1;
			player1.oldx2 = player1.x2;
			player1.x1+=player1.acc;
			player1.x2+=player1.acc;
		}
	}
	*/
}

static void updateBallPosition(){
	ball.oldx = ball.x;
	ball.oldy = ball.y;
	ball.y += ball.accy;
	ball.x += ball.accx;

	buzzer(0);

	// Player 1 check boundaries
	if(ball.y+ball.r >= player1.y && (ball.x+ball.r >= player1.x1 && ball.x-ball.r <= player1.x2)){
		//ball.y = player1.y;
		ball.accy *= -1;
		ball.y += ball.accy;
		ball.accx = (ball.x - (player1.x2-30))/2;
		//ball.r = ball.r;
	}
	// Player 2 check boundaries
	else if(ball.y-ball.r <= player2.y && (ball.x+ball.r >= player2.x1 && ball.x-ball.r <= player2.x2)){
		//ball.y = player2.y;
		ball.accy *= -1;
		ball.y += ball.accy;
		ball.accx = (ball.x - (player2.x2-30))/2;
	}
	else if(ball.y+ball.r > LCD_HEIGHT || ball.y - ball.r < 0){
		if(ball.y+ball.r > LCD_HEIGHT){
			player2.score++;
			game_pause = 2;
		}
		else{
			player1.score++;
			game_pause = 1;
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


	/* DRAW PLAYER 1 */
	/*
	if(player1.acc>0){
		UG_DrawLine(player1.x1-player1.acc, player1.y-1, player1.x1-1, player1.y-1, C_BLACK);
		UG_DrawLine(player1.x1-player1.acc, player1.y, player1.x1-1, player1.y, C_BLACK);
	}
	else{
		UG_DrawLine(player1.x2+1, player1.y-1, player1.x2-player1.acc, player1.y-1, C_BLACK);
		UG_DrawLine(player1.x2+1, player1.y, player1.x2-player1.acc, player1.y, C_BLACK);
	}
	UG_DrawLine(player1.x1, player1.y-1, player1.x2, player1.y-1, C_WHITE);
	UG_DrawLine(player1.x1, player1.y, player1.x2, player1.y, C_WHITE);
	*/
	/* DRAW PLAYER 2 */




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
	sprintf(score,"%" PRIu16,player1.score);
	LCD_PutStr(LCD_WIDTH/2, (LCD_HEIGHT/2)+(LCD_HEIGHT/4), score, FONT_16X26, color, C_BLACK);
//	UG_PutString(LCD_WIDTH/2, (LCD_HEIGHT/2)+(LCD_HEIGHT/4), score);
	sprintf(score,"%" PRIu16,player2.score);
	LCD_PutStr(LCD_WIDTH/2, (LCD_HEIGHT/2)-(LCD_HEIGHT/4), score, FONT_16X26, color, C_BLACK);
//	UG_PutString(LCD_WIDTH/2, (LCD_HEIGHT/2)-(LCD_HEIGHT/4), score);
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

static void winAnimation(const PLAYER player){
	UG_DrawCircle(10, 10, 5, C_BLUE);
	UG_DrawCircle(LCD_WIDTH-10, LCD_HEIGHT-10, 5, C_RED);
	if(player.y < LCD_HEIGHT){
		uint16_t head_x = LCD_WIDTH/2;
		uint16_t head_y = (LCD_HEIGHT/2)/2;
		uint16_t head_r = 5;

		uint16_t core_length = 20;

		uint16_t left_hand_x1 = head_x - 20;
		uint16_t left_hand_x2 = head_x;
		uint16_t left_hand_y1 = head_y-head_r-core_length/2;
		uint16_t left_hand_y2 = head_y-head_r-5;

		uint16_t right_hand_x1 = head_x;
		uint16_t right_hand_x2 = head_x + 20;
		uint16_t right_hand_y1 = head_y-head_r-5;
		uint16_t right_hand_y2 = head_y-head_r-core_length/2;

		uint16_t left_leg_x1 = left_hand_x1;
		uint16_t left_leg_x2 = left_hand_x2;
		uint16_t left_leg_y1 = left_hand_y1-core_length-10;
		uint16_t left_leg_y2 = left_hand_y2 -core_length;

		uint16_t right_leg_x1 = right_hand_x1;
		uint16_t right_leg_x2 = right_hand_x2;
		uint16_t right_leg_y1 = right_hand_y1-core_length;
		uint16_t right_leg_y2 = right_hand_y2 -core_length-10;

		UG_DrawCircle(head_x, head_y, head_r, C_WHITE); //head
		UG_DrawLine(head_x, head_y-head_r, head_x, head_y-head_r-core_length, C_WHITE); //core
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


	}
}

void gameLogic(){
	if(!game_pause){
		updatePlayersPosition();
		updateBallPosition();
		updateDisplay();
	}
	else{
		drawScore(C_WHITE);
		if((player1.score >= 5 || player2.score >= 5) && game_pause != 4){
			winAnimation(player2);
			game_pause = 4;
		}
		if((game_pause == 2 && uInput.leftAnalogKey == GPIO_PIN_RESET) || (game_pause == 1 && uInput.rightAnalogKey == GPIO_PIN_RESET)){
			game_pause = 0;
			drawScore(C_BLACK);
		}
	}
}

