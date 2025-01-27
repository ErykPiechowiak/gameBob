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

static PLAYER player1, player2;
static BALL ball;
static USER_INPUT uInput;
static uint32_t seed = 12342;
static uint8_t game_pause = 0;


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
void initGame(){
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
		}
		else{
			player1.score++;
		}

		ball.x = LCD_HEIGHT/2;
		ball.y = LCD_WIDTH/2;

		game_pause = 1;

	}
	else if(ball.x-ball.r <= 0 || ball.x+ball.r >= LCD_WIDTH){
		ball.accx *= -1;
		ball.x += ball.accx;
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

static void drawScore(){
	char score[1];
	sprintf(score,"%lu",player1.score);
	LCD_PutStr(LCD_WIDTH/2, (LCD_HEIGHT/2)+(LCD_HEIGHT/4), score, FONT_16X26, C_WHITE, C_BLACK);
//	UG_PutString(LCD_WIDTH/2, (LCD_HEIGHT/2)+(LCD_HEIGHT/4), score);
	sprintf(score,"%lu",player2.score);
	LCD_PutStr(LCD_WIDTH/2, (LCD_HEIGHT/2)-(LCD_HEIGHT/4), score, FONT_16X26, C_WHITE, C_BLACK);
//	UG_PutString(LCD_WIDTH/2, (LCD_HEIGHT/2)-(LCD_HEIGHT/4), score);
}

void gameLogic(){
	if(!game_pause){
		updatePlayersPosition();
		updateBallPosition();
		updateDisplay();
	}
	else{
		drawScore();
	}
}

