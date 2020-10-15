// 台車の速度制御（P制御）

#include "mbed.h"
#include "Motor.h"
#include "QEI.h"

BusOut led(LED1, LED2, LED3, LED4);	//基板LED用IO設定
AnalogIn pen(AD0);									//ポテンショメータ用IO設定

Ticker pen_control;								//台車の速度制御用タイマー割り込み

Serial pc(USBTX, USBRX);						//デバッグ用シリアル通信

//モータ制御用オブジェクト
Motor motor_left(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM);		//左モータ
Motor motor_right(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_PWM);	//右モータ

//***************　台車の速度制御　ここから　***************//
int pen_val, goal_pen_val; 							//振子の目標値(AD値)

int flg = 0;										//振子の目標値とポテンショメータのAD値の差分格納用
float theta, dtheta, theta0, e, ed, ei, e0, x, dx, x0;
int duty_ratio;

void pen_control_handler(){
	if(flg == 0) {goal_pen_val = pen.read_u16()>>6;
								flg = 1;}	

	pen_val = pen.read_u16()>>6;					//ADCを通してポテンショメータのAD値を取得
	
	theta = (float)pen_val * 0.0056808;
	e =  (float)goal_pen_val * 0.0056808 - theta;
	ed = (e - e0) / 0.08;
	ei += e * 0.08;
	e0 = e;
	
	if(ei > 10000) ei = 10000;
	if(ei < -10000) ei = -10000;

	duty_ratio = -(int)((e * -1.3 + ei * -0.1 + ed * -3.1)*100);

	if (duty_ratio > 100) duty_ratio = 100;
	else if (duty_ratio < -100) duty_ratio = -100;

	if ((duty_ratio < 1) && (duty_ratio > -1)) duty_ratio = 0;
	
	motor_left = duty_ratio;
	motor_right = duty_ratio;
}
//***************　台車の速度制御　ここまで　***************//

//***************　main関数　ここから　***************//
int main() {

	//モータの最大速度を設定
	motor_left.setMaxRatio(1.0);
	motor_right.setMaxRatio(1.0);
	
	pen_control.attach(&pen_control_handler, 0.001);		//台車の速度制御用のタイマー関数を設定
	
	led = 1;		//LEDの値を設定　動作確認用
	
	wait(1.0);		//なんとなく1秒待つ
	
	while(1) {		//無限ループ
		printf("pen:%d %d, %d \r\n", pen_val, duty_ratio, goal_pen_val);
		wait(0.08);
	}
}

