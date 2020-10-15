// ��Ԃ̑��x����iP����j

#include "mbed.h"
#include "Motor.h"
#include "QEI.h"

BusOut led(LED1, LED2, LED3, LED4);	//���LED�pIO�ݒ�
AnalogIn pen(AD0);									//�|�e���V�����[�^�pIO�ݒ�

Ticker pen_control;								//��Ԃ̑��x����p�^�C�}�[���荞��

Serial pc(USBTX, USBRX);						//�f�o�b�O�p�V���A���ʐM

//���[�^����p�I�u�W�F�N�g
Motor motor_left(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM);		//�����[�^
Motor motor_right(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_PWM);	//�E���[�^

//***************�@��Ԃ̑��x����@��������@***************//
int pen_val, goal_pen_val; 							//�U�q�̖ڕW�l(AD�l)

int flg = 0;										//�U�q�̖ڕW�l�ƃ|�e���V�����[�^��AD�l�̍����i�[�p
float theta, dtheta, theta0, e, ed, ei, e0, x, dx, x0;
int duty_ratio;

void pen_control_handler(){
	if(flg == 0) {goal_pen_val = pen.read_u16()>>6;
								flg = 1;}	

	pen_val = pen.read_u16()>>6;					//ADC��ʂ��ă|�e���V�����[�^��AD�l���擾
	
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
//***************�@��Ԃ̑��x����@�����܂Ł@***************//

//***************�@main�֐��@��������@***************//
int main() {

	//���[�^�̍ő呬�x��ݒ�
	motor_left.setMaxRatio(1.0);
	motor_right.setMaxRatio(1.0);
	
	pen_control.attach(&pen_control_handler, 0.001);		//��Ԃ̑��x����p�̃^�C�}�[�֐���ݒ�
	
	led = 1;		//LED�̒l��ݒ�@����m�F�p
	
	wait(1.0);		//�Ȃ�ƂȂ�1�b�҂�
	
	while(1) {		//�������[�v
		printf("pen:%d %d, %d \r\n", pen_val, duty_ratio, goal_pen_val);
		wait(0.08);
	}
}

