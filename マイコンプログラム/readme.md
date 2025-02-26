#include <cmath>
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "sken_module.hpp"
using namespace std;

int16_t x, y, theta;

const float MOTOR_PID_GAIN[4] = {1.5,0.1,0.1,20};
float motor_target_rps[4] = {1,-1,5,-5};

Uart uart;
uint8_t data[6],data2[4],data3[3],data4[2],data6;
SkenMdd mdd;
Motor motor1;
Motor motor2;
Motor motor3;
Motor motor4;
Gpio sw;
Encoder encoder;
Encoder_data encoder_data;
RcPwm rc_pwm;
RcPwm rc_pwm2;
Pid pid;

int sw_i;
int Square;
int Triangle;
int Up;
int Circle;
int Cross;
int Down;
int out;
int R1;
int L1;
int in;
int mama;

constexpr uint32_t CAN_STDID = SENSOR_0;
constexpr int SERVO_MIN_PWM[8] = {   0,   0,   0,   0,   0,   0,   0,   0};
constexpr int SERVO_MAX_PWM[8] = { 100, 100, 100, 100, 100, 100, 100, 100};

float COMMAND_MODE[4] = { 0, 0, 0, 0 };
float ENCODER_CONFIG[4] = { 8192, 8192, 8192, 8192 };
float DIAMETER_CONFIG[4] = { 101, 917, 0, 0 };
float M1_gain[4] = {10.0,0,0,20};
float M2_gain[4] = {10.0,0,0,20};
float M3_gain[4] = {10.0,0,0,20};
float M4_gain[4] = {10.0,0,0,20};

CanData received_raw;
uint8_t received_data[8];
RcPwm servo[8];
Gpio limit[2];

void main_interrupt(void) {
	for (int i = 0; i < 8; ++i) {
			servo[i].turn(SERVO_MIN_PWM[i] + (SERVO_MAX_PWM[i]-SERVO_MIN_PWM[i])/100*received_data[i]);
		}
	if (module_receiver[SENSOR_2].get_limit(0)) {}
	if (module_receiver[SENSOR_3].get_limit(1)) {}
	if (module_receiver[SENSOR_4].get_limit(2)) {}
	if (module_receiver[SENSOR_5].get_limit(3)) {}
	module_receiver[SENSOR_0].get_enc(&x, &y, &theta);
	module_receiver[SENSOR_1].get_enc(&x, &y, &theta);
}
int main(void){
	sken_system.init();
	sken_module_init();

	uart.init(A9,A10,SERIAL1,115200);

        mdd.tcp(OMNI4_MODE,MOTOR_PID_GAIN,10,2000);

	sken_system.addTimerInterruptFunc(main_interrupt, 0, 1);
	mdd.init(MDD_0,A12,A11,CAN_1);

	limit[0].init(A6, INPUT_PULLUP);
	limit[1].init(A7, INPUT_PULLUP);
	limit[2].init(B10, INPUT_PULLUP);
	limit[3].init(B9, INPUT_PULLUP);
	servo[0].init(B6, TIMER4, CH1);
	servo[1].init(B7, TIMER4, CH2);
	encoder.init(A0,A1,TIMER5);
	motor3.init(Apin,A6,TIMER1,CH1);
	motor3.init(Apin,A8,TIMER1,CH1);
        motor2.init(Apin,A11,TIMER1,CH1);
	motor1.init(Apin,A13,TIMER1,CH1);

	  mdd.init(C10, C11, SERIAL3);  //MDDピン設定
	  mdd.tcp(MOTOR_COMMAND_MODE_SELECT, COMMAND_MODE, 10, 2000);
	  mdd.tcp(ENCODER_RESOLUTION_CONFIG, ENCODER_CONFIG, 10, 2000);
	  mdd.tcp(ROBOT_DIAMETER_CONFIG, DIAMETER_CONFIG, 10, 2000);
	  mdd.tcp(M1_PID_GAIN_CONFIG, M1_gain, 10, 2000);
	  mdd.tcp(M2_PID_GAIN_CONFIG, M2_gain, 10, 2000);
	  mdd.tcp(M3_PID_GAIN_CONFIG, M3_gain, 10, 2000);
	  mdd.tcp(M4_PID_GAIN_CONFIG, M4_gain, 10, 2000);

	mdd.init(A13,A12,SERIAL2);
	mdd.tcp (OMNI4_MODE,M1_gain,10,2000);
	rc_pwm.init(A5,TIMER2,CH1);

	uart.startDmaRead(data,6);
	while(true) {
		for(int i=0;i<6;i++){
			if(data[i] == 0xa5 && data[(i+1)%8] == 0xa5){
				Square    = ((data[(i+2)%8])&0x01) ? true:false;
				Triangle     = ((data[(i+2)%8])&0x02) ? true:false;
				Up     = ((data[(i+2)%8])&0x04) ? true:false;
				Circle       = ((data[(i+2)%8])&0x08) ? true:false;
				Cross   = ((data[(i+2)%8])&0x10) ? true:false;
				Down    = ((data[(i+2)%8])&0x20) ? true:false;
				R1   = ((data[(i+2)%8])&0x40) ? true:false;
				L1       = ((data[(i+3)%8])&0x01) ? true:false;
			}
		sken_module_receive();
		rc_pwm.turn(45);
		}
	}
}
void box(void){
	sken_system.init();
	sken_system.startCanCommunicate(A12, A11, CAN_1);
	sken_system.addCanRceiveInterruptFunc(CAN_1, &received_raw);
	sken_system.addTimerInterruptFunc(main_interrupt, 0, 1);

	while(true) {
			if(Square){
				motor1.write(100);
			}
			if(!limit[1].read()){
					motor1.write(0);
			}
	}
}
void corn(void)
{
    pid.setGain(1,0,0,1);
    while(true){
    	encoder.interrupt(&encoder_data);
    	out = pid.control(100,encoder_data.deg,1);
    	if(Triangle){
    		motor2.write(-100);
    	}
    	if(!limit[3].read()){
    		motor2.write(0);
    		motor3.write(out);
    	}
    }
}
void borl(void){
	while(true){
		if(Up)
		rc_pwm.turn(-90);
		motor4.write(-100);
		motor2.write(-100);
		if(!limit[2].read()){
			motor2.write(0);
		}
	}
}
void boxes(void)
{
	sken_system.init();
	sken_system.startCanCommunicate(A12, A11, CAN_1);
	sken_system.addCanRceiveInterruptFunc(CAN_1, &received_raw);
	sken_system.addTimerInterruptFunc(main_interrupt, 0, 1);

	while(true) {
			if(Square){
				motor1.write(100);
			}
			if(!limit[1].read()){
					motor1.write(0);
			}
	}
}
void corns(void)
{
	sken_system.init();
	sken_system.startCanCommunicate(A12, A11, CAN_1);
	sken_system.addCanRceiveInterruptFunc(CAN_1, &received_raw);
	sken_system.addTimerInterruptFunc(main_interrupt, 0, 1);
	while(true){
    	encoder.interrupt(&encoder_data);
    	in = pid.control(100,encoder_data.deg,1);
    	if(Cross){
    		motor2.write(-100);
    		motor3.write(in);
    	}
    	if(!limit[2].read()){
    	motor2.write(0);
    	}
	}
}
void borls(void){
	while(true){
		if(Up)
		rc_pwm.turn(-90);
		motor4.write(-100);
		motor2.write(-100);
		if(!limit[3].read()){
			motor2.write(0);
		}
	}
}
void Cornya(void)
{
	sken_system.init();
		sken_system.startCanCommunicate(A12, A11, CAN_1);
		sken_system.addCanRceiveInterruptFunc(CAN_1, &received_raw);
		sken_system.addTimerInterruptFunc(main_interrupt, 0, 1);
		while(true){
	    	encoder.interrupt(&encoder_data);
	    	mama = pid.control(-100,encoder_data.deg,1);
	    	if(R1){
	    		motor2.write(-100);
	    	}
	    	if(!limit[3].read()){
	    		motor2.write(0);
	    		motor3.write(mama);
	    	}
		}
}
void borlya(void){
	while(true){
		if(L1)
		motor2.write(-100);
		rc_pwm.turn(90);
		motor4.write(100);
		if(!limit[3].read()){
				motor2.write(0);
		}
	}
}
