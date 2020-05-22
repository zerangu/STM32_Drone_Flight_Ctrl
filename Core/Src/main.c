/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h> // for pow function
#include <stdlib.h>// for abs function

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef pHeader;
CAN_RxHeaderTypeDef pRxHeader;


uint8_t aData[8] = {40, 41, 42, 43, 44, 45, 46, 47};
uint8_t rxData[8];
uint32_t pTxMailbox;
uint8_t error_code[8];

// 0x101 gneral control can1 rx
int16_t 		str_ang = 0;
uint8_t 		brk_ctrl = 0;
uint8_t 		thr_ctrl = 0;
uint8_t 		driving_mode = 0;
uint8_t 		target_gear = 0;
uint8_t 		shift_enable = 0;
uint8_t 		thr_enable = 0;
uint8_t 		brk_enable = 0;
uint8_t 		str_enable = 0;
int8_t          HWA_Diff_Ctrl = 0;
uint8_t         esp_mode = 0;

// value to store ADC
uint32_t		adc_reading[2];
float 			joystick_x = 0;
float           joystick_x_val;
float           x_mid_val;
float           x_max_val=0.30f;
float           x_min_val=0.30f;
int16_t         hwa_limit = 500;//units
int16_t         servo_out;
int16_t         servo_out_midpt = 1415;
int16_t         hwa_out = 0;
float           joystick_dead_band = 0.02;

float 			joystick_y = 0;
float 			joystick_y_val;
float 			y_mid_val;
float 			y_max_val = 0.30f;
float 			y_min_val = 0.30f;
int16_t 	    acc_limit = 500;
int16_t 		servo2_out;
int16_t         servo2_midpt = 1500;
int16_t 		servo2_out = 0;



//value for system time
uint16_t		sys_time_up;
uint32_t 		sys_time_ms;
uint32_t        current_time = 0;
uint32_t        hwa_out_zero_time = 0;
uint8_t         str_over_ride = 1;
uint32_t        received_0x101_time = 0;
uint8_t         receive_0x101_timeout = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

PUTCHAR_PROTOTYPE
{

  //HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return ch;
}
#endif

void Read_joystick(){
	joystick_x = (float) adc_reading[0]/4096;
    joystick_x_val = joystick_x - x_mid_val;
        if(joystick_x_val > 0) {
            if(joystick_x_val > x_max_val) {
                x_max_val = joystick_x_val;
            }
            joystick_x_val = joystick_x_val/x_max_val;
            if(joystick_x_val>1.0f) {
                joystick_x_val = 1.0f;
            }
            joystick_x_val = pow(joystick_x_val,1.5f);
        } else {
            if(-joystick_x_val > x_min_val) {
                x_min_val = -joystick_x_val;
            }
            joystick_x_val = joystick_x_val/x_min_val;
            if(joystick_x_val<-1.0f) {
                joystick_x_val = -1.0f;
            }
            joystick_x_val = -pow(-joystick_x_val,1.5f);
        }

        if (fabs(joystick_x_val) > joystick_dead_band) { //steering out put
            hwa_out = joystick_x_val*hwa_limit;
        } else {
            hwa_out = 0;
        }

        if (str_over_ride || receive_0x101_timeout){
            servo_out = servo_out_midpt+hwa_out;
        }else{
            if (str_enable && esp_mode){
                servo_out = servo_out_midpt+HWA_Diff_Ctrl*4;
            }else{
                servo_out = servo_out_midpt+hwa_out;
            }
        }
        htim3.Instance ->CCR1 = servo_out;

        joystick_y = (float) adc_reading[1]/4096;
        joystick_y_val = joystick_y - y_mid_val;
        if(joystick_y_val > 0) {
        	if(joystick_y_val > y_max_val) {
        		y_max_val = joystick_y_val;
            }
            joystick_y_val = joystick_y_val/y_max_val;
            if(joystick_y_val>1.0f) {
            	joystick_y_val = 1.0f;
            }
            joystick_y_val = pow(joystick_y_val,1.5f);
        } else {
        	if(-joystick_y_val > y_min_val) {
          		y_min_val = -joystick_y_val;
            }
            joystick_y_val = joystick_y_val/y_min_val;
            if(joystick_y_val<-1.0f) {
            	joystick_y_val = -1.0f;
            }
            joystick_y_val = -pow(-joystick_y_val,1.5f);
        }

        if (fabs(joystick_y_val) > joystick_dead_band) { //steering out put
          	servo2_out = servo2_midpt + joystick_y_val*acc_limit;
        } else {
          	servo2_out = servo2_midpt;
        }


        htim3.Instance ->CCR2 = servo2_out;

        //printf("adc0: %i adc1:%i\r\n",servo_out,servo2_out);
        //printf("sys_time_ms:%i\r\n",sys_time_ms);

}

void joystick_init(void){
	printf("calibrating middle point\n");
	//printf("adc_read: %i %i\r\n", adc_reading[0], adc_reading[1]);
	for(int i=0; i<100; i++) {
		joystick_x = (float) adc_reading[0]/4096;
		joystick_y = (float) adc_reading[1]/4096;
	    x_mid_val = x_mid_val*0.8f + joystick_x*0.2f;
	    y_mid_val = y_mid_val*0.8f + joystick_y*0.2f;
	    HAL_Delay(10);
	}
	printf("mid_x:%.4f mid_y:%.4f \n",x_mid_val,y_mid_val);
}


void CAN_BUS_init(void){
	// can bus filter setting
	pHeader.DLC = 8;
	pHeader.IDE = CAN_ID_STD;
	pHeader.RTR = CAN_RTR_DATA;
	pHeader.StdId = 0x321;

	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	sFilterConfig.FilterIdHigh=0x245<<5;
	sFilterConfig.FilterIdLow=0;
	sFilterConfig.FilterMaskIdHigh=0;
	sFilterConfig.FilterMaskIdLow=0;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;

	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
    HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_AddTxMessage(&hcan, &pHeader, aData, &pTxMailbox);
	  //while(HAL_CAN_IsTxMessagePending(&hcan, &pTxMailbox));
}

// PPM_IN SIGNAL READ find time between two rising edges
int32_t		ic_value1 = 0;
int32_t 	ic_value2 = 0;
int32_t 	ic_value_difference = 0;
uint16_t 	channel_pulse[8] = {0,0,0,0,0,0,0,0};
uint8_t		channel_number = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){ // if the source is channel 1

		ic_value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		ic_value_difference = ic_value2-ic_value1;
		ic_value1 = ic_value2;

		if (ic_value2 > ic_value1){
			ic_value_difference = ic_value2-ic_value1;
		}
		else if(ic_value2 < ic_value1){// you know
			ic_value_difference = ((0xffff-ic_value1)+ic_value2)+1;
		}
		else{
			//printf("PPM_error\n\r");
			Error_Handler();
		}
			// process captured time
		if (ic_value_difference > 6000){
			channel_number = 0;
			//printf("%i %i %i %i %i %i %i %i\r\n",channel_pulse[0],channel_pulse[1],channel_pulse[2],channel_pulse[3],channel_pulse[4],channel_pulse[5],channel_pulse[6],channel_pulse[7]);
			//printf("%i\r\n", channel_pulse[0]);
		}else if(ic_value_difference > 800){
			if(channel_number<9){
				channel_pulse[channel_number] = ic_value_difference;
				channel_number ++;
			}
		}else{
			channel_number ++;
		}
	}
}


// MPU6050IMU
int16_t 	Accel_X_RAW;
int16_t 	Accel_Y_RAW;
int16_t 	Accel_Z_RAW;
int16_t 	Gyro_X_RAW;
int16_t 	Gyro_Y_RAW;
int16_t 	Gyro_Z_RAW;
float 		Accel_X_CAL,Accel_Y_CAL,Accel_Z_CAL,Gyro_X_CAL,Gyro_Y_CAL,Gyro_Z_CAL;
float 		Ax,Ay,Az,Gx,Gy,Gz,pitch_a,roll_a,yaw_a,pitch_est,roll_est,yaw_est,pitch_ino,roll_ino,yaw_ino;
int			mpu6050_ini = 0;
float 		deg2rad = 0.0174532925199433;
float 		rad2deg = 57.2957795130823;
float		delta_time = 0.01;
float 		pitch_est_rad,roll_est_rad,yaw_est_rad,sin_roll,cos_roll,sin_pitch,cos_pitch,pitch_a_rad,roll_a_rad;
float 		Gz_rad, Gx_rad, Gy_rad;
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define CONFIG_REG 0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

void MPU6050_Read_Accel(void){
	uint8_t Rec_Data[6];
	//Read 6 byte of data starting from accel_xout_h register
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	Ax = (float)Accel_X_RAW/4096.0;
	Ay = (float)Accel_Y_RAW/4096.0;
	Az = (float)Accel_Z_RAW/4096.0;
}
void MPU6050_Read_Gyro(void){
	uint8_t Rec_Data[6];
	//Read 6 byte of data starting from accel_xout_h register
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	Gx = (float)Gyro_X_RAW/65.5;
	Gy = (float)Gyro_Y_RAW/65.5;
	Gz = (float)Gyro_Z_RAW/65.5;
}

void MPU6050_Init(void ){//MPU6050_START
	uint8_t check, Data;
	//check device ID WHO_AM_I
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
	if (check == 104){
		printf("mpu6050 check passed! \r\n");
		printf("reset mpu6050...\r\n");
		Data = 0x80;// power management register 0x6b we should write 0x80 to reset the sensor
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
		HAL_Delay(100);

		printf("set clock to 8khz and wake up mpu6050...\r\n");
		Data = 0;// power management register 0x6b we should write all 0's to wake the sensor up
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
		HAL_Delay(100);

		// set low pass filter
		printf("set low pass filter on mpu6050...\r\n");
		Data = 0x01; //Bandwidth(Hz) 0=260A_256G 1=184A_188G 2=94A_98G 3=44A_42G 4=21A_20G 5=10A_10G 6=5A_5G
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, 1000);

		// set data rate of 8khz by writing SMPLRT_DIV register
		printf("set sample rate to 8khz on mpu6050...\r\n");
		Data = 0x00; // 7=1khz  0=8khz
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		printf("set accelerometer to 8g on mpu6050...\r\n");
		// set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0, YA_ST=0, ZA_ST=0, FS_SEL=0 -> +/- 2g , 10 = 8g
		Data = 0x10;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		printf("set gyroscope to 500deg/sec on mpu6050...\r\n");
		// set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST,YG_ST,ZG_ST=0, FS_SEL=0 -> +/- 250 Deg/s , 8=500deg/s
		Data = 0x08;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);


		printf("calibrating IMU in 2...\r\n");
		HAL_Delay(1000);
		printf("calibrating IMU in 1...\r\n");
		HAL_Delay(1000);
		printf("calibrating IMU in 0...\r\n");
		printf("start calibration ... \r\n");

		for(int i=0; i<2000; i++){
			MPU6050_Read_Accel();
			MPU6050_Read_Gyro();
			Accel_X_CAL += Ax;
			Accel_Y_CAL += Ay;
			Accel_Z_CAL += Az;
			Gyro_X_CAL += Gx;
			Gyro_Y_CAL += Gy;
			Gyro_Z_CAL += Gz;
			HAL_Delay(1);
		}
		Accel_X_CAL /= 2000;
	    Accel_Y_CAL /= 2000;
	    Accel_Z_CAL /= 2000;
	    Gyro_X_CAL /= 2000;
	    Gyro_Y_CAL /= 2000;
	    Gyro_Z_CAL /= 2000;

	    printf("calibration finished! \r\n");
	    printf("ax:%f ay:%f az:%f gx:%f gy:%f gz:%f\r\n",Accel_X_CAL,Accel_Y_CAL,Accel_Z_CAL,Gyro_X_CAL,Gyro_Y_CAL,Gyro_Z_CAL);

//	    pitch_est = -atan2(Accel_X_CAL,Accel_Z_CAL)*rad2deg;
//	    roll_est = atan2(Accel_Y_CAL,Accel_Z_CAL)*rad2deg;
//	    pitch_est_rad = -atan2(Accel_X_CAL,Accel_Z_CAL);
//	    roll_est_rad = atan2(Accel_Y_CAL,Accel_Z_CAL);

	    pitch_est_rad = -atan(Accel_X_CAL/sqrt(Accel_Z_CAL*Accel_Z_CAL + Accel_Y_CAL*Accel_Y_CAL));
	    roll_est_rad = atan(Accel_Y_CAL/sqrt(Accel_Z_CAL*Accel_Z_CAL + Accel_X_CAL*Accel_X_CAL));

	    yaw_est_rad = 0;

	    pitch_est = pitch_est_rad*rad2deg;
	    roll_est = roll_est_rad*rad2deg;
	    yaw_est = yaw_est_rad*rad2deg;


	    printf("PE:%f PR:%f\r\n",pitch_est,roll_est);

		mpu6050_ini = 1;
	}else{
		printf("problem checking mpu6050 \r\n");
		mpu6050_ini = 0;
	}
}
// ducted drone output
int16_t		pitch_input,yaw_input,throttle_input,roll_input;
int16_t   	fin_left_out, fin_right_out, motor_left_out, motor_right_out;
int16_t		fin_left_trim = -25;
int16_t 	fin_right_trim = -25;

// gain for dual
/*
int16_t 	throttle_idel = 1100;
float 		gain_yaw_p = -1;
float 		gain_yaw_d = 1;
float 		gain_pitch_p = -2.5;
float 		gain_pitch_d = 2.5;
float 		gain_roll_p = 7.5;
float 		gain_roll_d = -1;
*/

// gain for quad
int16_t 	throttle_idel = 1200;
float 		gain_yaw_p = -3.0;
float 		gain_yaw_d = 1.25;
float 		gain_pitch_p = -1.20;
float 		gain_pitch_d = 1.25;
float 		gain_roll_p = 1.20;
float 		gain_roll_d = -1.25;

int16_t 	yaw_ctrl, pitch_ctrl, roll_ctrl;
float 		yaw_tar,pitch_tar,roll_tar;
float 		yaw_err,pitch_err,roll_err;

void Drone_Control_duel(void){//DRONE_CONTROL

	pitch_input = channel_pulse[2]-1500;
	yaw_input = -channel_pulse[3] + 1500;
	throttle_input = channel_pulse[0];
	roll_input = channel_pulse[1] - 1500;

	pitch_tar = (float) pitch_input/10;
	roll_tar = (float) roll_input/10;
	if (abs(yaw_input)>100)
		yaw_tar += (float) yaw_input/1000;
	if (yaw_tar < -180)
		yaw_tar += 360;
	if (yaw_tar > 180)
		yaw_tar -= 360;

	yaw_err = yaw_tar - yaw_est;
	// course difference
	yaw_err = fmod(yaw_err,360);
	if (yaw_err < -180)
		yaw_err += 360;

	if (yaw_err > 180)
		yaw_err -= 360;

	pitch_err = pitch_tar - pitch_est;
	roll_err = roll_tar - roll_est;


	yaw_ctrl = (int16_t) (Gz*gain_yaw_d + yaw_err*gain_yaw_p);
	pitch_ctrl = (int16_t) (pitch_err * gain_pitch_p + Gy*gain_pitch_d);
	roll_ctrl = (int16_t) (roll_err*gain_roll_p + Gx*gain_roll_d);

	fin_left_out = 1500 + pitch_ctrl - fin_left_trim + yaw_ctrl;
	fin_right_out = 1500 - pitch_ctrl + fin_right_trim + yaw_ctrl;

	if (fin_left_out>2000)
		fin_left_out = 2000;
	if (fin_left_out<1000)
		fin_left_out = 1000;
	if (fin_right_out > 2000)
		fin_right_out = 2000;
	if (fin_right_out < 1000)
		fin_right_out =1000;

	motor_left_out = throttle_input - roll_ctrl;
	motor_right_out = throttle_input + roll_ctrl;

	if (channel_pulse[4] > 1600){
		if (motor_left_out>2000)
			motor_left_out = 2000;
		if (motor_left_out<throttle_idel)
			motor_left_out = throttle_idel;
		if (motor_right_out > 2000)
			motor_right_out = 2000;
		if (motor_right_out < throttle_idel)
			motor_right_out =throttle_idel;
	}else{
		motor_left_out = 1000;
		motor_right_out =1000;
	}

	htim3.Instance ->CCR1 = fin_left_out;
	htim3.Instance ->CCR2 = fin_right_out;
	htim3.Instance ->CCR3 = motor_left_out;
	htim3.Instance ->CCR4 = motor_right_out;

	//printf("ML:%i,MR:%i\r\n",motor_left_out,motor_right_out);
	//printf("pt:%i pr:%i p%i\r\n", (int16_t)pitch_tar, (int16_t)pitch_err, (int16_t)pitch_est);
	//printf("yi:%i yt:%i ti:%i\r\n",yaw_input,(int16_t)yaw_tar,throttle_input);
	//printf("yt:%i y:%i ye:%i\r\n",(int16_t)yaw_tar,(int16_t)yaw_est,(int16_t)yaw_err);
	//printf("pt:%.2f rt:%.2f yt:%.2f\r\n", pitch_tar, roll_tar, yaw_tar);
	//printf("pi:%i ri:%i yi:%i\r\n", pitch_input, roll_input, yaw_input);
	//printf("yc:%i pc:%i rc:%i\r\n",yaw_ctrl, pitch_ctrl, roll_ctrl);
	//printf("ti:%i ri:%i\r\n",channel_pulse[0], channel_pulse[1]);
	//printf("pi:%i yi:%i Pi:%.2f Ro:%.2f Gy:%.2f Gx:%.2f Gz:%f\r\n",channel_pulse[2], channel_pulse[3],pitch_est,roll_est,Gy,Gx,Gz);
	//printf("pi:%i yi:%i\r\n",channel_pulse[2], channel_pulse[3]);
	//printf("Ro:%.2f Gy:%.2f Gx:%.2f Gy:%.2f Gz:%f\r\n",pitch_est,roll_est,Gy,Gx,Gz);
	//printf("Ro:%.2f Gy:%.2f\r\n",pitch_est,roll_est);
	//printf("Gx:%.2f Gy:%.2f Gz:%f\r\n",Gy,Gx,Gz);
	//printf("pi:%i yi:%i Pi:%.2f Ro:%.2f\r\n",channel_pulse[2], channel_pulse[3],pitch_est,roll_est);

	//printf("PE:%.2f RE:%.2f\r\n", pitch_est, roll_est);
	//htim3.Instance ->CCR3 = pitch_input;
	//htim3.Instance ->CCR4 = pitch_input;
}
int16_t   	motor_quad_1, motor_quad_2, motor_quad_3, motor_quad_4;

void Drone_Control_quad(void){//DRONE_CONTROL
	// FR SKY
	pitch_input = channel_pulse[2]-1500;
	yaw_input = -channel_pulse[3] + 1500;
	throttle_input = channel_pulse[0];
	roll_input = channel_pulse[1] - 1500;


	// FS
	//pitch_input = channel_pulse[1]-1500;
	//yaw_input = -channel_pulse[3] + 1500;
	//throttle_input = channel_pulse[2];
	//roll_input = channel_pulse[0] - 1500;

	pitch_tar = (float) pitch_input/10;
	roll_tar = (float) roll_input/10;
	if (abs(yaw_input)>100)
		yaw_tar += (float) yaw_input/1000;
	if (yaw_tar < -180)
		yaw_tar += 360;
	if (yaw_tar > 180)
		yaw_tar -= 360;

	yaw_err = yaw_tar - yaw_est;
	// course difference
	yaw_err = fmod(yaw_err,360);
	if (yaw_err < -180)
		yaw_err += 360;

	if (yaw_err > 180)
		yaw_err -= 360;

	pitch_err = pitch_tar - pitch_est;
	roll_err = roll_tar - roll_est;


	yaw_ctrl = (int16_t) (Gz*gain_yaw_d + yaw_err*gain_yaw_p);
	pitch_ctrl = (int16_t) (pitch_err * gain_pitch_p + Gy*gain_pitch_d);
	roll_ctrl = (int16_t) (roll_err*gain_roll_p + Gx*gain_roll_d);

	motor_quad_1 = throttle_input + pitch_ctrl + roll_ctrl - yaw_ctrl;
	motor_quad_2 = throttle_input + pitch_ctrl - roll_ctrl + yaw_ctrl;
	motor_quad_3 = throttle_input - pitch_ctrl - roll_ctrl - yaw_ctrl;
	motor_quad_4 = throttle_input - pitch_ctrl + roll_ctrl + yaw_ctrl;

	if (channel_pulse[4] > 1600){
		if (motor_quad_1>2000)
			motor_quad_1 = 2000;
		if (motor_quad_1<throttle_idel)
			motor_quad_1 = throttle_idel;
		if (motor_quad_2 > 2000)
			motor_quad_2 = 2000;
		if (motor_quad_2 < throttle_idel)
			motor_quad_2 =throttle_idel;
		if (motor_quad_3 > 2000)
			motor_quad_3 = 2000;
		if (motor_quad_3 < throttle_idel)
			motor_quad_3 =throttle_idel;
		if (motor_quad_4 > 2000)
			motor_quad_4 = 2000;
		if (motor_quad_4 < throttle_idel)
			motor_quad_4 =throttle_idel;
	}else{
		yaw_tar = yaw_est;
		motor_quad_1 = 1000;
		motor_quad_2 = 1000;
		motor_quad_3 = 1000;
		motor_quad_4 = 1000;
	}

	htim3.Instance ->CCR1 = motor_quad_1;
	htim3.Instance ->CCR2 = motor_quad_2;
	htim3.Instance ->CCR3 = motor_quad_3;
	htim3.Instance ->CCR4 = motor_quad_4;

	//printf("Ti:%i Ri:%i Pi:%i Yi:%i Sw%i\r\n", throttle_input, roll_input, pitch_input, yaw_input,channel_pulse[4]);

	//printf("ML:%i,MR:%i\r\n",motor_left_out,motor_right_out);
	//printf("pt:%i pr:%i p%i\r\n", (int16_t)pitch_tar, (int16_t)pitch_err, (int16_t)pitch_est);
	//printf("yi:%i yt:%i ti:%i\r\n",yaw_input,(int16_t)yaw_tar,throttle_input);
	//printf("yt:%i y:%i ye:%i\r\n",(int16_t)yaw_tar,(int16_t)yaw_est,(int16_t)yaw_err);
	//printf("pt:%.2f rt:%.2f yt:%.2f\r\n", pitch_tar, roll_tar, yaw_tar);
	//printf("pi:%i ri:%i yi:%i\r\n", pitch_input, roll_input, yaw_input);
	//printf("yc:%i pc:%i rc:%i\r\n",yaw_ctrl, pitch_ctrl, roll_ctrl);
	//printf("ti:%i ri:%i\r\n",channel_pulse[0], channel_pulse[1]);
	//printf("pi:%i yi:%i Pi:%.2f Ro:%.2f Gy:%.2f Gx:%.2f Gz:%f\r\n",channel_pulse[2], channel_pulse[3],pitch_est,roll_est,Gy,Gx,Gz);
	//printf("pi:%i yi:%i\r\n",channel_pulse[2], channel_pulse[3]);
	//printf("Ro:%.2f Gy:%.2f Gx:%.2f Gy:%.2f Gz:%f\r\n",pitch_est,roll_est,Gy,Gx,Gz);
	//printf("Ro:%.2f Gy:%.2f\r\n",pitch_est,roll_est);
	//printf("Gx:%.2f Gy:%.2f Gz:%f\r\n",Gy,Gx,Gz);
	//printf("pi:%i yi:%i Pi:%.2f Ro:%.2f\r\n",channel_pulse[2], channel_pulse[3],pitch_est,roll_est);

	//printf("PE:%.2f RE:%.2f\r\n", pitch_est, roll_est);
	//htim3.Instance ->CCR3 = pitch_input;
	//htim3.Instance ->CCR4 = pitch_input;
}

#define TFLUNA_ADDR 0x20
#define TFLUNA_DIST_LOW 0x00
#define TFLUNA_DIST_HIGH 0x01
#define TFLUNA_AMP_LOW 0x02
#define TFLUNA_AMP_HIGH 0x03
#define TFLUNA_TEMP_LOW 0x00
#define TFLUNA_TEMP_HIGH 0x00

uint8_t 	dist_low = 0;
uint8_t 	dist_high = 0;
uint16_t 	dist_tfluna = 0;
uint32_t 	dist_measure_devider = 50;
//uint16_t 	dist_tfluna_1 = 0;
float 		dist_spd = 0;
uint32_t 	prev_time_tf = 0;
uint32_t 	delta_time_tf = 0;
float	 	delta_dist = 0;
float 		dist_spd_lowpass = 0;
float 		dist_tfluna_f = 0;
float 		dist_tfluna_f1 = 0;
int			tf_luna_ini = 0;

void ReadTF_Luna_init(void){
	HAL_I2C_Mem_Read(&hi2c1, TFLUNA_ADDR, TFLUNA_DIST_LOW, 1, &dist_low, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, TFLUNA_ADDR, TFLUNA_DIST_HIGH, 1, &dist_high, 1, 1000);
	dist_tfluna = (uint16_t)dist_high<<8 | (dist_low);
	dist_tfluna_f1 = (float)dist_tfluna/100;
	if(dist_tfluna != 0)
		tf_luna_ini = 1;
	printf("TF_Luna initialization complete: %.2f \r\n", dist_tfluna_f1);
}

void ReadTF_Luna(void){
	delta_time_tf = sys_time_ms - prev_time_tf;
	HAL_I2C_Mem_Read(&hi2c1, TFLUNA_ADDR, TFLUNA_DIST_LOW, 1, &dist_low, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, TFLUNA_ADDR, TFLUNA_DIST_HIGH, 1, &dist_high, 1, 1000);
	dist_tfluna = (uint16_t)dist_high<<8 | (dist_low);
	dist_tfluna_f = (float)dist_tfluna/100;
	//printf("Dis:%i Lo:%i Hi:%i\r\n",dist_tfluna,dist_low,dist_high);
	delta_dist = (dist_tfluna_f - dist_tfluna_f1);
	dist_spd = delta_dist/(float)delta_time_tf*1000;
	dist_spd_lowpass = dist_spd_lowpass*0.9 + dist_spd*0.1;
	//printf("d:%.2f s1:%.2f\r\n",dist_tfluna_f,dist_spd_lowpass);
	//printf("d:%.2f s:%.2f s1:%.2f\r\n",dist_tfluna_f,dist_spd,dist_spd_lowpass);
	//printf("Dis:%i Spd:%.2f\r\n",dist_tfluna,dist_spd);
	prev_time_tf = sys_time_ms;
	dist_tfluna_f1 = dist_tfluna_f;
}

/*
#define VL53L1_ADDR 							0x52// 0x29
#define VL53L1_IDENTIFICATION__MODEL_ID			0x010F

int VL53L1_Write(uint8_t *pdata, uint32_t count) {
    int status;

    status = HAL_I2C_Master_Transmit(&hi2c1, VL53L1_ADDR, pdata, count, 1000);
    if (status) {
        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
        //XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }
    return status;
}

int VL53L1_Read(uint8_t *pdata, uint32_t count) {
    int status;

    status = HAL_I2C_Master_Receive(&hi2c1, VL53L1_ADDR|1, pdata, count, 1000);
    if (status) {
        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
        //XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }
    return status;
}

void VL53L1X_init(void){
	uint8_t 	Rec_Data[2];
	uint8_t 	Trs_Data[2] = {0x01,0x0F};
	VL53L1_Write(Trs_Data, 2);
	VL53L1_Read(Rec_Data,2);

	//HAL_I2C_Mem_Read(&hi2c1, VL53L1_ADDR, Trs_Data, 2, Rec_Data, 2, 1000);
	printf("v1:%i v2:%i\r\n",Rec_Data[0],Rec_Data[1]);
}
*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  MPU6050_Init();//MPU6050_START
  ReadTF_Luna_init();//tf_luna start
  //VL53L1X_init();//VL53L1X start

  CAN_BUS_init();//setup filter and send a test message

  //HAL_TIM_Base_Start_IT(&htim2);//PPM_IN
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim3);//PWM_OUT
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);



  HAL_ADC_Start_DMA(&hadc1, adc_reading, 2);
  // if using 3 channels set adc_reading as an array like this -> adc_reading[3];
  //HAL_ADC_Start_DMA(&hadc1, adc_reading, 3);
  joystick_init();//calibration find mid point, this is after ADC DMA initialization

  HAL_UART_Transmit(&huart3, aData, 8, 1000);
  printf("Hello, world!\r\n");
  printf("Hello, world!\r\n");
  printf("Hello, world!\r\n");
  HAL_UART_Transmit(&huart3, aData, 8, 1000);//test uart 3

  //htim2.Instance ->CCR2 = servo_out_midpt;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //printf("adc0: %i adc1:%i\r\n",adc_reading[0],adc_reading[1]);

	  /*
	  current_time = sys_time_ms;

	  if (hwa_out != 0){
		  hwa_out_zero_time = current_time;
	  }
	  if (current_time - hwa_out_zero_time > 2000){// 2,000 = 2seconds
		  str_over_ride = 0;
	  }else{
		  str_over_ride = 1;
	  }

	  if (current_time - received_0x101_time > 500){// 500 = 0.5second
		  receive_0x101_timeout = 1;
	  }else{
		  receive_0x101_timeout = 0;
	  }
	  */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
	printf("RCC_error_1\n\r");
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
	printf("RCC_error_2\n\r");
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
	printf("RCC_error_3\n\r");
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
	printf("ADC_error_1\n\r");
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	printf("ADC_error_2\n\r");
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  printf("ADC_error_3\n\r");
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
	  printf("CAN_error_1\n\r");
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
	  printf("I2C_err_1\n\r");
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
	  printf("TIM2_err_1\n\r");
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
	  printf("TIM2_err_2\n\r");
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
	  printf("TIM2_err_3\n\r");
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
	  printf("TIM2_err_4\n\r");
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
	  printf("TIM2_err_5\n\r");
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000-1;//main timer cycle interrupt 10000-1 100hz 5000-1 200hz
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
	  printf("TIM3_err_1\n\r");
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
	  printf("TIM3_err_2\n\r");
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
	  printf("TIM3_err_3\n\r");
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
	  printf("TIM3_err_4\n\r");
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
	  printf("TIM3_err_5\n\r");
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
	  printf("TIM3_err_6\n\r");
    Error_Handler();
  }
  sConfigOC.Pulse = 1000;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
	  printf("TIM3_err_7\n\r");
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
	  printf("TIM3_err_8\n\r");
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
	  printf("UART2_err_1\n\r");
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 2000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
	  printf("UART3_err_1\n\r");
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim3){//TIME_INTERRUPT

  	sys_time_up ++;
 	sys_time_ms = sys_time_up * 5;
 	//Read_joystick();
 	if (tf_luna_ini == 1){
 		if (sys_time_ms%dist_measure_devider == 0)
 			ReadTF_Luna();
 	}

 	if (mpu6050_ini == 1){
 		MPU6050_Read_Accel();
 		MPU6050_Read_Gyro();
 		Gx -= Gyro_X_CAL;
 		Gy -= Gyro_Y_CAL;
 		Gz -= Gyro_Z_CAL;
 		// times delta time and convert to rad
 		Gx_rad = Gx * 0.0001745329;
 		Gy_rad = Gy * 0.0001745329;
 		Gz_rad = Gz * 0.0001745329;
 		//pitch_a = -atan2(Ax,Az)*57.2957795;
 		//roll_a = atan2(Ay,Az)*57.2957795;
 		pitch_a_rad = -atan(Ax/sqrt(Az*Az + Ay*Ay));
 		roll_a_rad = atan(Ay/sqrt(Az*Az + Ax*Ax));
 		//pitch_a_rad = -atan2(Ax,Az);
 		//roll_a_rad = atan2(Ay,Az);

 		cos_roll = cos(roll_est_rad);
 		sin_roll = sin(roll_est_rad);
 		cos_pitch = cos(pitch_est_rad);
 		sin_pitch = sin(pitch_est_rad);


// 		yaw_est += cos(roll_est*0.01745329)*cos(pitch_est*0.01745329)*Gz*0.01;
// 		yaw_est += sin(roll_est*0.01745329)*Gy*0.01;
// 		yaw_est -= sin(pitch_est*0.01745329)*Gx*0.01;

 		yaw_est_rad +=  cos_roll*cos_pitch*Gz_rad + sin_roll*Gy_rad - sin_pitch*Gx_rad;
 		roll_est_rad += cos_pitch*Gx_rad + sin_pitch*Gz_rad;
 		pitch_est_rad += cos_roll*Gy_rad -sin_roll*Gz_rad;

 		if (yaw_est_rad > 3.14159265358979)
			yaw_est_rad -= 6.28318530717959;
 		if (yaw_est_rad < -3.14159265358979)
			yaw_est_rad += 6.28318530717959;


// 		pitch_est += cos(roll_est*0.01745329)*Gy*0.01;
// 		roll_est += cos(pitch_est*0.01745329)*Gx*0.01;

// 		pitch_est += -sin(roll_est*0.01745329)*Gz*0.01;//0.01*0.01745329);
// 		roll_est += sin(pitch_est*0.01745329)*Gz*0.01;//0.01*0.01745329);

 		pitch_ino = pitch_a_rad - pitch_est_rad;
 		pitch_est_rad += pitch_ino*0.005;

 		roll_ino = roll_a_rad - roll_est_rad;
 		roll_est_rad += roll_ino*0.005;

 		pitch_est = pitch_est_rad*rad2deg;
 		roll_est = roll_est_rad*rad2deg;
 		yaw_est = yaw_est_rad*rad2deg;


 		//printf("Pa:%.2f Ra:%.2f\r\n",pitch_a_rad*rad2deg, roll_a_rad*rad2deg);
 		//printf("Pe:%.2f Re:%.2f Ye:%.2f\r\n",pitch_est, roll_est, yaw_est);
 		//printf("Ax:%.3f Gx:%.3f\r\n", Ax, Gx);
 		//printf("PA:%.3f RA:%.3f GY:%.3f GX:%.3f\r\n",pitch_a,roll_a,Gy,Gx);
 		//printf("PE:%.2f RE:%.2f\r\n", pitch_est, roll_est);
 		//printf("%.3f %.3f %.3f %.3f %.3f %.3f\r\n",Ax,Ay,Az,Gx,Gy,Gz);
 		//printf("%i %i %i %i %i %i\r\n",Accel_X_RAW,Accel_Y_RAW,Accel_Z_RAW,Gyro_X_RAW,Gyro_Y_RAW,Gyro_Z_RAW);
 	}

 	Drone_Control_quad();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	//printf(error_code);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
