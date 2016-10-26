#ifndef __NIMING_H
#define __NIMING_H	
#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "led.h" 		 	 
#include "lcd.h"  
#include "key.h"  
#include "mpu6050.h"
#include "usmart.h"   
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "rtc.h"
#define DOTCL GREEN
//这里面包含了串口数据同步函数以及屏幕显示函数 2016年10月26日
void Init(void);
void mpuDmpInit(void);
void usart1_send_char(u8 c);
void usart1_niming_report(u8 fun,u8*data,u8 len);
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);
void prints(u8 *p,u8 nextline);
short windowfilterT(short temp);
short windowfilterAx(short aacx);
short windowfilterAy(short aacy);
short windowfilterAz(short aacz);
void updateScaleLine(void);
void recordAcxyzMax(short aacx,short aacy,short aacz);
void showNumtxyzrpy(short temp,short aacx,short aacy,short aacz,short roll,short pitch,short yaw);
void showClockNum(_calendar_obj calendar);

#endif
