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
#include "niming.h"

int main(void){		
	u8 t=0,time=0,report=1,start=0;			//默认开启上报
	u8 t60=0;
	u8 key;
	float pitch,roll,yaw,dertayaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	short temp;					//温度	    
 	dertayaw = 0.013445;//per three second
	Init();
	//mpuDmpInit();//上电直接初始化融合数据输出。
 	while(1){
		key=KEY_Scan(0);
		if(key==KEY0_PRES){
			report=!report;
			if(report)LCD_ShowString(30,170,200,16,16,"UPLOAD ON ");
			else LCD_ShowString(30,170,200,16,16,"UPLOAD OFF");
		}	
		if(key==KEY1_PRES&&start==0){
			start=1;	
		}
		if(start==1){//按键之后再开始初始化dmp融合，给开机一定的时间
			mpuDmpInit();
			start=2;
		}
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0){ //成功获取一次mpu融合之后的值
			yaw = yaw - dertayaw;//消除地转误差，以后使用地磁计可弃用航向角
			if(yaw < 0) yaw+=360; 
			if(yaw > 360) yaw-=360; //控制yaw的范围是0°--360°
			temp=MPU_Get_Temperature();	//得到温度值
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			temp=windowfilterT(temp);		//对温度三轴加速度进行滤波，平滑窗口。
			aacx=windowfilterAx(aacx);
			aacy=windowfilterAy(aacy);
			aacz=windowfilterAz(aacz);
			if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧发送加速度和陀螺仪原始数据
			if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));	
			if((t%13)==0){
				updateScaleLine();//画标尺线
				showxyupdatepo(aacx,aacy,0,0);//画指示线
				showXandY(aacx,aacy);
				showzupdate(aacz,RED);
				showprylineupdate(&pitch,&roll,&yaw);
				recordAcxyzMax(aacx,aacy,aacz);
				showNumtxyzrpy(temp,aacx,aacy,aacz,roll,pitch,yaw);
				t=0;
			}
		}
		if(time != calendar.sec){
			time = calendar.sec;
			showClockNum(calendar);
			if(t60%60==0){
				LED0=!LED0;
				//usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
				//printf("\t\n");
				t60=0;
				LED0=!LED0;
			}
			if(t60%3==0){
				dertayaw += 0.013445;
				LED0=!LED0;
			}
			t60++;	
		}
		t++; 
	} 	
}






