#include "lcd.h"  
#include "niming.h"

#define FNUM 40//数据滤波器的滑动窗口大小
void Init(void){//各种初始化，画界面
	Stm32_Clock_Init(9);		//系统时钟设置
	uart_init(72,500000);		//串口初始化为500000
	delay_init(72);	   	 		//延时初始化 
	usmart_dev.init(72);		//初始化USMART
	LED_Init();		  			//初始化与LED连接的硬件接口
	LCD_Init();			   		//初始化LCD  
	KEY_Init();					//初始化按键
	MPU_Init();					//初始化MPU6050
 	LCD_Clear(BLACK);
 	LCD_Clear(WHITE);
	POINT_COLOR=BLACK;			//设置字体为红色 
	LCD_DrawRectangle(1,1,798,478);
	LCD_DrawLine(239,1,239,478);
	gyro_circle_panel(ARX,ARY,110,6);
	gyro_circle_panel(BRX,BRY,110,6);
	gyro_circle_panel(CRX,RRY,90,6);
	gyro_circle_panel(DRX,RRY,90,6);
	gyro_circle_panel(ERX,RRY,90,6);
	prints("Pandaroll",1);
	prints("MPU6050 TEST",1);

	while(RTC_Init()){ 		//RTC初始化	，一定要初始化成功
		prints("RTC ERROR!   ",0);
		delay_ms(800);
		prints("RTC Trying...",0);	
	}	prints(" ",1);    						

 	LCD_ShowString(30,200,200,16,16," Temp:    . C");	
 	LCD_ShowString(30,220,200,16,16," acx :   .  m/s2");	
 	LCD_ShowString(30,240,200,16,16," acy :   .  m/s2");	 
 	LCD_ShowString(30,260,200,16,16," acz :   .  m/s2");
 	LCD_ShowString(285,455,200,16,16," Roll:    . C");	
 	LCD_ShowString(470,455,200,16,16,"Pitch:    . C");	 
 	LCD_ShowString(655,455,200,16,16," Yaw :    . C");					 
	LCD_ShowString(30,370,200,16,16,"    -  -     ");	   
	LCD_ShowString(30,406,200,16,16,"  :  :  ");
	dotline(ARX-100,ARY,ARX,ARY,4,DOTCL);
	dotline(ARX,ARY,ARX+100,ARY,4,RED);
	dotline(ARX,ARY-100,ARX,ARY+100,4,DOTCL);
	dotline(BRX-100,ARY,BRX+100,ARY,4,DOTCL);
	dotline(BRX,ARY-100,BRX,ARY+100,4,DOTCL);
	LCD_ShowString(ARX-125,ARY-8,16,16,16,"g");
	LCD_ShowString(ARX-4,ARY+113,16,16,16,"g");
	POINT_COLOR=RED;
	LCD_ShowString(ARX+50,ARY+3,16,16,16,"g");
	LCD_ShowString(ARX+113,ARY-8,16,16,16,"2g");
	POINT_COLOR=BLACK;
}

void mpuDmpInit(void){
	while(mpu_dmp_init())
	{
		prints("MPU6050 Error",0);
		delay_ms(200);
		prints("             ",0);
		delay_ms(200);
	} 
	LCD_ShowString(30,130,200,16,16,"");
	prints("MPU6050 OK   ",1);
}

void usart1_send_char(u8 c){//串口1发送1个字符 //c:要发送的字符
	while((USART1->SR&0X40)==0);//等待上一次发送完毕   
	USART1->DR=c;
} 

void usart1_niming_report(u8 fun,u8*data,u8 len){//传送数据给匿名四轴上位机软件(V2.6版本)//fun:功能字. 0XA0~0XAF										
	u8 send_buf[32];								//data:数据缓存区,最多28字节!!//len:data区有效数据个数
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
}

void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz){
	u8 tbuf[12]; 								//发送加速度传感器数据和陀螺仪数据
	tbuf[0]=(aacx>>8)&0XFF;						//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
	tbuf[1]=aacx&0XFF;							//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart1_niming_report(0XA1,tbuf,12);//自定义帧头,0XA1
}

void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw){
	u8 tbuf[28]; 								//通过串口1上报结算后的姿态数据给电脑
	u8 i;										//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
	for(i=0;i<28;i++)tbuf[i]=0;//清0			//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
	tbuf[0]=(aacx>>8)&0XFF;						//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
	tbuf[1]=aacx&0XFF;							//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
	tbuf[2]=(aacy>>8)&0XFF;						//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;	
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_report(0XAF,tbuf,28);//飞控显示帧,0XAF
}  

void prints(u8 *p,u8 nextline){//*p:字符串起始地址	
	static u8 line=0;
	LCD_ShowString(3,2+line*28,240,24,24,p);
	if(nextline == 1)
		line++;
	if(line>16){
		line = 0;
	}
}

short windowfilterT(short temp){//滑动窗口滤波器，可以优化//温度滤波
	static u8 tadd = 0;
	u8 i = 0;
	static short ftemp[FNUM];
	long sum=0;
	short avt=0;
	if(tadd<FNUM){
		ftemp[tadd]=temp;
		tadd++;
	}
	if (tadd==FNUM){
		for (i = 0; i < FNUM; i++){
			sum+=ftemp[i];
		}
		avt = sum/FNUM;
		for (i = 0; i < FNUM-1; i++){
			ftemp[i]=ftemp[i+1];
		}
		ftemp[FNUM-1]=temp;
	}
	return avt;		
}

short windowfilterAx(short aacx){//滑动窗口滤波器，可以优化//x方向加速度滤波
	static u8 tadd = 0;
	u8 i = 0;
	static short ftemp[FNUM];
	long sum=0;
	short avx=0;
	if(tadd<FNUM){
		ftemp[tadd]=aacx;
		tadd++;
	}
	if (tadd==FNUM){
		for (i = 0; i < FNUM; i++){
			sum+=ftemp[i];
		}
		avx = sum/FNUM;
		for (i = 0; i < FNUM-1; i++){
			ftemp[i]=ftemp[i+1];
		}
		ftemp[FNUM-1]=aacx;
	}
	return avx;		
}

short windowfilterAy(short aacy){//滑动窗口滤波器，可以优化//y方向加速度滤波
	static u8 tadd = 0;
	u8 i = 0;
	static short ftemp[FNUM];
	long sum=0;
	short avy=0;
	if(tadd<FNUM){
		ftemp[tadd]=aacy;
		tadd++;
	}
	if (tadd==FNUM){
		for (i = 0; i < FNUM; i++){
			sum+=ftemp[i];
		}
		avy = sum/FNUM;
		for (i = 0; i < FNUM-1; i++){
			ftemp[i]=ftemp[i+1];
		}
		ftemp[FNUM-1]=aacy;
	}
	return avy;		
}

short windowfilterAz(short aacz){//滑动窗口滤波器，可以优化//z方向加速度滤波
	static u8 tadd = 0;
	u8 i = 0;
	static short ftemp[FNUM];
	long sum=0;
	short avz=0;
	if(tadd<FNUM){
		ftemp[tadd]=aacz;
		tadd++;
	}
	if (tadd==FNUM){
		for (i = 0; i < FNUM; i++){
			sum+=ftemp[i];
		}
		avz = sum/FNUM;
		for (i = 0; i < FNUM-1; i++){
			ftemp[i]=ftemp[i+1];
		}
		ftemp[FNUM-1]=aacz;
	}
	return avz;		
}

void updateScaleLine(void){//画指示盘里的坐标虚线，标尺等。
	dotline(ARX-100,ARY,ARX,ARY,4,DOTCL);
	dotline(ARX,ARY,ARX+100,ARY,4,RED);
	dotline(ARX,ARY-100,ARX,ARY+100,4,DOTCL);
	dotline(BRX-100,ARY,BRX+100,ARY,4,DOTCL);
	dotline(BRX,ARY-100,BRX,ARY+100,4,DOTCL);
	LCD_ShowString(ARX-125,ARY-8,16,16,16,"g");
	LCD_ShowString(ARX-4,ARY+113,16,16,16,"g");
	POINT_COLOR=RED;
	LCD_ShowString(ARX+50,ARY+3,16,16,16,"g");
	LCD_ShowString(ARX+113,ARY-8,16,16,16,"2g");
	POINT_COLOR=BLACK;
}

void recordAcxyzMax(short aacx,short aacy,short aacz){//显示xyz加速度的最大值
	static short xmax=0,xmin=0,ymax=0,ymin=0,zmax=0;
	if(aacx>xmax){//记录x轴加速度最大值
		POINT_COLOR = WHITE;
		LCD_Draw_Circle(ARX-xmax*100/GRAVTY,ARY,4);
		xmax = aacx;
	}
	POINT_COLOR = RED;
	LCD_Draw_Circle(ARX-xmax*100/GRAVTY,ARY,4);

	if(aacx<xmin){
		POINT_COLOR = WHITE;
		LCD_Draw_Circle(ARX-xmin*100/GRAVTY,ARY,4);
		xmin = aacx;
	}
	POINT_COLOR = RED;
	LCD_Draw_Circle(ARX-xmin*100/GRAVTY,ARY,4);

	if(aacy>ymax){
		POINT_COLOR = WHITE;
		LCD_Draw_Circle(ARX,ARY+ymax*100/GRAVTY,4);
		ymax = aacy;
	}
	POINT_COLOR = RED;
	LCD_Draw_Circle(ARX,ARY+ymax*100/GRAVTY,4);

	if(aacy<ymin){
		POINT_COLOR = WHITE;
		LCD_Draw_Circle(ARX,ARY+ymin*100/GRAVTY,4);
		ymin = aacy;
	}
	POINT_COLOR = RED;
	LCD_Draw_Circle(ARX,ARY+ymin*100/GRAVTY,4);

	if(aacz>zmax){
		POINT_COLOR = WHITE;
		LCD_Draw_Circle(ARX,ARY,zmax*50/GRAVTY);
		zmax = aacz;
	}
	POINT_COLOR = RED;
	LCD_Draw_Circle(ARX,ARY,zmax*50/GRAVTY);
	
	POINT_COLOR = BLACK;
}

void showNumtxyzrpy(short temp,short aacx,short aacy,short aacz,short roll,short pitch,short yaw){
	long templong;
	if(temp<0){
		LCD_ShowChar(30+48,200,'-',16,0);			//显示负号
		temp=-temp;									//转为正数
	}else 
	LCD_ShowChar(30+48,200,' ',16,0);				//去掉负号 
	LCD_ShowxNum(30+48+8,200,temp/100,3,16,0);		//显示整数部分
	LCD_ShowxNum(30+48+40,200,temp%10,1,16,128);	//显示小数部分

	templong = aacx*980/GRAVTY;
	if(templong<0){
		LCD_ShowChar(30+48,220,'-',16,0);
		templong=-templong;
	}else 
	LCD_ShowChar(30+48,220,'+',16,0);
	LCD_ShowxNum(30+48+8,220,templong/100,2,16,0);
	LCD_ShowxNum(30+48+32,220,templong%100,2,16,128);
	
	templong = aacy*980/GRAVTY;
	if(templong<0){
		LCD_ShowChar(30+48,240,'-',16,0);
		templong=-templong;
	}else 
	LCD_ShowChar(30+48,240,'+',16,0);
	LCD_ShowxNum(30+48+8,240,templong/100,2,16,0);
	LCD_ShowxNum(30+48+32,240,templong%100,2,16,128);

	templong = aacz*980/GRAVTY;
	if(templong<0){
		LCD_ShowChar(30+48,260,' ',16,0);
		//LCD_ShowChar(30+48,340,'--',16,0);
		templong=-templong;
	}else 
	LCD_ShowChar(30+48,260,' ',16,0);
	LCD_ShowxNum(30+48+8,260,templong/100,2,16,0);
	LCD_ShowxNum(30+48+32,260,templong%100,2,16,128);

	temp=pitch*10;
	if(temp<0){
		LCD_ShowChar(285+48,455,'-',16,0);		//显示负号
		temp=-temp;		//转为正数
	}else 
	LCD_ShowChar(285+48,455,' ',16,0);		//去掉负号 
	LCD_ShowxNum(285+48+8,455,temp/10,3,16,0);		//显示整数部分
	LCD_ShowxNum(285+48+40,455,temp%10,1,16,128);		//显示小数部分 
	
	temp=roll*10;
	if(temp<0){
		LCD_ShowChar(470+48,455,'-',16,0);		//显示负号
		temp=-temp;		//转为正数
	}else 
	LCD_ShowChar(470+48,455,' ',16,0);		//去掉负号 
	LCD_ShowxNum(470+48+8,455,temp/10,3,16,0);		//显示整数部分
	LCD_ShowxNum(470+48+40,455,temp%10,1,16,128);		//显示小数部分 
	
	temp=yaw*10;
	if(temp<0){
		LCD_ShowChar(655+48,455,'-',16,0);		//显示负号
		temp=-temp;		//转为正数
	}else 
	LCD_ShowChar(655+48,455,' ',16,0);		//去掉负号 
	LCD_ShowxNum(655+48+8,455,temp/10,3,16,0);		//显示整数部分	    
	LCD_ShowxNum(655+48+40,455,temp%10,1,16,128);		//显示小数部分  
}

void showClockNum(_calendar_obj calendar){
	LCD_ShowxNum(30,370,calendar.w_year,4,16,0);
	LCD_ShowxNum(70,370,calendar.w_month,2,16,128);									  
	LCD_ShowxNum(94,370,calendar.w_date,2,16,128);

	LCD_ShowxNum(30,406,calendar.hour,2,16,0);									  
	LCD_ShowxNum(54,406,calendar.min,2,16,128);								  
	LCD_ShowxNum(78,406,calendar.sec,2,16,128);
}
