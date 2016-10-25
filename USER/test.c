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

//����1����1���ַ� 
//c:Ҫ���͵��ַ�
void usart1_send_char(u8 c)
{
	while((USART1->SR&0X40)==0);//�ȴ���һ�η������   
	USART1->DR=c;   	
} 
//�������ݸ�����������λ�����(V2.6�汾)
//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//���28�ֽ����� 
	send_buf[len+3]=0;	//У��������
	send_buf[0]=0X88;	//֡ͷ
	send_buf[1]=fun;	//������
	send_buf[2]=len;	//���ݳ���
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//��������
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//����У���	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1 
}
//���ͼ��ٶȴ��������ݺ�����������
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]; 

	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
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
	usart1_niming_report(0XA1,tbuf,12);//�Զ���֡,0XA1
}	
//ͨ������1�ϱ���������̬���ݸ�����
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//��0
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
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
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_report(0XAF,tbuf,28);//�ɿ���ʾ֡,0XAF
}  

//*p:�ַ�����ʼ��ַ	
void prints(u8 *p,u8 nextline){
	static u8 line=0;
	LCD_ShowString(3,2+line*28,240,24,24,p);
	if(nextline == 1)
		line++;
	if(line>16){
		line = 0;
	}
}





int main(void)
{		
	u8 t=0,time=0,report=1,start=0;			//Ĭ�Ͽ����ϱ�
	u8 t60=0;
	u8 key;
	u8 i; 
	//u8 tempnum=100;
	
	float pitch,roll,yaw,dertayaw; 		//ŷ����
	//float tempfloat;
	
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short axystart=0;
	short xmax=0,xmin=0,ymax=0,ymin=0,zmax=GRAVTY;
	//short xmaxpure=0,xminpure=0,ymaxpure=0,yminpure=0,aacxpure=0,aacypure=0;
	short gyrox,gyroy,gyroz;	//������ԭʼ����
	short temp;					//�¶�	    
	long templong;

	u8 fnum = 40;
	u8 tadd = 0;
	short avt = 0;
	short avx = 0;
	short avy = 0;
	short avz = 0;
	short ftemp[40];//fnum+1
	short fx[40];//fnum+1
	short fy[40];//fnum+1
	short fz[40];//fnum+1
	long sumt = 0;
	long sumx = 0;
	long sumy = 0;
	long sumz = 0;

 	dertayaw = 0.013445;//per three second
	Stm32_Clock_Init(9);		//ϵͳʱ������
	uart_init(72,500000);		//���ڳ�ʼ��Ϊ500000
	delay_init(72);	   	 		//��ʱ��ʼ�� 
	usmart_dev.init(72);		//��ʼ��USMART
	LED_Init();		  			//��ʼ����LED���ӵ�Ӳ���ӿ�
	LCD_Init();			   		//��ʼ��LCD  
	
	KEY_Init();					//��ʼ������
	MPU_Init();					//��ʼ��MPU6050
 	LCD_Clear(BLACK);
 	LCD_Clear(WHITE);


	POINT_COLOR=BLACK;			//��������Ϊ��ɫ 

	LCD_DrawRectangle(1,1,798,478);
	LCD_DrawLine(239,1,239,478);
	gyro_circle_panel(ARX,ARY,110,6);
	gyro_circle_panel(BRX,BRY,110,6);
	gyro_circle_panel(CRX,RRY,90,6);
	gyro_circle_panel(DRX,RRY,90,6);
	gyro_circle_panel(ERX,RRY,90,6);

	
	prints("Pandaroll",1);
	prints("MPU6050 TEST",1);

	
	while(RTC_Init())		//RTC��ʼ��	��һ��Ҫ��ʼ���ɹ�
	{ 
		//LCD_ShowString(30,430,200,16,16,"RTC ERROR!   ");
		prints("RTC ERROR!   ",0);
		delay_ms(800);
		//LCD_ShowString(30,430,200,16,16,"RTC Trying...");
		prints("RTC Trying...",0);	
	}	prints(" ",1);    						
			
	// while(mpu_dmp_init())
	// {
	// 	//LCD_ShowString(30,130,200,16,16,"MPU6050 Error");
	// 	prints("MPU6050 Error",0);
	// 	delay_ms(200);
	// 	//LCD_Fill(30,130,239,130+16,WHITE);
	// 	prints("             ",0);
	// 	delay_ms(200);
	// }  
	
	//LCD_ShowString(30,150,200,16,16,"KEY0:UPLOAD ON/OFF");
//	POINT_COLOR=WHITE;//��������Ϊ��ɫ 
 	//LCD_ShowString(30,170,200,16,16,"UPLOAD ON ");
	//prints("UPLOAD ON ",1);
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
 	while(1)
	{
		key=KEY_Scan(0);
		if(key==KEY0_PRES)
		{
			report=!report;
			if(report)LCD_ShowString(30,170,200,16,16,"UPLOAD ON ");
			else LCD_ShowString(30,170,200,16,16,"UPLOAD OFF");
		}
		if(key==KEY1_PRES)
		{
			start=!start;
			
		}
		if(start==1){
			while(mpu_dmp_init())
			{
				//LCD_ShowString(30,130,200,16,16,"MPU6050 Error");
				prints("MPU6050 Error",0);
				delay_ms(200);
				//LCD_Fill(30,130,239,130+16,WHITE);
				prints("             ",0);
				delay_ms(200);
			} 

			LCD_ShowString(30,130,200,16,16,"");
			prints("MPU6050 OK   ",1);
			start=2;
		}
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			yaw = yaw - dertayaw;//for decracing the bias
			if(yaw < 0){
				yaw+=360;
			}
			if(yaw > 360){
				yaw-=360;
			}
			
			temp=MPU_Get_Temperature();	//�õ��¶�ֵ
			
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������

			if(tadd<fnum){
				ftemp[tadd]=temp;
				fx[tadd]=aacx;
				fy[tadd]=aacy;
				fz[tadd]=aacz;
				tadd++;
			}
			if (tadd==fnum){
				sumt=0;
				sumx = 0;
				sumy = 0;
				sumz = 0;
				for (i = 0; i < fnum; i++){
					sumt+=ftemp[i];
					sumx+=fx[i];
					sumy+=fy[i];
					sumz+=fz[i];
				}
				avt = sumt/fnum;
				avx = sumx/fnum;
				avy = sumy/fnum;
				avz = sumz/fnum;
				for (i = 0; i < fnum-1; i++){
					ftemp[i]=ftemp[i+1];
					fx[i]=fx[i+1];
					fy[i]=fy[i+1];
					fz[i]=fz[i+1];
				}
				ftemp[fnum-1]=temp;
				fx[fnum-1]=aacx;
				fy[fnum-1]=aacy;
				fz[fnum-1]=aacz;
			}
			temp=avt;		
			aacx=avx;
			aacy=avy;
			aacz=avz;

			if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//���Զ���֡���ͼ��ٶȺ�������ԭʼ����
			if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
			if((t%10)==0)
			{
				if(axystart<10){
					axystart++;
				} 
				if(axystart==10){
					//axstart=avx;
					//aystart=avy;
					//aacxpure=
					axystart = 10+2;
				}
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
				showxyupdatepo(aacx,aacy,0,0);
				showXandY(aacx,aacy);
				showzupdate(aacz,RED);
				showprylineupdate(&pitch,&roll,&yaw);

				if(aacx>xmax){
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
				

				// if(aacz<zmin){
				// 	POINT_COLOR = WHITE;
				// 	LCD_Draw_Circle(ARX,ARY,zmin*50/GRAVTY);
				// 	zmin = aacz;
				// }
				// POINT_COLOR = RED;
				// LCD_Draw_Circle(ARX,ARY,30);
				POINT_COLOR = BLACK;
				if(temp<0)
				{
					LCD_ShowChar(30+48,200,'-',16,0);		//��ʾ����
					temp=-temp;		//תΪ����
				}else LCD_ShowChar(30+48,200,' ',16,0);		//ȥ������ 
				LCD_ShowxNum(30+48+8,200,temp/100,3,16,0);		//��ʾ��������
				LCD_ShowxNum(30+48+40,200,temp%10,1,16,128);		//��ʾС������
				

				templong = aacx*980/GRAVTY;
				if(templong<0)
				{
					LCD_ShowChar(30+48,220,'-',16,0);
					templong=-templong;
				}else LCD_ShowChar(30+48,220,'+',16,0);
				LCD_ShowxNum(30+48+8,220,templong/100,2,16,0);
				LCD_ShowxNum(30+48+32,220,templong%100,2,16,128);
				
				templong = aacy*980/GRAVTY;
				if(templong<0)
				{
					LCD_ShowChar(30+48,240,'-',16,0);
					templong=-templong;
				}else LCD_ShowChar(30+48,240,'+',16,0);
				LCD_ShowxNum(30+48+8,240,templong/100,2,16,0);
				LCD_ShowxNum(30+48+32,240,templong%100,2,16,128);

				// tempfloat = aacz*9.79/GRAVTY;
				// templong = tempfloat*100;
				templong = aacz*980/GRAVTY;
				if(templong<0)
				{
					LCD_ShowChar(30+48,260,' ',16,0);
					//LCD_ShowChar(30+48,340,'--',16,0);
					templong=-templong;
				}else LCD_ShowChar(30+48,260,' ',16,0);
				LCD_ShowxNum(30+48+8,260,templong/100,2,16,0);
				LCD_ShowxNum(30+48+32,260,templong%100,2,16,128);


				temp=pitch*10;
				if(temp<0)
				{
					LCD_ShowChar(285+48,455,'-',16,0);		//��ʾ����
					temp=-temp;		//תΪ����
				}else LCD_ShowChar(285+48,455,' ',16,0);		//ȥ������ 
				LCD_ShowxNum(285+48+8,455,temp/10,3,16,0);		//��ʾ��������
				LCD_ShowxNum(285+48+40,455,temp%10,1,16,128);		//��ʾС������ 
				
				temp=roll*10;
				if(temp<0)
				{
					LCD_ShowChar(470+48,455,'-',16,0);		//��ʾ����
					temp=-temp;		//תΪ����
				}else LCD_ShowChar(470+48,455,' ',16,0);		//ȥ������ 
				LCD_ShowxNum(470+48+8,455,temp/10,3,16,0);		//��ʾ��������
				LCD_ShowxNum(470+48+40,455,temp%10,1,16,128);		//��ʾС������ 
				
				temp=yaw*10;
				if(temp<0)
				{
					LCD_ShowChar(655+48,455,'-',16,0);		//��ʾ����
					temp=-temp;		//תΪ����
				}else LCD_ShowChar(655+48,455,' ',16,0);		//ȥ������ 
				LCD_ShowxNum(655+48+8,455,temp/10,3,16,0);		//��ʾ��������	    
				LCD_ShowxNum(655+48+40,455,temp%10,1,16,128);		//��ʾС������  
				t=0;
				//LED0=!LED0;//LED��˸
			}
		}
		
		if(time != calendar.sec){
			time = calendar.sec;
			LCD_ShowxNum(30,370,calendar.w_year,4,16,0);
			LCD_ShowxNum(70,370,calendar.w_month,2,16,128);									  
			LCD_ShowxNum(94,370,calendar.w_date,2,16,128);

			LCD_ShowxNum(30,406,calendar.hour,2,16,0);									  
			LCD_ShowxNum(54,406,calendar.min,2,16,128);								  
			LCD_ShowxNum(78,406,calendar.sec,2,16,128);
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












