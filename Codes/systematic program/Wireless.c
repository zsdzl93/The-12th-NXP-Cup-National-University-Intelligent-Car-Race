#include "common.h"
#include "include.h"
#include "self_include.h"

/******************** Wireless Oscilloscope ***************************/
void SendBox()
{
  uint8 b[24];
  int j=0;
  int _temp; 
  
  b[0]=23;
  b[1]=0xAA;
  b[2]=0xAA;
  b[3]=0x02;
  b[4]=18;// Number of data
  _temp = (int16)(ENC03); // ENC03 gyro AD  rightTriangleflag ACC_X   crossflag  leftObstacleflag  jiasu_flag
  b[5]=BYTE1(_temp);
  b[6]=BYTE0(_temp);
  _temp = (int16)(GYRO_VAL);//acceleration AD Val_Set  leftTriangleflag  smallestColomn  ACC_Y   rightObstacleflag   MMA7361
  b[7]=BYTE1(_temp);
  b[8]=BYTE0(_temp);
  _temp = (int16)(angle_offset_vertical);//normalized angular velocity  exceptionflag1 angle_offset_vertical lostLeftBound_flag g_fDirectionControlOut   outBoundRow  ACC_Z narrowBottomflag
  b[9]=BYTE1(_temp);
  b[10]=BYTE0(_temp);
  _temp = (int16)(g_fCarAngle);//final angle  shiziliangkongflag middleline_start   shiziflag  lostRightBound_flag     g_fCarAngle        GYRO_X
  b[11]=BYTE1(_temp);
  b[12]=BYTE0(_temp);
  _temp=(int16)(g_fCarSpeed);//encoder set  fillLeftBoundflag  Val_Right  leftEmptyflag    Val_Set    outBoundRow   L      GYRO_Y
  b[13]=BYTE1(_temp);
  b[14]=BYTE0(_temp);
  _temp=(int16)(Val_Right);//right measure  fillRightBoundflag    rightEmptyflag    R      d     youdajiao2     GYRO_Z
  b[15]=BYTE1(_temp);
  b[16]=BYTE0(_temp);
  _temp=(int16)(Val_Left);//left measure   g_fAngleControlOut   p       DIPswitch_1       downhillDelay      MAG_X   Speed_L
  b[17]=BYTE1(_temp);
  b[18]=BYTE0(_temp);
  _temp=(int16)(Val_Set);//error=Val_Set- Val_speed  g_fDirectionControlOut  MAG_Y  g_fSpeedControl_L  youdajiao1  g_fCarSpeed
  b[19]=BYTE1(_temp);
  b[20]=BYTE0(_temp);
  _temp=(int16)(downhillflag);      // Right_B2W-Left_B2W         g_fSpeedControl_L        MAG_Z
  b[21]=BYTE1(_temp);
  b[22]=BYTE0(_temp);
  b[23]=0;
  for(j=1;j<23;j++)
  b[23]=b[23]+b[j];
  //uart_putbuff (UART0,b,18); // Send array b, via wireless
  nrf_tx(b,24);
}

void recieve_check()  // put after main program end of while（1）
{
	do										// Not in the receiving state, check if pressed;
											// In the receiving state, execute loop, waiting to receive and check whether to jump out;
	{
		if(key_check(KEY_A) == KEY_DOWN)	// Waiting for the button to release
		{
			if( receive_flag == 0)
			{
				disable_irq (PIT0_IRQn);	// Stop pid integral
				led(LED0,LED_ON);
//				PID_reinit(1,20,20);		// Pid reset, integral clear
				receive_flag++;
			}
			else if( receive_flag == 1)
			{
				led(LED0,LED_OFF);
				led(LED1,LED_ON);
				receive_flag++;
			}
			else if( receive_flag == 2)
			{
				enable_irq (PIT0_IRQn);
				led(LED1,LED_OFF);
				receive_flag=0;
			}
		}
		while(key_check(KEY_A) == KEY_DOWN);

		if( receive_flag != 0 )
			Receive_PID();
	}
	while( receive_flag != 0 );
}

void Receive_PID()
{
	uint8 relen;
	relen = nrf_rx(buff,DATA_PACKET);               // Waiting to receive a packet, the data is stored in the buffer
	if(relen != 0)
	{
		if( buff[1]=='?')
			;
		else
		{
			if( receive_flag == 1 )
				set_pid();
			else if( receive_flag == 2 )
				set_pd();
		}
		if( receive_flag == 1 )
			sprintf((char *)buff,"\x1e p=%ld,i=%ld,d=%ld\n",(int16)(Kp2*100),(int16)(Ki2*1000),(int16)(Kd2*1000)); //把str和i合并成一个字符串到buff里，再进行发送
		else if( receive_flag == 2 )
			sprintf((char *)buff,"\x1e p=%ld,d=%ld\n",(int16)(P_ANGLE),(int16)(D_ANGLE*10)); //把str和i合并成一个字符串到buff里，再进行发送
		nrf_tx(buff,DATA_PACKET);       // ,l=%ld,r=%ld     ,(int16)p_SD5_PID.left_setpoint,(int16)p_SD5_PID.right_setpoint
	}
}

void set_pid()
{
	Kp2=buff[1]/16*10 + buff[1]%16*1 + buff[2]/16*0.1+buff[2]%16*0.01 ;
	Ki2=buff[3]/16*1 + buff[3]%16*0.1 + buff[4]/16*0.01+buff[4]%16*0.001 ;
	Kd2=buff[5]/16*1 + buff[5]%16*0.1 + buff[6]/16*0.01+buff[6]%16*0.001 ;
}
void set_pd()
{
	P_ANGLE=buff[1]/16*10 + (float)(buff[1]%16)*1 + (float)(buff[2]/16)*0.1+(float)(buff[2]%16)*0.01 ;
	D_ANGLE=buff[3]/16 + (float)(buff[3]%16)*0.1 + (float)(buff[4]/16)*0.01+(float)(buff[4]%16)*0.001 ;
}
