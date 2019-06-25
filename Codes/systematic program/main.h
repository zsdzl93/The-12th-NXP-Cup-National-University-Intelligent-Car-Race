#include "common.h"
#include "include.h"
#include "math.h"


///********************  Wireless Oscilloscope  ***************************/
int receive_flag = 0;
uint8 buff[DATA_PACKET];

// /********************  Image  ***************************/
int  Left_Black;
int Right_Black;

///********************  Angle  ***************************/
float Gyro_Now,angle_offset_vertical;			// The angular velocity after conversion of the gyroscope, the acceleration angle after conversion
float g_fCarAngle,g_fGyroscopeAngleIntegral;	// Final angle
volatile int    MMA7361 ,ENC03,real_angle;		// Accelerator AD, gyro AD, module output angle
volatile int Pre_Pre_Pre_MMA7361,Pre_Pre_MMA7361,Pre_MMA7361;
volatile int   pre_pre_pre_ENC03,pre_pre_ENC03,pre_ENC03,average_ENC03;
float g_fAngleControlOut;						// Angle control output(output needed for self-balancing)

float P_ANGLE =35;                           //   10 14 
float D_ANGLE =16;  // 4                           //1.2   0.7´ó

int AngleSet=3;

/********************  SpeedControl  ***************************/
double g_fCarSpeed;			// Average measuring speed of left and right
int16 Val_Left,Val_Right;	// left and right measuring speed




volatile float  Speed_L,Speed_R,Speed_L_Last,Speed_R_Last;  // Left and right wheel speed, final speed

float g_fSpeedControlError_L,g_fSpeedControlError_R,g_fSpeedControlDelta_L,g_fSpeedControlDelta_R;// Speed control error and its differential
float g_fSpeedControlOld_L,g_fSpeedControlOld_R;	// Output to motors, actual final old value
float g_fSpeedControl_L,g_fSpeedControl_R;			// Cycle output to motor value

int  g_nSpeedControlPerid=0;						//Speed control sub-cycle
int  g_nSpeedControlCount=0; 

int error=0,LastError=0,PrevError=0;

float Kp2 = 7;      // 0.85    6
float Ki2 = 0.15;    //0.08
float Kd2 = 0.055;  //0.055

int Val_Set;



/******************** DIP switch  ***************************/
int DIPswitch_1,DIPswitch_2,DIPswitch_3,DIPswitch_4,DIPswitch_5,DIPswitch_6,DIPswitch_7,DIPswitch_8;

int ringEnter_R;

/********************  Direction Control middle line  ***************************/

int ControlRowMiddleline_New,ControlRowMiddleline_Old;		// New and old control line center line
int MiddlelineError,MiddlelinePreError,MiddlelineLastError;	// middle line error
float g_fDirectionControlNew;								// Directoin control output by PID
float g_fDirectionControlOut;								// Directoin control output

float Kp1=1.2,Kd1=0.12;//    0.4

int ControlRow=40 ;
int AverageMiddleline;


/******************** Speed set ***************************/

/******************** Analysis ***************************/

// middle line
int middleline_start = 40;
uint16 left[CAMERA_H]={0};
uint16 right[CAMERA_H]={0};
int mid[CAMERA_H]={0};

// Small triangle recognition (Curved road)
//#define Curve

int rightTriangleFlag, leftTriangleFlag;
int rightSmallTriangleflag, leftSmallTriangleflag;
int leftTurn_flag1=0,rightTurn_flag1=0;
uint8 outBoundRecord[80];
int outBoundRecordAverage;
uint8 oldoutBoundRow;
uint8 now_outBoundRow;
int smallestColumn=0;
uint8 outBoundRow;

int fillRightBound=0,fillLeftBound=0;

//********************(Obstacle recognition)****************************
uint8 leftObstacleflag=0;
uint8 rightObstacleflag=0;
uint8 obstacleflag=0;
uint8 narrowBottomflag=0;	// Narrow bottom
uint8 leftPosLeap=0;		 	// left positive leap
uint8 leftNegLeap=0;			// left negative leap
uint8 rightPosLeap=0;		// right positive leap
uint8 rightNegLeap=0;		// right negative leap
int obstacleDelay=0;
int narrowBottomDelay=0;
int cross_ring_obstacleDelay=0;
int leftDerivative[60];
int rightDerivative[60];

//**********************(Ring recognition)********************
int ringflag=0;
int Right_B2W,Left_B2W;
int youdajiao1=0;
int youdajiao2=0;
int ringEnterflag=0;
int chazhi;
int lukuanjishu;
int lukuanflag;
float left,right;
int ringDelay;
int right_high,left_high;

//**********************(Crossroad recognition)********************
int crossflag=0;
int leftEmptyflag=0;
int leftEmptyRow=0;
int rightEmptyflag=0;
int bothSideEmptyRow=0;


//**********************(Bound losing check)********************
int loseRightBound_flag=0;
int loseLeftBound_flag=0;
int loseLeftBound=0;
int loseRightBound=0;

//********************(Stop)**************************
int B2H=0,H2B=0,t=0;      // Waiting time after turning on the power
int stopflag=0;
int stop_obstacleDelay=0;
int stoptimes=0;
int stopline=0;
int stopDelay=0;
int lowerBound=0;
int upperBound=59;
int oldlowerBound=0;
int oldupperBound=59;

//********************(Exception processing)**************************

int exceptionflag1, exceptionflag2;
int GYRO_VAL;

//**********************(Ramp recognition)********************

int uphillflag,downhillflag=0;
int ramp=0;
float leftSlope,rightSlope,leftRampBound[60],rightRampBound[60];
int downhillDelay=0;

/********************  Ring direction control Kp  ***************************/
float Kp3;

/********************  DIP switch  ***************************/
int DIPswitch_1,DIPswitch_2,DIPswitch_3,DIPswitch_4,DIPswitch_5,DIPswitch_6,DIPswitch_7,DIPswitch_8;
//int GYROLevel,GYROSet[2]={1753,1830};
int SpeedLevel,Kp3_Level;
int rampSet=0;
int yuanhuanchengxu;
// Speed set
int SpeedSet[8]={80,90,100,110,120,130,140,150};
float Kp3_Set[4]={ 1.0,1.2,1.4,1.6};
                   
                 
