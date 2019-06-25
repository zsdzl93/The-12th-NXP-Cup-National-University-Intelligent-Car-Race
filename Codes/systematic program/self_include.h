#include "common.h"
#include "include.h"
#include "math.h"
#include "Wireless.h"
#include "AngleCalculate.h"
#include "InterruptService.h"
#include "CameraGet.h"
#include "DirectionControl.h"
#include "SpeedControl.h"
#include "Analysis.h"
#include "MotorOut.h"
#include "Switch.h"

/********************  Wireless Oscilloscope ***************************/
#define BYTE0(dwTemp)           (*(char *)(&dwTemp))
#define BYTE1(dwTemp)           (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)           (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)           (*((char *)(&dwTemp) + 3))

extern int receive_flag;
extern uint8 buff[DATA_PACKET];

extern int Left_Black;
extern int Right_Black;


 /********************  Image  ***************************/
extern uint8 imgbuff[CAMERA_SIZE];		//Define an array that stores received images
extern uint8 img[CAMERA_H*CAMERA_W];	//Decompressed one-dimensional array


extern Site_t site;			//Display the top left corner of the image
extern Size_t imgsize;		//Image size
extern Size_t size;			//Image size in display area


/********************  Angle  ***************************/
extern float Gyro_Now,angle_offset_vertical;			// The angular velocity after conversion of the gyroscope, the acceleration angle after conversion
extern float g_fCarAngle,g_fGyroscopeAngleIntegral;		// final angle
extern volatile int    MMA7361 ,ENC03,real_angle;		// accelerator AD, gyro AD, module output angle
extern volatile int   pre_pre_pre_ENC03,pre_pre_ENC03,pre_ENC03,average_ENC03;
extern  volatile int Pre_Pre_Pre_MMA7361,Pre_Pre_MMA7361,Pre_MMA7361;
extern float g_fAngleControlOut;						// Angle control output(output needed for self-balancing)
extern float P_ANGLE ;
extern float D_ANGLE ;

extern void PIT0_IRQHandler(void);  // Main interrupt service function for self-balancing code
extern void PORTE_IRQHandler();     // For nrf interrupt service

#define ZOUT    ADC0_SE17
#define Gyro1   ADC1_SE16

//#define AngleSet  -2
extern int AngleSet;

/*
 * self-balancing control parameter
 */
#define MMA7361_vertical             2028  // 1860// 1760  //1850// 2600
#define GYRO_SET                     1720  // Gyro median value
#define Gyro_ratio                   0.03   //0.04
#define GRAVITY_ADJUST_TIME_CONSTANT 2
#define DT                           0.05
#define MMA7361_ratio                1.122//1150    
//#define P_ANGLE                      10                           //20ะก45
//#define D_ANGLE                       0// 1.0      10                           //1.2   0.7ด๓
//Repeatedly adjust the above five parameters, as well as the mechanical structure
 
#define MOTOR_DEAD_FORE_L  26   //  Dead zone voltage
#define MOTOR_DEAD_FORE_R  25
#define MOTOR_DEAD_BACK_L  25   //  Dead zone voltage
#define MOTOR_DEAD_BACK_R  30

/*
 * function declaration
 */
 extern   void Rd_Ad_Value(void);                              // AD collection
 extern   void AD_Calculate(void);                             // AD collection and calculate
 extern   void Speed_Calculate(float angle,float angle_dot);   // calculate speed
 
 /********************  SpeedControl  ***************************/
#define SPEED_CONTROL_PERIOD      100      // speed control period
        

//#define SpeedSet 100                      //40-60
extern int GYRO_VAL;
extern double g_fCarSpeed;			// Average measuring speed of left and right
extern int16 Val_Left,Val_Right;	// left and right measuring speed



extern int Val_Set;                 // encoder set
extern float g_fSpeedControlError_L,g_fSpeedControlError_R,g_fSpeedControlDelta_L,g_fSpeedControlDelta_R;// speed control error and its differential.
extern volatile float  Speed_L ,Speed_R,Speed_L_Last,Speed_R_Last;
extern float g_fSpeedControlOld_L,g_fSpeedControlOld_R;		// Output to motors, actual final old value
extern float g_fSpeedControl_L,g_fSpeedControl_R;			// Cycle output to motor value

extern int  g_nSpeedControlPerid;							// Speed control sub-cycle
extern int  g_nSpeedControlCount;


extern int error,LastError,PrevError;

extern float Kp2 ;      // 0.85
extern float Ki2 ;    //0.08
extern float Kd2 ;  //0.055



/********************  Direction Control middle line  ***************************/

//#define ControlRow               40
extern int ControlRow;

extern int MiddlelineError,MiddlelinePreError,MiddlelineLastError;	// middle line error
extern int ControlRowMiddleline_New,ControlRowMiddleline_Old;		// New and old control line center line
extern float g_fDirectionControlNew;								// Directoin control output by PID
extern float g_fDirectionControlOut;								// Directoin control output

extern float Kp1,Kd1;



/******************** Speed set ***************************/
extern int jiasu_flag;
extern int jiansu_flag;
/********************  Analysis  ***************************/

// middle line
extern int middleline_start;
extern uint16 left[CAMERA_H];
extern uint16 right[CAMERA_H];
extern int mid[CAMERA_H];

extern int AverageMiddleline;

// Small triangle recognition (Curved road)
#define Curve           6  //  12
extern int rightTriangleFlag, leftTriangleFlag;
extern int rightSmallTriangleflag, leftSmallTriangleflag;
extern int leftTurn_flag1, rightTurn_flag1;
extern uint8 outBoundRecord[80];
extern int outBoundRecordAverage;
extern uint8  oldoutBoundRow;
extern uint8 now_outBoundRow;
extern int smallestColumn;
extern uint8 outBoundRow;

extern int fillRightBound, fillLeftBound;

//********************(Obstacle recognition)****************************
extern uint8 leftObstacleflag;
extern uint8 rightObstacleflag;
extern uint8 obstacleflag;
extern uint8 narrowBottomflag;	 // Narrow bottom
extern uint8 leftPosLeap;		 // left positive leap
extern uint8 leftNegLeap;		 // left negative leap
extern uint8 rightPosLeap;		 // right positive leap
extern uint8 rightNegLeap;		 // right negative leap
extern int obstacleDelay;
extern int narrowBottomDelay;
extern int cross_ring_obstacleDelay;
extern int leftDerivative[60];
extern int rightDerivative[60];

//**********************(Crossroad recognition)********************
extern int crossflag;
extern int leftEmptyflag;
extern int leftEmptyRow;
extern int rightEmptyflag;
extern int bothSideEmptyRow;

//**********************(Ring recognition)********************

extern int ringflag;
extern int Right_B2W,Left_B2W;
extern int youdajiao1;
extern int youdajiao2;
extern int ringEnterflag;
extern int chazhi;
extern int lukuanjishu;
extern int lukuanflag;
extern float left,right;
extern int ringDelay;

//**********************(Bound losing check)********************

extern int loseRightBound_flag;
extern int loseLeftBound_flag;
extern int loseLeftBound;
extern int loseRightBound;

//********************(Stop)**************************
extern int B2H,H2B,stop_obstacleDelay,t,stopflag,stoptimes,stopline;
extern int stopDelay,lowerBound,upperBound,oldlowerBound,oldupperBound;

//********************(Exception processing)**************************

extern int exceptionflag1,exceptionflag2;

//********************(Ramp recognition)**************************/
extern int ramp, downhillflag, uphillflag;
extern float leftSlope,rightSlope;
extern float leftRampBound[60],rightRampBound[60];
extern int downhillDelay;

/******************** Ring direction control Kp ***************************/
extern float Kp3;

/******************** DIP switch ***************************/
extern int DIPswitch_1,DIPswitch_2,DIPswitch_3,DIPswitch_4,DIPswitch_5,DIPswitch_6,DIPswitch_7,DIPswitch_8;
extern int Kp3_Level;
extern float Kp3_Set[4];
extern int SpeedLevel,SpeedSet[8];
extern int rampSet;
extern int ringEnter_R;
extern int yuanhuanchengxu;
extern int right_high,left_high;
