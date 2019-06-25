//extern float Gyro_Now, angle_offset_vertical;  //Normalized angular velocity acquired by the gyroscope, Acceleration angle after conversion
//extern float g_fCarAngle,g_fGyroscopeAngleIntegral; // final angle
//extern volatile float  Speed_L,Speed_R,speed_Start,Speed_L_Last,Speed_R_Last;  // Left and right wheel speed, final speed
//extern volatile int	MMA7361 ,ENC03;             // accelerator AD, Gyro AD

void Rd_Ad_Value(void);
void QingHua_AngleCalaulate(float G_angle,float Gyro);
extern void AD_Calculate(void);
extern void AngleControlOut(float angle,float angle_dot);
