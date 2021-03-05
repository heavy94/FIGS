#include "project.h"
#include <mpu6050.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>

int16_t Counter_X = 0, Counter_Y = 0, Counter_Z = 0;

int32_t average_aX [2] = {0};
int32_t average_aY [2] = {0};
int32_t average_aZ [2] = {0};

int32_t velocity_X [2] ={0};
int32_t velocity_Y [2] = {0};
int32_t velocity_Z [2] = {0};

int32_t position_X [2] = {0};
int32_t position_Y [2] = {0};
int32_t position_Z [2] = {0};

#define ACCELEROMETER_SENSITIVITY 16384
#define GYROSCOPE_SENSITIVITY 131 

//#define M_PI 3.14159265359	    

#define dt 0.01							// 10 ms sample rate!    

int16_t absolute(int16_t x)
{
    if(x < 0) return -x;
    else return x;
}

void getData(int16_t* acc, int16_t* gyr)
{
    unsigned int count1;
    count1 = 0;
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
    int64_t acc_xg=0,acc_yg=0,acc_zg = 0;
    int64_t acc_x=0, acc_y=0, acc_z = 0;
    
    while(count1<=32){
        MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        acc_x = acc_x + ax; // Accumulate Samples
        acc_y = acc_y + ay;
        acc_z = acc_z + az;
        acc_xg = acc_xg + gx; // Accumulate Samples
        acc_yg = acc_yg + gy;
        acc_zg = acc_zg + gz;
        count1++;
    }// 32 times
    gyr[0] = acc_xg>>5; // division by 32 Averaging
    gyr[1] = acc_yg>>5;
    gyr[2] = acc_zg>>5;
    acc[0] = acc_x>>5; // division by 32 Averaging
    acc[1] = acc_y>>5;
    acc[2] = acc_z>>5;
    
    
}

void getGyroAngle(int16_t accData[3], int16_t gyrData[3], int16_t *p, int16_t *r, int16_t *y)
{
    float pitchAcc, rollAcc, yawAcc;
    float pitch = *p, roll = *r, yaw = *y;               
 
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    roll += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
    pitch -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
    yaw += ((float)gyrData[2] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Z-axis
    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = absolute(accData[0]) + absolute(accData[1]) + absolute(accData[2]);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
	// Turning around the X axis results in a vector on the Y-axis
        rollAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
        roll = roll * 0.85 + rollAcc * 0.15;
 
	// Turning around the Y axis results in a vector on the X-axis
        pitchAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
        pitch = pitch * 0.85 + pitchAcc * 0.15;

    // Turning around the Y axis results in a vector on the X-axis
        yawAcc = atan2f((float)accData[0], (float)accData[1]) * 180 / M_PI;
        yaw = yaw * 0.85 + yawAcc * 0.15;
        
        *r = (int)roll%180;
        *p = (int)pitch%180;
        *y = (int)yaw%180;
    }
} 

void minusGravity(int16_t* acc, int16_t pitch, int16_t roll)
{   
    //Convert angle to radians
    float p = (pitch) * (M_PI/180);
    float r = (roll) * (M_PI/180);
    
    acc[0] = acc[0] - (sinf(p)*ACCELEROMETER_SENSITIVITY);
    acc[1] = acc[1] - (sinf(r)*ACCELEROMETER_SENSITIVITY);
    acc[2] = acc[2] - (cosf(r)*cosf(p)*ACCELEROMETER_SENSITIVITY);
    
    //Changing acc range from -1000 to 1000
    acc[0] = acc[0]*1000/ACCELEROMETER_SENSITIVITY;
    acc[1] = acc[1]*1000/ACCELEROMETER_SENSITIVITY;
    acc[2] = acc[2]*1000/ACCELEROMETER_SENSITIVITY;
    /*
    char buf[50];
    sprintf(buf, "AX:%d, AY:%d AZ:%d",(int)acc[0], (int)acc[1], (int)acc[2]);
    SERIAL_UartPutString(buf);
    SERIAL_UartPutString("\n\r");*/
}

void Movement_Check(int32_t ax, int32_t ay, int32_t az)
{
    if(ax == 0)Counter_X++;
    else {Counter_X = 0;}
    
    if(ay == 0)Counter_Y++;
    else {Counter_Y = 0;}
    
    if(az == 0)Counter_Z++;
    else {Counter_Z = 0;}
    
    if(Counter_X >= 20)
    {
        velocity_X[0] = 0;
        velocity_X[1] = 0;
    }
    
    if(Counter_Y >= 20)
    {
        velocity_Y[0] = 0;
        velocity_Y[1] = 0;
    }
    
    if(Counter_Z >= 20)
    {
        velocity_Z[0] = 0;
        velocity_Z[1] = 0;
    }
}

int calculateVar(int16_t* arr){
    int mean = (arr[0]+arr[1]+arr[2]+arr[3]+arr[4])/5;
    int val1 = arr[0]-mean;
    int val2 = arr[1]-mean;
    int val3 = arr[2]-mean;
    int val4 = arr[3]-mean;
    int val5 = arr[4]-mean;
    int var = (val1*val1 + val2*val2 + val3*val3 + val4*val4 + val5*val5)/5;
    return var;
}

int Acc_Constant(int16_t* acc, int16_t* prevAx, int16_t* prevAy, int16_t* prevAz, int index)
{
    //Make acc and velocity equal to zero, if acc is constant
    prevAx[index] = acc[0];
    prevAy[index] = acc[1];
    prevAz[index] = acc[2];
    index ++;
    index = index%5;
    
    int varX = calculateVar(prevAx);
    int varY = calculateVar(prevAy);
    int varZ = calculateVar(prevAz);
    
    if(varX<100) {
        acc[0] = 0; velocity_X[0] = 0; velocity_X[1] = 0;
    }
    
    if(varY<100)
    {
        acc[1] = 0; velocity_Y[0] = 0; velocity_Y[1] = 0;
    }
    
    if(varZ<100)
    {
        acc[2] = 0; velocity_Z[0] = 0; velocity_Z[1] = 0;
    }
    return index;
}

void position(int16_t* acc)
{
    float t = 0.05;
    //Timer[1]= TimerMilli;
    
    average_aX[1]= acc[0]; //- Offset_x;
    average_aY[1]= acc[1]; //- Offset_y ;
    average_aZ[1]= acc[2]; //- Offset_z;
    
    //Mehcnaical Window
    
   if((average_aX[1] >= -10) &&  (average_aX[1] <= 10))
    {
        average_aX[1] = 0;
    }
    
    if((average_aY[1] >= -10) &&  (average_aY[1] <= 10))
    {
        average_aY[1] = 0;
    }
    
    if((average_aZ[1] >= -10) &&  (average_aZ[1] <= 10))
    {
        average_aY[1] = 0;
    }
    Movement_Check(average_aX[1],average_aY[1],average_aZ[1]);
    
    velocity_X[1]= velocity_X[0]+ (average_aX[0]+ ((average_aX[1] - average_aX[0])>>1))*t;//*(Timer[1]-Timer[0])
    velocity_Y[1]= velocity_Y[0]+ (average_aY[0]+ ((average_aY[1] - average_aY[0])>>1))*t;//*(Timer[1]-Timer[0]); 
    velocity_Z[1]= velocity_Z[0]+ (average_aZ[0]+ ((average_aZ[1] - average_aZ[0])>>1))*t;//*(Timer[1]-Timer[0]);
    
    
    position_X[1]= position_X[0]+ (velocity_X[0]+ ((velocity_X[1] - velocity_X[0])>>1))*t;//*(Timer[1]-Timer[0]);
    position_Y[1]= position_Y[0]+ (velocity_Y[0]+ ((velocity_Y[1] - velocity_Y[0])>>1))*t;//*(Timer[1]-Timer[0]);
    position_Z[1]= position_Z[0]+ (velocity_Z[0]+ ((velocity_Z[1] - velocity_Z[0])>>1))*t;//*(Timer[1]-Timer[0]); 
    
    average_aX[0]= average_aX[1];
    average_aY[0]= average_aY[1];
    average_aZ[0]= average_aZ[1];
    
    velocity_X[0]= velocity_X[1];
    velocity_Y[0]= velocity_Y[1];
    velocity_Z[0]= velocity_Z[1];
    
    position_X[0]= position_X[1];
    position_Y[0]= position_Y[1];
    position_Z[0]= position_Z[1];
    
    
    /*sprintf(buf, "AX:%d, AY:%d, AZ:%d",(int)average_aX[1], (int)average_aY[1], (int)average_aZ[1]);
    SERIAL_UartPutString(buf);
    SERIAL_UartPutString("\n\r");
    sprintf(buf, "VX:%d, VY:%d, VZ:%d",(int)velocity_X[1], (int)velocity_Y[1], (int)velocity_Z[1]);
    SERIAL_UartPutString(buf);
    SERIAL_UartPutString("\n\r");*/
    /*sprintf(buf, "PX:%d, PY:%d, PZ:%d",(int)position_X[1], (int)position_Y[1], (int)position_Z[1]);
    SERIAL_UartPutString(buf);
    SERIAL_UartPutString("\n\r");*/
       
}

int main()
{
    char buf[50];
    int index = 0;
    int16_t acc[3]={0}, gyr[3]={0};
    int16_t prevAx[10]={0}, prevAy[10]={0}, prevAz[10]={0};
    int16_t pitch, yaw, roll;
    CyGlobalIntEnable;
    CyDelay(1000);
	I2C_MPU6050_Start();
	SERIAL_Start();
    
	MPU6050_init();
	MPU6050_initialize();
    SERIAL_UartPutString(MPU6050_testConnection() ? "MPU6050 connection successful\n\r" : "MPU6050 connection failed\n\n\r");
    
    while(1)
    {
        getData(acc, gyr);
        getGyroAngle(acc, gyr, &pitch, &roll, &yaw);
        minusGravity(acc,pitch,roll);
        index = Acc_Constant(acc,prevAx,prevAy,prevAz,index);
        position(acc);
        sprintf(buf,"Pitch:%d  Roll:%d  Yaw:%d\t Px:%d PY:%d PZ:%d", pitch, roll, yaw, (int)position_X[1]%1000, (int)position_Y[1]%1000, (int)position_Z[1]%1000);
        SERIAL_UartPutString(buf);
        SERIAL_UartPutString("\n\r");
    }
}