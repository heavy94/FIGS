/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include <stdio.h>
#include <math.h>
#include <mpu6050.h>

#define AVG_SIZE 10
#define ACCELEROMETER_SENSITIVITY 16384
#define GYROSCOPE_SENSITIVITY 131
#define dt 0.01
#define MECH_WIN 0

#define LOCK 40
#define UNLOCK 60
#define FINGER_UNLOCK_CAP_0 100
#define FINGER_UNLOCK_CAP_1 100
#define FINGER_UNLOCK_CAP_2 100
#define FINGER_UNLOCK_CAP_3 100

int32 movmean_vector_0[AVG_SIZE] = {0};
int32 movmean_vector_1[AVG_SIZE] = {0};
int32 movmean_vector_2[AVG_SIZE] = {0};
int32 movmean_vector_3[AVG_SIZE] = {0};


int16_t absolute(int16_t x);
void Control_Servo(uint8_t Servo, uint8_t Control_sig);
void Calibrate_Fingers(int32 Max_Cap[4], int32 Min_Cap[4]);
void ReadFingerCap (int32 Finger_Cap[4]);
void GetFingerAngle(int32 Angle_data[4], int32 Max_Cap[4],int32 Min_Cap[4]);
void UARTReadString (uint8 Locked_Finger[4]);
uint8 Tip_Locked(int8_t Tip);
void Lock_Fingers(uint8 Locked_Finger[4]);
void Unlock_Fingers(uint8 Locked_Finger[4]);
void GetIMUData(int16_t* acc, int16_t* gyr);
void GetFilteredAngle(int16_t accData[3], int16_t gyrData[3], int16_t *p, int16_t *r, int16_t *y);
uint8 ButtonStatus(uint8 ButtonStat);



int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    CyDelay(1000);
    BT_Start();
    CapSense_Start();
    EZI2C_Start();
    EZI2C_EzI2CSetBuffer1(sizeof(CapSense_dsRam), sizeof(CapSense_dsRam), (uint8_t *)&(CapSense_dsRam));
    Servo_PWM_0_Start();
    Servo_PWM_1_Start();
    Servo_PWM_2_Start();
    Servo_PWM_3_Start();
   // Tip_Sense_ADC_Start();
    I2C_MPU6050_Start();
    MPU6050_init();
	MPU6050_initialize();
    
    int16_t Acc_Data[3] ={0}, Gyr_Data[3]={0};
    int16_t pitch=0, yaw=0, roll=0;
    int32 Finger_angle[4]={0};
    int32 Max_Cap[4]={0};
    int32 Min_Cap[4]={0};
    uint8 Locked_Finger[4]={0};
    int16_t ax, ay, az;
	int16_t gx, gy, gz;
    char buf[50];
    
    CyPins_SetPin(SERVO_LED_0_0);
    CyPins_SetPin(SERVO_LED_1_0);
    CyPins_SetPin(SERVO_LED_2_0);
    CyPins_SetPin(SERVO_LED_3_0);
    
    CyPins_SetPin(STATUS_LED_0);
    CyPins_SetPin(STATUS_LED_1);
    CyDelay(1000);
    
    Calibrate_Fingers(Max_Cap, Min_Cap);
    
    for(;;)
    {
        GetIMUData(Acc_Data, Gyr_Data); //Get IMU Data (Acc,Gyr)
        GetFilteredAngle(Acc_Data, Gyr_Data, &pitch, &roll, &yaw); //Complimentary filter for the angles
        
        if (CapSense_NOT_BUSY == CapSense_IsBusy())
        {
            GetFingerAngle(Finger_angle, Max_Cap ,Min_Cap); //Get finger angle based on calibration and current capacitance
        }
        
        uint8 but = CyPins_ReadPin(Buttons_0);
        but = (but == 0) ? 1 : 0;
        sprintf(buf,"%d,%d,%d;%ld,%ld,%ld,%ld;%u\n", pitch ,roll ,yaw, Finger_angle[0], Finger_angle[1], Finger_angle[2], Finger_angle[3], but);
        BT_PutString(buf);
        UARTReadString(Locked_Finger);
        //CyDelay(10); //Read UART for Lock commands
        Lock_Fingers(Locked_Finger); //Lock fingers
        Unlock_Fingers(Locked_Finger); //Unlock Fingers
        
        //sprintf(buf,"%lu,%lu,%lu,%lu\r\n", Finger_angle[0],Finger_angle[1],Finger_angle[2],Finger_angle[3]);
        //BT_PutString(buf);
        //Control_Servo(0, LOCK);
        //CyDelay(1000);
        //Control_Servo(0,UNLOCK);

       
    }
}

int16_t absolute(int16_t x)
{
    if(x < 0) return -x;
    else return x;
}

void Control_Servo(uint8_t Servo, uint8_t Control_sig){
    
   uint32_t Servo_sig = 0;
    Servo_sig = (uint32_t)Control_sig*20 + 500;
    
    switch(Servo)
    {
        case 1:
            Servo_PWM_0_WriteCompare(Servo_sig);
            break;
            
        case 2:
            Servo_PWM_1_WriteCompare(Servo_sig);
            break;
            
        case 3:
            Servo_PWM_2_WriteCompare(Servo_sig + 1);
            break;
            
        case 4:
            Servo_PWM_3_WriteCompare(Servo_sig + 1);
            break;
            
        case 0:
            Servo_PWM_0_WriteCompare(Servo_sig);
            Servo_PWM_1_WriteCompare(Servo_sig);
            Servo_PWM_2_WriteCompare(Servo_sig);
            Servo_PWM_3_WriteCompare(Servo_sig);     
            break;
        default: {}
    }
    
}

void Calibrate_Fingers(int32 Max_Cap[4], int32 Min_Cap[4]) {
    int i = 0;
    
    CyPins_ClearPin(STATUS_LED_0);
    CyDelay(5000);
    CapSense_ScanAllWidgets();
    
    while (i < 200)
    {
        while (CapSense_NOT_BUSY != CapSense_IsBusy());
        ReadFingerCap(Max_Cap);
    
        i++;
    }
    
    i = 0;
    
    while (i < AVG_SIZE)
    {
        while (CapSense_NOT_BUSY != CapSense_IsBusy());
        ReadFingerCap(Max_Cap);
    
        i++;
    }
    
    CyPins_SetPin(STATUS_LED_0);
    
    CyPins_ClearPin(STATUS_LED_1);
    CyDelay(5000);
    CapSense_ScanAllWidgets();
    
    i = 0;
    while (i < 200)
    {
        while (CapSense_NOT_BUSY != CapSense_IsBusy());
        ReadFingerCap(Min_Cap);
    
        i++;
    }
    
    i = 0;    
    while (i < AVG_SIZE)
    {
        while (CapSense_NOT_BUSY != CapSense_IsBusy());
        ReadFingerCap(Min_Cap);
        i++;
    }
    
    CyPins_SetPin(STATUS_LED_1);
}

void ReadFingerCap (int32 Finger_Cap[4])
{
    uint32 raw_count0 = 0, raw_count1 = 0, raw_count2 = 0, raw_count3 = 0;
    uint32 mean0 = 0, mean1 = 0, mean2 = 0, mean3 = 0;
    
    CapSense_ProcessAllWidgets();
    CapSense_RunTuner();

    CapSense_GetParam(CapSense_PROXIMITY0_SNS0_RAW0_PARAM_ID, &raw_count0);
    CapSense_GetParam(CapSense_PROXIMITY1_SNS0_RAW0_PARAM_ID, &raw_count1);
    CapSense_GetParam(CapSense_PROXIMITY2_SNS0_RAW0_PARAM_ID, &raw_count2);   
    CapSense_GetParam(CapSense_PROXIMITY3_SNS0_RAW0_PARAM_ID, &raw_count3);
    
    for (uint32 i = AVG_SIZE - 1; i >= 1; i--)
    {
        movmean_vector_0[i] = movmean_vector_0[i - 1];
        movmean_vector_1[i] = movmean_vector_1[i - 1];
        movmean_vector_2[i] = movmean_vector_2[i - 1];
        movmean_vector_3[i] = movmean_vector_3[i - 1];
    }
    
    movmean_vector_0[0] = raw_count0;
    movmean_vector_1[0] = raw_count1;
    movmean_vector_2[0] = raw_count2;
    movmean_vector_3[0] = raw_count3;
    
    for (uint32 i = 0; i < AVG_SIZE; i++)
    {
        mean0 += movmean_vector_0[i];
        mean1 += movmean_vector_1[i];
        mean2 += movmean_vector_2[i];
        mean3 += movmean_vector_3[i];
    }
    
    mean0 /= AVG_SIZE;
    mean1 /= AVG_SIZE;
    mean2 /= AVG_SIZE;
    mean3 /= AVG_SIZE;
    
    Finger_Cap[0] = mean0;
    Finger_Cap[1] = mean1;
    Finger_Cap[2] = mean2;
    Finger_Cap[3] = mean3;
    
    CapSense_ScanAllWidgets();
}


void GetFingerAngle(int32 Angle_data[4], int32 Max_Cap[4],int32 Min_Cap[4]){
    
    int32 Cap_Rawdata[4];
    
    ReadFingerCap(Cap_Rawdata);
    
    if (Cap_Rawdata[0] < Min_Cap[0])
    {
        Angle_data[0] = 0;
        
    } else
    {
        Angle_data[0] = (-90 * (Cap_Rawdata[0] - Min_Cap[0]))/(Max_Cap[0]- Min_Cap[0]);
    }
    
    if (Cap_Rawdata[1] < Min_Cap[1])
    {
        Angle_data[1] = 0;
        
    } else
    {
        Angle_data[1] = (-90 * (Cap_Rawdata[1] - Min_Cap[1]))/(Max_Cap[1]- Min_Cap[1]);
    }
    
    if (Cap_Rawdata[2] < Min_Cap[2])
    {
        Angle_data[2] = 0;
        
    } else
    {
        Angle_data[2] = (-90 * (Cap_Rawdata[2] - Min_Cap[2]))/(Max_Cap[2]- Min_Cap[2]);
    }
    
    if (Cap_Rawdata[3] < Min_Cap[3])
    {
        Angle_data[3] = 0;
        
    } else
    {
        Angle_data[3] = (-90 * (Cap_Rawdata[3] - Min_Cap[3]))/(Max_Cap[3]- Min_Cap[3]);
    }
}

void UARTReadString (uint8 Locked_Finger[4])
{
        while (BT_GetRxBufferSize() > 0)
        {
            char c = BT_GetChar();
                switch(c)
                {
                    case '1':
                        Locked_Finger[0] = 1;
                        break;
                        
                    case '2':
                        Locked_Finger[1] = 1;
                        break;
                        
                    case '3':
                        Locked_Finger[2] = 1;
                        break;
                        
                    case '4':
                        Locked_Finger[3] = 1;
                        break;
                        
                    case 'Y':
                        CyPins_SetPin(VIBRATION_0);
                        break;
                        
                    case 'N':
                        CyPins_ClearPin(VIBRATION_0);
                        break;
                        
                    default: {}   
                }
                
        }
}

void Lock_Fingers(uint8 Locked_Finger[4])
{
    
    if (Locked_Finger[0] == 1)
    {
        Locked_Finger[0] = 0;
        Control_Servo(1,LOCK);
        //CyPins_ClearPin(SERVO_LED_0_0);
    }
    if (Locked_Finger[1] == 1)
    {
        Locked_Finger[1] = 0;
        Control_Servo(2,LOCK);
        //CyPins_ClearPin(SERVO_LED_1_0);
    }
    if (Locked_Finger[2] == 1)
    {
        Locked_Finger[2] = 0;
        Control_Servo(3,LOCK);
        //CyPins_ClearPin(SERVO_LED_2_0);
    }
    if (Locked_Finger[3] == 1)
    {
        Locked_Finger[3] = 0;
        Control_Servo(4,LOCK);
        //CyPins_ClearPin(SERVO_LED_3_0);
    }
    
}
void Unlock_Fingers(uint8 Locked_Finger[4])
{
    /*int32 Cap_Unlock[4];
    
    CapSense_ScanAllWidgets();
    while (CapSense_NOT_BUSY == CapSense_IsBusy());
    ReadFingerCap(Cap_Unlock);
    
    if(Locked_Finger[0] && (Locked_Finger_Cap[0] - Cap_Unlock[0]) > FINGER_UNLOCK_CAP_0)
    {
        Locked_Finger[0] = 0;
        Control_Servo(1,UNLOCK); 
    }
    
    if(Locked_Finger[1] && (Locked_Finger_Cap[1] - Cap_Unlock[1]) > FINGER_UNLOCK_CAP_1)
    {
        Locked_Finger[1] = 0;
        Control_Servo(2,UNLOCK); 
    }
    
    if(Locked_Finger[2] && (Locked_Finger_Cap[2] - Cap_Unlock[2]) > FINGER_UNLOCK_CAP_2)
    {
        Locked_Finger[2] = 0;
        Control_Servo(3,UNLOCK); 
    }
    
    if(Locked_Finger[3] && (Locked_Finger_Cap[3] - Cap_Unlock[3]) > FINGER_UNLOCK_CAP_3)
    {
        Locked_Finger[3] = 0;
        Control_Servo(4,UNLOCK); 
    }
    */
    uint8 but = CyPins_ReadPin(Buttons_1);
    but = (but == 0) ? 1 : 0;
    if(but)
    {
        Locked_Finger[0] = 0;
        Control_Servo(1,UNLOCK);
        
        Locked_Finger[1] = 0;
        Control_Servo(2,UNLOCK);
        
        Locked_Finger[2] = 0;
        Control_Servo(3,UNLOCK);
        
        Locked_Finger[3] = 0;
        Control_Servo(4,UNLOCK);
        
        //CyPins_SetPin(SERVO_LED_0_0);
        //CyPins_SetPin(SERVO_LED_1_0);
        //CyPins_SetPin(SERVO_LED_2_0);
        //CyPins_SetPin(SERVO_LED_3_0);
    }
}

void GetIMUData(int16_t* acc, int16_t* gyr)
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

void GetFilteredAngle(int16_t accData[3], int16_t gyrData[3], int16_t *p, int16_t *r, int16_t *y)
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
        yaw = yaw * 1 + yawAcc*0;
        
        if((int)roll%180 < MECH_WIN && (int)roll%180 > -MECH_WIN)
        {
           *r = 0; 
        } else {
            
            *r = (int)roll%180;
        }
        
        if((int)pitch%180 < MECH_WIN && (int)pitch%180 > -MECH_WIN)
        {
           *p = 0; 
        } else {
            
            *p = (int)pitch%180;
        }
        
        
        if((int)yaw%180 < MECH_WIN && (int)yaw%180 > -MECH_WIN)
        {
           *y = 0; 
        } else {
            
            *y = (int)yaw%180;
        }
        
    }
} 
/* [] END OF FILE */
