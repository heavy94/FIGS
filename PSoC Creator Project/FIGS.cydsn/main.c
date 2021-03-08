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
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include <mpu6050.h>

#define AVG_SIZE 40
#define ACCELEROMETER_SENSITIVITY 16384
#define GYROSCOPE_SENSITIVITY 131
#define dt 0.01

uint32 count = 0;

int32 movmean = 0;
int32 movmean_vector[AVG_SIZE];

void Control_Servo(int8_t Servo, int8_t Control_sig){
    
    int8_t Servo_sig = 0;
    Servo_sig = Control_sig*20 + 1000;
    
    switch(Servo)
    {
        case 1:
            Servo_PWM_0_WriteCompare(Servo_sig);
            SERVO_LED_0_Write(!SERVO_LED_0_Read());
            break;
            
        case 2:
            Servo_PWM_1_WriteCompare(Servo_sig);
            SERVO_LED_1_Write(!SERVO_LED_1_Read());
            break;
            
        case 3:
            Servo_PWM_2_WriteCompare(Servo_sig);
            SERVO_LED_2_Write(!SERVO_LED_2_Read());
            break;
            
        case 4:
            Servo_PWM_3_WriteCompare(Servo_sig);
            SERVO_LED_3_Write(!SERVO_LED_3_Read());
            break;
            
        default:
            Servo_PWM_0_WriteCompare(Servo_sig);
            Servo_PWM_1_WriteCompare(Servo_sig);
            Servo_PWM_2_WriteCompare(Servo_sig);
            Servo_PWM_3_WriteCompare(Servo_sig);          
    }
    
}

uint32 CapSense(int8_t Cap){
    
    uint32 Cap_data = 0;
    CapSense_InitializeAllBaselines();
    CapSense_ScanAllWidgets();
    
    switch(Cap)
    {
        case 1:
            CapSense_GetParam(CapSense_PROXIMITY0_SNS0_RAW0_PARAM_ID, &Cap_data);
            break;
            
        case 2:
            CapSense_GetParam(CapSense_PROXIMITY1_SNS0_RAW0_PARAM_ID, &Cap_data);
            break;
            
        case 3:
            CapSense_GetParam(CapSense_PROXIMITY2_SNS0_RAW0_PARAM_ID, &Cap_data);
            break;
            
        case 4:
            CapSense_GetParam(CapSense_PROXIMITY3_SNS0_RAW0_PARAM_ID, &Cap_data);
            break;
            
        default:
            Cap_data = 0;
    }
    
    return Cap_data;
}
bool Tip_Locked(int8_t Tip){
    
    uint16 Tip_sense_data = 0;
    
    switch(Tip)
    {
        case 1:
            Tip_sense_data = Tip_Sense_ADC_GetResult16(0);          
            break;
            
        case 2:
            Tip_sense_data = Tip_Sense_ADC_GetResult16(1);    
            break;
            
        case 3:
            Tip_sense_data = Tip_Sense_ADC_GetResult16(2);    
            break;
            
        case 4:
            Tip_sense_data = Tip_Sense_ADC_GetResult16(3);    
            break;
            
        default:
            Tip_sense_data = 0;
    }
    
    if(Tip_sense_data > 0)
    {
        return true;
    } else { return false; };
}

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

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    CyDelay(1000);
    BT_Start();
    CapSense_Start();
    Servo_PWM_0_Start();
    Servo_PWM_1_Start();
    Servo_PWM_2_Start();
    Servo_PWM_3_Start();
    Tip_Sense_ADC_Start();
    I2C_MPU6050_Start();
    int16_t acc[3]={0}, gyr[3]={0};
    int16_t pitch, yaw, roll;
    MPU6050_init();
	MPU6050_initialize();
    
    
    
    
    
    
    /*
    // Capacitance measurement
    CyDelay(3000);
    
    while(1)
    {
        uint32 cp = CapSense_GetSensorCapacitance(CapSense_PROXIMITY0_WDGT_ID, CapSense_PROXIMITY0_SNS0_ID);
        uint32 cm = CapSense_GetExtCapCapacitance(CapSense_TST_CMOD_ID);
        char buffer[100] = "";
        sprintf(buffer, "\nCp = %lupF\nCmod = %lupF\n", cp, cm);
        UART_UartPutString(buffer);
        CyDelay(1000);
    }
    UART_Stop();
    */
    
    for (uint32 i = 0; i < AVG_SIZE; i++) movmean_vector[i] = 0;
    Tuner_Start(); /* Start EZI2C Component */
    /*
    * Set up communication and initialize data buffer to CapSense data structure
    * to use Tuner application
    */
    Tuner_EzI2CSetBuffer1(sizeof(CapSense_dsRam), sizeof(CapSense_dsRam), (uint8_t *)&(CapSense_dsRam));
    CapSense_Start(); /* Initialize Component */

    for(;;)
    {
        getData(acc, gyr);
        getGyroAngle(acc, gyr, &pitch, &roll, &yaw);
        
        /* Do this only when a scan is done */
        if(CapSense_NOT_BUSY == CapSense_IsBusy())
        {
            CapSense_ProcessAllWidgets(); /* Process all widgets */
            CapSense_RunTuner(); /* To sync with Tuner application */
            
            uint32 raw_count = 0;
            CapSense_GetParam(CapSense_PROXIMITY0_SNS0_RAW0_PARAM_ID, &raw_count);
            //raw_count /= 40;
            
            //movmean = movmean + (((int32)raw_count - movmean_vector[AVG_SIZE - 1]) / AVG_SIZE);
            for (uint32 i = AVG_SIZE - 1; i >= 1; i--)
            {
                movmean_vector[i] = movmean_vector[i - 1];
            }
            movmean_vector[0] = (int32)raw_count;
            movmean = 0;
            for (uint32 i = 0; i < AVG_SIZE; i++)
            {
                movmean += movmean_vector[i];
            }
            movmean /= AVG_SIZE;
            //if (count >= 1000)
            //{
                char buffer[20] = "";
                sprintf(buffer, "%lu,4100,2100\n", movmean);
                BT_PutString(buffer);
                count = 0;
            //}
            //else
            //{
            //    count++;
            //}
            
            
            /*
            if (CapSense_IsAnyWidgetActive()) // Scan result verification
            {
                // add custom tasks to execute when touch detected
                if (prev_state == 0u)
                {
                    prev_state = 1u;
                    On_Detect_Write(1u);
                    Timer_1_Enable();
                }
            }
            else
            {
                if (prev_state == 1u)
                {
                    prev_state = 0u;
                    On_Release_Write(1u);
                    Timer_1_Enable();
                }
            }
            */
            CapSense_ScanAllWidgets(); /* Start next scan */
        }
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
