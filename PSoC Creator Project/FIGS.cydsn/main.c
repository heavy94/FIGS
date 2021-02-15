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

#define AVG_SIZE 40

uint32 count = 0;

int32 movmean = 0;
int32 movmean_vector[AVG_SIZE];

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    UART_Start();
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
    
    CapSense_ScanAllWidgets(); /* Scan all widgets */

    for(;;)
    {
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
                UART_UartPutString(buffer);
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
