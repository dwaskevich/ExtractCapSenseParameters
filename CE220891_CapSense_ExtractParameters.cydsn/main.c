/*****************************************************************************
* File Name: main.c
*
* Version: 1.00
*
* Update - 12-Aug-2021 (David Waskevich) --> open circuit test using Cp array in RAM
*
* Update - 17-Aug-2021 (David Waskevich) --> adding Cp calculations using scan parameters
*
* Update - 18-Aug-2021 (David Waskevich) --> added check for scan clock source which
*                                            appeard to be responsible for 2x error in Cp
*
* Update - 19-Aug-2021 (David Waskevich) --> created function to calculate Cp
*
* Update -  1-Oct-2021 (David Waskevich) --> adding scan parameter printing
*
* Update -  2-Oct-2021 (David Waskevich) --> created function to extract parameters
*
* Description: This code example demonstrates the use of a CapSense Slider and 
* Buttons using PSoC 4100S Plus Device. As soon as the user touches the button
* respective LED will glow. When user touches the slider, LEDs up to the relative
* position glows. Touching CapSense buttons and slider for about 2 sec will auto-reset
* the sensor, turning off LEDs. Apart from that there is a breathing LED which keeps 
* glowing on the User LED which has been implemented using SmartIO.
*
* Related Document: CE220891_CapSense_with_Breathing_LED.pdf
*
* Hardware Dependency: CY8CKIT-149 PSoC 4100S Plus Prototyping Kit
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and 
* foreign), United States copyright laws and international treaty provisions. 
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the 
* Cypress source code and derivative works for the sole purpose of creating 
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited 
* without the express written permission of Cypress.
* 
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products 
* where a malfunction or failure may reasonably be expected to result in 
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's 
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*****************************************************************************/
/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include "CapSense.h"
#include <project.h>
#include <stdio.h>
#include "calculateCp.h"
#include "extractParams.h"

/*****************************************************************************
* MACRO Definitions
*****************************************************************************/     

/* Boolean constants */
#define LED_ON						(1u)
#define LED_OFF						(0u)
#define FALSE                       (0u)
#define TRUE                        (1u)

/*Set the macro value to '1' to use tuner for debugging and tuning CapSense sensors
  Set the macro value to '0' to disable the tuner*/
#define ENABLE_TUNER                (1u)

/* NOTE - Sensor_Cp test requires CapSense BIST library. Turn off SmartSense and enable BIST. */

/* define register map that will be exposed to I2C host */
struct RegisterMap {
    uint16 CmodValue; /* holds measured value of Cmod capacitor on P4[1] */
	uint16 CycleCount; /* holds I2C Master-read cycle count */
    uint16 SensorCp_BIST[CapSense_TOTAL_CSD_SENSORS]; /* holds sensor Cp values calculated using BIST CapSense_GetSensorCapacitance API */
    uint16 SensorCp_Calculated[CapSense_TOTAL_CSD_SENSORS]; /* holds calculated sensor Cp values */
    uint8 EndOfBuffer; /* arbitrary marker */
} RegisterMap;

/* construct look-up table for iDAC gain values */
uint16 static const GainValueLUT[] = {37u, 300u, 2400, 1200};

/* construct look-up table for widget type names */
static const char * WidgetTypeLUT[8] = {"null", "BTN", "LS", "RS", "MTX", "X-Y", "PROX", "ENC"};

/*
from CapSense_Configuration.h:

#define CapSense_CLK_SOURCE_DIRECT            (0x00000000Lu)

#define CapSense_CLK_SOURCE_SSC1              (0x01u)
#define CapSense_CLK_SOURCE_SSC2              (0x02u)
#define CapSense_CLK_SOURCE_SSC3              (0x03u)
#define CapSense_CLK_SOURCE_SSC4              (0x04u)

#define CapSense_CLK_SOURCE_PRS8              (0x05u)
#define CapSense_CLK_SOURCE_PRS12             (0x06u)
#define CapSense_CLK_SOURCE_PRSAUTO           (0xFFu)

*/
/* construct look-up table for sense clock source */
static const char * SnsClkSourceLUT[8] = {"Direct", "SSC1", "SSC2", "SSC3", "SSC4", "PRS8", "PRS12", "PRSAuto"};

/* Finite state machine for device operating states 
    SENSOR_SCAN - Sensors are scanned in this state
    WAIT_FOR_SCAN_COMPLETE - CPU is put to sleep in this state
    PROCESS_DATA - Sensor data is processed, LEDs are controlled,
                   and I2C buffer is updated in this state */
typedef enum
{
    SENSOR_SCAN = 0x01u,                
    WAIT_FOR_SCAN_COMPLETE = 0x02u,     
    PROCESS_DATA = 0x03u,               
} DEVICE_STATE;

int main()
{	        
    widgetParameters_struct djjw_capsense_parameters;
//    uint8 loopCount;
	CapSense_FLASH_WD_STRUCT const *ptrFlashWdgt; /* pointer to widget array in Flash */
    CapSense_RAM_STRUCT *ptrRamStruct; /* pointer to dsRAM structure */
//    CapSense_RAM_WD_LIST_STRUCT *ptrWidgetList;
//    CapSense_TST_MEASUREMENT_STATUS_ENUM measurementStatus; /* variable to pass to CapSense_GetSensorCapacitance API */
    uint32 status; /* EZI2C function call status return */
    
//    CapSense_FLASH_WD_STRUCT const *ptrFlashWdgtObj;
    
    char printBuf[100];
    char rxChar;
    uint8 printIndividual = FALSE;
    uint32 widgetID = 0;
    
    static uint8 widgetsFound = 0;
    
    /* Variable to hold the current device state 
    *  State machine starts with Sensor_Scan state after power-up
    */
    DEVICE_STATE currentState = SENSOR_SCAN;  
           
     /* Enable global interrupts. */
    CyGlobalIntEnable; 

    /* Start EZI2C block */
    EZI2C_Start();
    /* Start CapSense block */
    CapSense_Start();
    
    UART_Start();
    UART_UartPutString("\r\nUART started ...\r\n");
    
    #if ENABLE_TUNER
        /* Set up I2C communication data buffer with CapSense data structure 
        to be exposed to I2C master on a primary slave address request (Tuner) */
        EZI2C_EzI2CSetBuffer1(sizeof(CapSense_dsRam), sizeof(CapSense_dsRam),(uint8 *)&CapSense_dsRam);
    #endif
    
    /* Set up communication data buffer to be exposed to I2C master at secondary slave address (BCP). */
	EZI2C_EzI2CSetBuffer2(sizeof(RegisterMap), sizeof(RegisterMap), (uint8 *) &RegisterMap);

    RegisterMap.EndOfBuffer = 0x55; /* arbitrary value to mark end of buffer */
    
    ptrRamStruct = &CapSense_dsRam;
    sprintf(printBuf, "\nhwClock frequency from dsRAM = %u\r\n", ptrRamStruct->hwClock);
    UART_UartPutString(printBuf);
    sprintf(printBuf, "modClkDivider from dsRAM = %u\r\n\n", ptrRamStruct->modCsdClk);
    UART_UartPutString(printBuf);
    sprintf(printBuf, "Modulator clock freq (in kHz) from my function = %u\r\n", getModClkFreq());
    UART_UartPutString(printBuf);
    
    
    sprintf(printBuf, "\r\nCmod = %hu pF\r\n", (uint16) CapSense_GetExtCapCapacitance(CapSense_TST_CMOD_ID));
    UART_UartPutString(printBuf);
    sprintf(printBuf, "MODULATOR_CLK_FREQ = %u\r\n", MODULATOR_CLK_FREQ);
    UART_UartPutString(printBuf);
    sprintf(printBuf, "Number of widgets = %d, total number of sensors = %d.\r\n\n", CapSense_TOTAL_WIDGETS, CapSense_TOTAL_SENSORS);
    UART_UartPutString(printBuf);
    
    /* search for widgets in dsFlash structure */
//    do
//    {
//        ptrFlashWdgtObj = &CapSense_dsFlash.wdgtArray[widgetsFound];
//        widgetsFound++;
//    }while (ptrFlashWdgtObj->wdgtType <= CapSense_WD_ENCODERDIAL_E);
//    
//    widgetsFound -= 1; /* adjust count from do/while loop */
    if(CYRET_SUCCESS == getWidgetConfig(10, &widgetsFound))
    {
        sprintf(printBuf, "Widgets found = %d.\r\n\n", widgetsFound);
        UART_UartPutString(printBuf);
    }
    else
    {
        sprintf(printBuf, "No widgets found. Returned value = %d.\r\n\n", widgetsFound);
        UART_UartPutString(printBuf);
    }
    
    /* retrieve/print CapSense configuration */
    for(uint8 i = 0; i < widgetsFound; i++)
	{
		/* using structure pointer to access widget array */
		ptrFlashWdgt = &CapSense_dsFlash.wdgtArray[i]; /* use loop index "i" to point to current widget */
        sprintf(printBuf, "Widget %d has %d sensor(s)\r\n", i, ptrFlashWdgt->totalNumSns);
        UART_UartPutString(printBuf);
    }
    UART_UartPutString("\r\n");
    
    extractParams(CapSense_BTN0_WDGT_ID, CapSense_BTN0_SNS0_ID, &djjw_capsense_parameters);
//    extractParams(CapSense_LINEARSLIDER0_WDGT_ID, CapSense_LINEARSLIDER0_SNS3_ID, &djjw_capsense_parameters);

    UART_UartPutString("Scan parameters for sensor 0 ...\r\nmod\tcomp\traw\tsource\tdivider\tFsw_kHz\tgain\tres\tfinger\tnoise\tnNoise\tdeb\thys\tlowBsln\ttype\r\n");
    sprintf(printBuf, "%d\t%d\t%d\t%s\t%d\t%d\t%4d\t%2d\t%d\t%d\t%d\t%d\t%d\t%d\t%s\r\n\n",
        djjw_capsense_parameters.mod_iDAC_value, djjw_capsense_parameters.comp_iDAC_value,
        djjw_capsense_parameters.sensor_raw_count, SnsClkSourceLUT[djjw_capsense_parameters.snsClkSource],
        djjw_capsense_parameters.snsClkDiv, djjw_capsense_parameters.snsClkFreq,
        djjw_capsense_parameters.iDAC_gain, djjw_capsense_parameters.scanResolution,
        djjw_capsense_parameters.fingerThreshold, djjw_capsense_parameters.noiseThreshold,
        djjw_capsense_parameters.negativeNoiseThreshold, djjw_capsense_parameters.onDebounce,
        djjw_capsense_parameters.hysteresis, djjw_capsense_parameters.lowBslnRst,
        WidgetTypeLUT[djjw_capsense_parameters.wdgtType]);
    UART_UartPutString(printBuf);
    
    UART_UartPutString("\r\nPress <enter> to extract all parameters.\r\n\n");
     
    for(;;)
    {
        /* Switch between SENSOR_SCAN->WAIT_FOR_SCAN_COMPLETE->PROCESS_DATA states */
        switch(currentState)
        {
            case SENSOR_SCAN:
	            /* Initiate new scan only if the CapSense block is idle */
                if(CapSense_NOT_BUSY == CapSense_IsBusy())
                {
                    #if ENABLE_TUNER
                        /* Update CapSense parameters set via CapSense tuner before the 
                           beginning of CapSense scan 
                        */
                        CapSense_RunTuner();
                    #endif
                    
                    /* Scan widget(s) */
                    if(FALSE == printIndividual)
                        CapSense_ScanAllWidgets();
                    else
                    {
                        CapSense_SetupWidget(widgetID);                        
                        /* Scan widget configured by CSDSetupWidget API */
                        CapSense_Scan();
                    }
                                        
                    /* Set next state to WAIT_FOR_SCAN_COMPLETE  */
                    currentState = WAIT_FOR_SCAN_COMPLETE;
                }
                break;

            case WAIT_FOR_SCAN_COMPLETE:

                /* Put the device to CPU Sleep until CapSense scanning is complete*/
                if(CapSense_NOT_BUSY != CapSense_IsBusy())
                {
                    CySysPmSleep();
                }
                /* If CapSense scanning is complete, process the CapSense data */
                else
                {
                    currentState = PROCESS_DATA;
                }
                break;
        
            case PROCESS_DATA:
                
                /* Process data on all the enabled widgets */
                CapSense_ProcessAllWidgets();
                
                if(CapSense_IsAnyWidgetActive())
                    LED1_Write(LED_ON);
                else LED1_Write(LED_OFF);

                if(0 != UART_SpiUartGetRxBufferSize())
                {
                    UART_UartPutString("\r\nwidget\tsensor\tmod\tcomp\traw\tsource\tdivider\tFsw_kHz\tgain\tres\tfinger\tnoise\tnNoise\tdeb\thys\tlowBsln\ttype\r\n");
                    for(uint8 i = 0; i < widgetsFound; i++)
                	{
                		/* using structure pointer to access widget array */
                		ptrFlashWdgt = &CapSense_dsFlash.wdgtArray[i]; /* use loop index "i" to point to current widget */
                        for(uint8 j = 0; j < ptrFlashWdgt->totalNumSns; j++) /* process all sensors associated with current widget */
                		{
                			extractParams(i, j, &djjw_capsense_parameters);
                            sprintf(printBuf, "%d\t%d\t%d\t%d\t%d\t%s\t%d\t%d\t%4d\t%2d\t%d\t%d\t%d\t%d\t%d\t%d\t%s\r\n", i, j,
                                djjw_capsense_parameters.mod_iDAC_value, djjw_capsense_parameters.comp_iDAC_value,
                                djjw_capsense_parameters.sensor_raw_count, SnsClkSourceLUT[djjw_capsense_parameters.snsClkSource],
                                djjw_capsense_parameters.snsClkDiv, djjw_capsense_parameters.snsClkFreq,
                                djjw_capsense_parameters.iDAC_gain, djjw_capsense_parameters.scanResolution,
                                djjw_capsense_parameters.fingerThreshold, djjw_capsense_parameters.noiseThreshold,
                                djjw_capsense_parameters.negativeNoiseThreshold, djjw_capsense_parameters.onDebounce,
                                djjw_capsense_parameters.hysteresis, djjw_capsense_parameters.lowBslnRst,
                                WidgetTypeLUT[djjw_capsense_parameters.wdgtType]);
                            UART_UartPutString(printBuf);
                		}
                    }
                    
                    rxChar = UART_UartGetChar();
                    if('b' == rxChar)
                    {
                        
                    }
                    if('x' == rxChar)
                    {
                        
                    }
                    
                }                
                
                /* I2C - BCP interface */
                status = EZI2C_EzI2CGetActivity(); /* Get slave status to see if a Master read transaction happened. */
                if(1 == (status & EZI2C_EZI2C_STATUS_READ2))
                {
                    RegisterMap.CycleCount += 1; /* arbitrary transaction counter */
                }
                
                /* Set the device state to SENSOR_SCAN */
                currentState = SENSOR_SCAN;  
                break;  
             
            /*******************************************************************
             * Unknown power mode state. Unexpected situation.
             ******************************************************************/    
            default:
                break;
        } 
    }
}

/* [] END OF FILE */
