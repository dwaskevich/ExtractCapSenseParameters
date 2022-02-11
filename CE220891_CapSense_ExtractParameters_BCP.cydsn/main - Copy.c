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
* Update -  5-Oct-2021 (David Waskevich) --> Modified firmware to report via BCP instead of UART.
*                                            Also removed source files (added function to main.c).
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
#include <cytypes.h>
#include "project.h"

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

#define MODULATOR_CLK_FREQ      (CYDEV_BCLK__HFCLK__KHZ / CapSense_CSD_SCANSPEED_DIVIDER)
#define HW_CLK_MULTIPLIER       (8u)

/*****************************************************************************
* Global variables
*****************************************************************************/
CapSense_FLASH_WD_STRUCT const *ptrFlashWdgt; /* pointer to widget array in Flash */
uint8 loopCount;

/* register map that will be exposed to I2C host at secondary address (for Bridge Control Panel) */
struct RegisterMap {
    uint16  totalNumSensors[CapSense_TOTAL_CSD_SENSORS];
    uint16  snsClkSource[CapSense_TOTAL_CSD_SENSORS];
    uint16  snsClkDiv[CapSense_TOTAL_CSD_SENSORS];
    uint16  snsClkFreq[CapSense_TOTAL_CSD_SENSORS];
    uint16  scanResolution[CapSense_TOTAL_CSD_SENSORS];
    uint16  mod_iDAC_value[CapSense_TOTAL_CSD_SENSORS];
    uint16  iDAC_gain[CapSense_TOTAL_CSD_SENSORS];
    uint16  fingerThreshold[CapSense_TOTAL_CSD_SENSORS];
    uint16  noiseThreshold[CapSense_TOTAL_CSD_SENSORS];
    uint16  negativeNoiseThreshold[CapSense_TOTAL_CSD_SENSORS];
    uint16  lowBslnRst[CapSense_TOTAL_CSD_SENSORS];
    uint16  onDebounce[CapSense_TOTAL_CSD_SENSORS];
    uint16  sensorNum[CapSense_TOTAL_CSD_SENSORS];
    uint16  comp_iDAC_value[CapSense_TOTAL_CSD_SENSORS];
    uint16  sensor_raw_count[CapSense_TOTAL_CSD_SENSORS];
    uint16  hysteresis[CapSense_TOTAL_CSD_SENSORS];
    CapSense_WD_TYPE_ENUM wdgtType[CapSense_TOTAL_CSD_SENSORS];
    uint16  CycleCount; /* holds aribitrary I2C Master-read cycle count */
} RegisterMap;

/*****************************************************************************
* Function prototypes
*****************************************************************************/
void extractParams(uint32 widgetId, uint32 sensorElement);

int main()
{	        
    uint32 status; /* EZI2C function call status return */
    
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
   
    EZI2C_Start(); /* Start EZI2C Component */
    /*
    * Set up communication and initialize data buffer to CapSense data structure
    * to use Tuner application
    */
    EZI2C_EzI2CSetBuffer1(sizeof(CapSense_dsRam), sizeof(CapSense_dsRam), (uint8 *)&CapSense_dsRam);
    
    /* Set up communication data buffer to be exposed to I2C master at secondary slave address (BCP). */
    EZI2C_EzI2CSetBuffer2(sizeof(RegisterMap), sizeof(RegisterMap), (uint8 *)&RegisterMap);
    
    CapSense_Start(); /* Initialize Component */
    CapSense_ScanAllWidgets(); /* Scan all widgets */

    for(;;)
    {        
        /* Do this only when a scan is done */
        if(CapSense_NOT_BUSY == CapSense_IsBusy())
        {
            CapSense_ProcessAllWidgets(); /* Process all widgets */
            CapSense_RunTuner(); /* To sync with Tuner application */
            if (CapSense_IsAnyWidgetActive()) /* Scan result verification */
            {
                /* add custom tasks to execute when touch detected */
                LED1_Write(LED_ON);
            }
            else LED1_Write(LED_OFF);
            
            /* calculate delta sensor Cp's from scan parameters */
            loopCount = 0;            	
            for(uint8 i = 0; i < CapSense_TOTAL_WIDGETS; i++)
        	{
        		ptrFlashWdgt = &CapSense_dsFlash.wdgtArray[i];
        		for(uint8 j = 0; j < ptrFlashWdgt->totalNumSns; j++) /* process all sensors associated with current widget */
        		{
                    extractParams(i, j);
        			loopCount++; /* increment index to next location in RegisterMap */
        		}
            }
            
            /* I2C - BCP interface */
            status = EZI2C_EzI2CGetActivity(); /* Get slave status to see if a Master read transaction happened. */
            if(1 == (status & EZI2C_EZI2C_STATUS_READ2))
            {
                RegisterMap.CycleCount += 1; /* arbitrary transaction counter */
            }
            
            CapSense_ScanAllWidgets(); /* Start next scan */
        }

    }
    
}

/*******************************************************************************
* Function Name: extractParams
********************************************************************************
*
* Description:  Extracts CapSense parameters from dsRAM structure. The
*               parameters returned are intended to mimic the information
*               available in the CapSense Tuner Widget Parameters Pane.
*
* Parameters:   widgetId, sensorId
*  widgetId specifies the ID number of the widget to be processed.
*  sensorId specifies the ID number of the sensor within the widget to
*   be processed.
*
* Return:
*  None
*
*******************************************************************************/

void extractParams(uint32 widgetId, uint32 sensorNumber)
{
    CapSense_RAM_SNS_STRUCT *ptrSns; /* pointer variable to point to widget's sensor array in SRAM */
    CapSense_RAM_WD_BUTTON_STRUCT *ptrRamWdgt; /* pointer to widget array in SRAM */
    
    ptrFlashWdgt = &CapSense_dsFlash.wdgtArray[widgetId];
    if(widgetId < CapSense_TOTAL_CSD_WIDGETS && sensorNumber < ptrFlashWdgt->totalNumSns)
    {
        ptrRamWdgt = ptrFlashWdgt->ptr2WdgtRam; /* set pointer to requested widget in SRAM */
        ptrSns = ptrFlashWdgt->ptr2SnsRam; /* set pointer to point to first widget's sensor data in SRAM */
        ptrSns += sensorNumber; /* adjust pointer to requested sensor element */
        RegisterMap.wdgtType[loopCount] = ptrFlashWdgt->wdgtType;
        RegisterMap.mod_iDAC_value[loopCount] = ptrRamWdgt->idacMod[0]; /* modulator iDAC is associated with the entire widget */
        RegisterMap.snsClkDiv[loopCount] = ptrRamWdgt->snsClk;
        RegisterMap.snsClkSource[loopCount] = ptrRamWdgt->snsClkSource;
        RegisterMap.snsClkFreq[loopCount] = MODULATOR_CLK_FREQ / ptrRamWdgt->snsClk;
        RegisterMap.scanResolution[loopCount] = ptrRamWdgt->resolution;
        RegisterMap.iDAC_gain[loopCount] = CapSense_idacGainTable[ptrRamWdgt->idacGainIndex].gainValue / 1000;
        RegisterMap.comp_iDAC_value[loopCount] = ptrSns->idacComp[0]; /* compensation iDAC is associated with each sensor within the widget */
        RegisterMap.sensor_raw_count[loopCount] = ptrSns->raw[0]; /* raw values are associated with each sensor within the widget */
        RegisterMap.fingerThreshold[loopCount] = ptrRamWdgt->fingerTh;
        RegisterMap.noiseThreshold[loopCount] = ptrRamWdgt->noiseTh;
        RegisterMap.negativeNoiseThreshold[loopCount] = ptrRamWdgt->nNoiseTh;
        RegisterMap.onDebounce[loopCount] = ptrRamWdgt->onDebounce;
        RegisterMap.hysteresis[loopCount] = ptrRamWdgt->hysteresis;
        RegisterMap.lowBslnRst[loopCount] = ptrRamWdgt->lowBslnRst;
    }
}

/* [] END OF FILE */
