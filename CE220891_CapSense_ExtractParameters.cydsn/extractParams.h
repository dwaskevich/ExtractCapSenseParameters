/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * Date:        2-Oct-2021
 * Author:      David Waskevich
 * Filename:    extractParams.h
 *
 * ========================================
*/
#include <cytypes.h>
#include "project.h"

#define MODULATOR_CLK_FREQ      (CYDEV_BCLK__HFCLK__KHZ / CapSense_CSD_SCANSPEED_DIVIDER)
#define HW_CLK_MULTIPLIER       (8u)

typedef struct
{
    /**
     *  Specifies one of the following widget types: 
     *  WD_BUTTON_E, WD_LINEAR_SLIDER_E, WD_RADIAL_SLIDER_E, 
     *  WD_MATRIX_BUTTON_E, WD_TOUCHPAD_E, WD_PROXIMITY_E
     */
    CapSense_WD_TYPE_ENUM wdgtType;
    
    uint16  totalNumSensors;
    uint8   snsClkSource;
    uint8   snsClkDiv;
    uint16  snsClkFreq;
    uint16  scanResolution;
    uint8   mod_iDAC_value;
    uint16  iDAC_gain;
    uint16  fingerThreshold;
    uint8   noiseThreshold;
    uint8   negativeNoiseThreshold;
    uint8   lowBslnRst;
    uint8   onDebounce;
    uint8   sensorNum;
    uint8   comp_iDAC_value;
    uint16  sensor_raw_count;
    uint8   hysteresis;
    
} widgetParameters_struct;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
cystatus extractParams(uint32 widgetId, uint32 sensorNumber, widgetParameters_struct *widgetParameters);
uint16 getModClkFreq(void);
cystatus getWidgetConfig(uint16 guessNumWidgets, uint8 *numWidgetsFound);

/* [] END OF FILE */
