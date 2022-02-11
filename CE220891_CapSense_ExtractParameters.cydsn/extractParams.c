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
 * Filename:    extractParams.c
 *
 * Description: Extracts CapSense parameters from dsRAM structure. The
 *              parameters returned are intended to mimic the information
 *              available in the CapSense Tuner Widget Parameters Pane.
 *              
 * \param widgetId, sensorId, widgetParameters
 *  widgetId specifies the ID number of the widget to be processed.
 *    A function to search for widgets is included below:
 *    '--> getWidgetConfig(uint16 guessNumWidgets, uint8 *numWidgetsFound)
 * sensorId specifies the ID number of the sensor within the widget to
 *    be processed.
 * widgetParameters - pointer to variable of type widgetParameters_struct
 *    where extracted parameters will be stored.
 *
 * \return
 *  Returns the status of the widget parameter extraction:
 *  - CYRET_SUCCESS - The operation is successfully completed.
 *  - CYRET_BAD_PARAM - The input parameter is invalid.
 *
 * Returns CYRET_SUCCESS if successful, an error code if either widget ID
 *              or sensor number is out of range.
 *
 * ========================================
*/
#include <project.h>
#include "extractParams.h"

cystatus extractParams(uint32 widgetId, uint32 sensorNumber, widgetParameters_struct *widgetParameters)
{
    CapSense_FLASH_WD_STRUCT const *ptrFlashWdgt; /* pointer to widget array in Flash */
    CapSense_RAM_SNS_STRUCT *ptrSns; /* pointer variable to point to widget's sensor array in SRAM */
    CapSense_RAM_WD_BUTTON_STRUCT *ptrRamWdgt; /* pointer to widget array in SRAM */
    
    ptrFlashWdgt = &CapSense_dsFlash.wdgtArray[widgetId];
    if(widgetId < CapSense_TOTAL_CSD_WIDGETS && sensorNumber < ptrFlashWdgt->totalNumSns)
    {
        ptrRamWdgt = ptrFlashWdgt->ptr2WdgtRam; /* set pointer to requested widget in SRAM */
        ptrSns = ptrFlashWdgt->ptr2SnsRam; /* set pointer to point to first widget's sensor data in SRAM */
        ptrSns += sensorNumber; /* adjust pointer to requested sensor element */
        widgetParameters->wdgtType = ptrFlashWdgt->wdgtType;
        widgetParameters->mod_iDAC_value = ptrRamWdgt->idacMod[0]; /* modulator iDAC is associated with the entire widget */
        widgetParameters->snsClkDiv = ptrRamWdgt->snsClk;
        widgetParameters->snsClkSource = ptrRamWdgt->snsClkSource;
        widgetParameters->snsClkFreq = MODULATOR_CLK_FREQ / widgetParameters->snsClkDiv;
        widgetParameters->scanResolution = ptrRamWdgt->resolution;
        widgetParameters->iDAC_gain = CapSense_idacGainTable[ptrRamWdgt->idacGainIndex].gainValue / 1000;
        widgetParameters->comp_iDAC_value = ptrSns->idacComp[0]; /* compensation iDAC is associated with each sensor within the widget */
        widgetParameters->sensor_raw_count = ptrSns->raw[0]; /* raw values are associated with each sensor within the widget */
        widgetParameters->fingerThreshold = ptrRamWdgt->fingerTh;
        widgetParameters->noiseThreshold = ptrRamWdgt->noiseTh;
        widgetParameters->negativeNoiseThreshold = ptrRamWdgt->nNoiseTh;
        widgetParameters->onDebounce = ptrRamWdgt->onDebounce;
        widgetParameters->hysteresis = ptrRamWdgt->hysteresis;
        widgetParameters->lowBslnRst = ptrRamWdgt->lowBslnRst;
        return CYRET_SUCCESS;
    }
    else
    {
        return CYRET_BAD_PARAM;
    }
}

/*******************************************************************************
 * Function name: getModClkFreq
 *******************************************************************************
 * 
 * Description: Extracts CSD hardware clock and modulator clock divider
 *              from CapSense RAM structure. Clock frequency is then
 *              determined by multiplying by HW_CLK_MULTIPLIER (which
 *              is assumed to be 8 but can be changed in the macro
 *              defined in extractParams.h).
 *
 * \param none
 *
 * \return
 *  Returns the modulator clock frequency in kHz.
*******************************************************************************/
uint16 getModClkFreq(void)
{
    CapSense_RAM_STRUCT *ptrRamStruct; /* pointer to dsRAM structure */
    ptrRamStruct = &CapSense_dsRam;
    return ((ptrRamStruct->hwClock * HW_CLK_MULTIPLIER) / ptrRamStruct->modCsdClk);
}

/*******************************************************************************
 * Function name: getWidgetConfig
 *******************************************************************************
 * 
 * Description: Searches for widget identifiers in the CapSense 
 *              Flash structure. Widgets are identified by interrogating 
 *              the wdgtType field. Widget types are enumerated in
 *              CapSense_Structure.h. If the value returned by the
 *              ptrFlashWdgtObj pointer is less than or equal to the
 *              largest enumerated value (CapSense_WD_ENCODERDIAL_E)
 *              then the index to this pointer is assumed to be a valid
 *              widget and the search will continue.
 *
 * \param guessNumWidgets, numWidgetsFound
 *  guessNumWidgets is an input by the user to limit how deep the search
 *    for widgets will go. If the number of widgets found is equal to
 *    guessNumWidgets, a higher value should be passed and the function
 *    called again. 20 is a resonable high-end "guess".
 * numWidgetsFound - pointer to the variable where result is to be stored.
 *    The number of widgets will be either 0 (unsuccessful/no widgets
 *    found) or the number of widget identifiers successfully found (up to
 *    the value of guessNumWidgets).
 *
 * \return
 *  Returns the status of the widget search:
 *  - CYRET_SUCCESS - The operation is successfully completed.
 *  - 0xffff - aribitrary indicator that no widgets were found.
*******************************************************************************/
cystatus getWidgetConfig(uint16 guessNumWidgets, uint8 *numWidgetsFound)
{
    CapSense_FLASH_WD_STRUCT const *ptrFlashWdgtObj;
    uint8 widgetsFound = 0;
    
    /* search for widgets in dsFlash structure */
    do
    {
        ptrFlashWdgtObj = &CapSense_dsFlash.wdgtArray[widgetsFound];
        widgetsFound++;
    }while ((widgetsFound <= guessNumWidgets) && (ptrFlashWdgtObj->wdgtType <= CapSense_WD_ENCODERDIAL_E));
    
    widgetsFound -= 1; /* adjust count */
    
    if(0 != widgetsFound)
    {
        *numWidgetsFound = widgetsFound;
        return CYRET_SUCCESS;
    }
    else 
    {
        *numWidgetsFound = 0;
        return 0xffff;
    }
}
/* [] END OF FILE */
