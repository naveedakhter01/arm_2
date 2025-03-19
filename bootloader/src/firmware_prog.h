/**************************************************************************
* File Name          : firmware_prog.h
* Author             : DOC Electronics Development Team
* Version            : V1.0.0
* Date               : 17/08/2016
* Description        : functions for programming arm and dsp
***************************************************************************/

#ifndef FIRMWARE_PROG_H
#define FIRMWARE_PROG_H


#include "stm32f10x.h"



#define ARM_FIRMWARE        ((uint32_t)0x00001)  
#define DSP_FIRMWARE        ((uint32_t)0x00002)  
#define BOOTLOADER_FIRMWARE ((uint32_t)0x00004)  
#define SETTINGS_MASK            ((uint32_t)0x00008)  


#define ROM_BASE              ((uint32_t)0x08000000)
#define PROGRAM_START_OFFSET  ((uint32_t)0x00010000)
#define PROGRAM_START_ADDR (ROM_BASE + PROGRAM_START_OFFSET)
#define EndAddr    ((uint32_t)0x0807EFFF)


#define ARM_MAIN_FIRMWARE_FILENAME "arm.boot"  // arm main program only
#define DSP_FIRMWARE_FILENAME "dsp.boot"   // dsp firmware
#define ARM_BOOTLOADER_FIRMWARE_FILENAME "bootloader.boot"  // arm bootloader only
#define ARM_BOTH_FIRMWARE_FILENAME "arm_bootloader.boot"  // both the main program and bootloader
#define SETTINGS_FILENAME "settings.ini"  // file to modify settings



bool FlashARM(void);
bool FlashDSP(void);
uint32_t CheckForFirmware(void);




#endif


/*************************************EOF**************************************/