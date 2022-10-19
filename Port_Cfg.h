/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_cfg.h
 *
 * Description: Pre-compile configuration file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Ziad Hisham
 ******************************************************************************/
#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
Module Version 1.0.0
*/
#define PORT_CFG_SW_MAJOR_VERSION (1U)
#define PORT_CFG_SW_MINOR_VERSION (0U)
#define PORT_CFG_SW_PATCH_VERSION (0U)

/*
AUTOSAR Version 4.0.3
*/
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION (3U)

/*number of configurable pins on/off*/
#define PORT_PINS_CONFIGURED (43U)

/*Pin direction changeability on/off*/
#define PORT_SET_PIN_DIR_API (STD_ON)

/*get module version on/off*/
#define PORT_VERSION_INFO_API (STD_ON)

/*pin mode changeability on/off*/
#define PORT_SET_PIN_MODE_API (STD_ON)

/*development error detect on/off*/
#define PORT_DEV_ERROR_DETECT (STD_ON)

/*MCU PORTS*/
#define PORT_A (0U)
#define PORT_B (1U)
#define PORT_C (2U)
#define PORT_D (3U)
#define PORT_E (4U)
#define PORT_F (5U)

/*MCU PINS*/
#define PORT_PIN0 (0U)
#define PORT_PIN1 (1U)
#define PORT_PIN2 (2U)
#define PORT_PIN3 (3U)
#define PORT_PIN4 (4U)
#define PORT_PIN5 (5U)
#define PORT_PIN6 (6U)
#define PORT_PIN7 (7U)

#endif /*PORT_CFG_H*/