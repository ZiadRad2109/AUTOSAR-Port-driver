/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Ziad Hisham
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

#include "Common_Macros.h"

/*Standard AUTOSAR types*/
#include "Std_Types.h"

/*Port module pre-compilation header file*/
#include "Port_Cfg.h"

/*Company Id in AUTOSAR*/
#define PORT_VENDOR_ID (1000U)

/*Port module Id*/
#define PORT_MODULE_ID (124U)

/*Port Instance Id*/
#define PORT_INSTANCE_ID (0U)

/*
Module Version 1.0.0
*/
#define PORT_SW_MAJOR_VERSION (1U)
#define PORT_SW_MINOR_VERSION (0U)
#define PORT_SW_PATCH_VERSION (0U)

/*
AUTOSAR Version 4.0.3
*/
#define PORT_AR_RELEASE_MAJOR_VERSION (4U)
#define PORT_AR_RELEASE_MINOR_VERSION (0U)
#define PORT_AR_RELEASE_PATCH_VERSION (3U)

/*
 Port Status
 */
#define PORT_INITIALIZED (1U)
#define PORT_NOT_INITIALIZED (0U)

/* AUTOSAR versions checking if Port modules and Standard types align */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION) || (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION) || (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Std_Types.h does not match the expected version"
#endif

/* AUTOSAR versions checking if Port.h and Port_cfg.h align */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION) || (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION) || (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software versions checking if Port.h and Port_cfg.h align */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION) || (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION) || (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
#error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/

/* port initialization */
#define PORT_INIT_ID (uint8)0x00

/* Port direction set */
#define PORT_SET_PIN_DIR_ID (uint8)0x01

/* Port direction refresh */
#define PORT_REFRESH_DIR_ID (uint8)0x02

/* get port version info */
#define PORT_GET_VER_INFO_ID (uint8)0x03

/* Port set pin mode */
#define PORT_SET_PIN_MODE_ID (uint8)0x04

/******************************************************************************
 *                      DET Error Codes                                 *
 ******************************************************************************/

/* DET code to report invalid port pin identification */
#define PORT_E_PARAM_PIN (uint8)0x0A

/* DET code to report Port pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE (uint8)0x0B

/* DET code to report Port_init service called with wrong parameter */
#define PORT_E_PARAM_CONFIG (uint8)0x0C

/* DET code to report Port_SetPinMode service called when mode is invalid/unchangeable */
#define PORT_E_PARAM_INVALID_MODE (uint8)0x0D
#define PORT_E_MODE_UNCHANGEABLE (uint8)0x0E

/*
 * API service used without module initialization is reported using following
 * error code (Not exist in AUTOSAR 4.0.3 DIO SWS Document.
 */
#define PORT_E_UNINIT (uint8)0x0F

/*
 * The API service shall return immediately without any further action,
 * beside reporting this development error, service called with NULL PTR
 */
#define PORT_E_PARAM_POINTER (uint8)0x10

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/*Description: Enum for possible pin directions */
typedef enum
{
  PORT_PIN_IN,
  PORT_PIN_OUT
} Port_PinDirectionType;

/* Description: Enum to hold PIN direction */
typedef enum
{
  INPUT,
  OUTPUT
} Port_PinDirection;

/*Description: names for each of 43 pins (uint8) */
typedef uint8 Port_PinType;

/*Description: Pin mode selection */
typedef uint8 Port_PinModeType;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
  OFF,
  PULL_UP,
  PULL_DOWN
} Port_InternalResistor;



/*Description: Port pin default or alternative modes(14 modes)*/
typedef enum
{
  PORTPIN_MODE_DIO,
  PORTPIN_MODE_ADC,
  PORTPIN_MODE_ALT_1,
  PORTPIN_MODE_ALT_2,
  PORTPIN_MODE_ALT_3,
  PORTPIN_MODE_ALT_4,
  PORTPIN_MODE_ALT_5,
  PORTPIN_MODE_ALT_6,
  PORTPIN_MODE_ALT_7,
  PORTPIN_MODE_ALT_8,
  PORTPIN_MODE_ALT_9,
  PORTPIN_MODE_ALT_10,
  PORTPIN_MODE_ALT_11,
  PORTPIN_MODE_ALT_12,
  PORTPIN_MODE_ALT_13,
  PORTPIN_MODE_ALT_14
} Port_PinMode;

/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *      3. the direction of pin --> INPUT or OUTPUT
 *      4. the internal resistor --> Disable, Pull up or Pull down
 *      5. initial value (STD_HIGH,STD_LOW)
 *      6. Pin mode (DIO or alternatives)
 *      7. direction changeability (STD_ON,STD_OFF)
 *      8. Pin mode changeability (STD_ON,STD_OFF)
 */
typedef struct
{
  uint8 port_num;                 /*Port A,B,C,D,E,F*/
  uint8 pin_num;                  /*Port pin number PF0,PF1,...etc*/
  Port_PinDirectionType direction;    /*PORT_PIN_IN,PORT_PIN_OUT*/
  Port_InternalResistor resistor; /*OFF,PULL_UP,PULL_DOWN*/
  uint8 initial_value;            /*STD_HIGH,STD_LOW*/
  Port_PinMode pin_mode;          /*PORTPIN_MODE_DIO,PORTPIN_MODE_ALT_1,...etc*/
  uint8 Pin_Dir_Change;           /*STD_ON,STD_OFF*/
  uint8 Pin_Mode_change;          /*STD_ON,STD_OFF*/
} Port_ConfigPin;

/*Description: Configuration array for number of pins*/
typedef struct
{
  Port_ConfigPin Pin[PORT_PINS_CONFIGURED];
} Port_ConfigType;

/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *      3. the direction of pin --> INPUT or OUTPUT
 *      4. the internal resistor --> Disable, Pull up or Pull down
 */
/***********************remove********************/
typedef struct
{
  uint8 port_num;
  uint8 pin_num;
  Port_PinDirection direction;
  Port_InternalResistor resistor;
  uint8 initial_value;
} Port_ConfigType_NAR;

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/************************************************************************************
 * Service Name: Port_Init               ServiceID: 0x00
 * Sync/Async: Synchronous
 * Reentrancy: non-reentrant
 * Parameters (in): ConfigPtr - Pointer to post-build configuration data
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Port driver initialization
 ************************************************************************************/
void Port_Init(const Port_ConfigType *ConfigPtr);

/************************************************************************************
 * Service Name: Port_SetPinDirection    ServiceID: 0x01
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): Pin (Port Pin Identification), Direction (Port Pin Direction)
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Sets the Port Pin direction
 ************************************************************************************/
#if (PORT_SET_PIN_DIR_API == STD_ON)

void Port_SetPinDirection(
    Port_PinType Pin,
    Port_PinDirectionType Direction);
#endif

/************************************************************************************
 * Service Name: Port_RefreshPortDirection               ServiceID: 0x02
 * Sync/Async: Synchronous
 * Reentrancy: non-reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Refreshes Port direction
 ************************************************************************************/
void Port_RefreshPortDirection(void);

/************************************************************************************
 * Service Name: Port_GetVersionInfo    ServiceID: 0x03
 * Sync/Async: Synchronous
 * Reentrancy: non-reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): VersionInfo
 * Return value: None
 * Description: Gets module version information
 ************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo);
#endif

/************************************************************************************
 * Service Name: Port_SetPinMode               ServiceID: 0x04
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): Pin (Port Pin Identification), Mode (Pin mode to be set)
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Sets the pin mode
 ************************************************************************************/
#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode(
    Port_PinType Pin,
    Port_PinModeType Mode);
#endif

/***************************************remove*********************************************
 * Service Name: Port_SetupGpioPin
 * Sync/Async: Synchronous
 * Reentrancy: reentrant
 * Parameters (in): ConfigPtr - Pointer to post-build configuration data
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to Setup the pin configuration:
 *              - Setup the pin as Digital GPIO pin
 *              - Setup the direction of the GPIO pin
 *              - Setup the internal resistor for i/p pin
 ************************************************************************************/
/*void Port_SetupGpioPin(const Port_ConfigPin *ConfigPtr);*/

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by Port module */
extern const Port_ConfigType Port_ConfigPin_ex;

#endif /* PORT_H */
