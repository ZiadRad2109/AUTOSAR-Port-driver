/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Ziad Hisham
 ******************************************************************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"
#include "Port_Regs.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"

/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION) || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION) || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Det.h does not match the expected version"
#endif
#endif
/*Pointer that stores port configuration*/
STATIC const Port_ConfigType *Port_ConfigTypes = NULL_PTR;

/*Port initialization status in order to use Port_Init()*/
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

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
void Port_Init(const Port_ConfigType *ConfigPtr)
{
  boolean error = FALSE;
  volatile uint32 delay = 0;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /*checking if configuration parameter is not a NULL_PTR*/
  if (NULL_PTR == ConfigPtr)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_ID, PORT_E_PARAM_CONFIG);
    error = TRUE;
  }
#endif
  else
  {

    /*
     * Set the module state to initialized and point to the PB configuration structure using a global pointer.
     * This global pointer is global to be used by other functions to read the PB configuration structures
     */
    Port_Status = PORT_INITIALIZED;
    Port_ConfigTypes = ConfigPtr;
  }

  /*looping through the 43 pins in the MC */
  for (Port_PinType PIN_COUNT = 0; PIN_COUNT < PORT_PINS_CONFIGURED; PIN_COUNT++)
  {

    volatile uint32 *Port_ptr = NULL_PTR;

    if (FALSE == error)
    {

      switch (ConfigPtr->Pin[PIN_COUNT].port_num)
      {

      case PORT_A:
        Port_ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;
        break;
      case PORT_B:
        Port_ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;
        break;
      case PORT_C:
        Port_ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;
        break;
      case PORT_D:
        Port_ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;
        break;
      case PORT_E:
        Port_ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;
        break;
      case PORT_F:
        Port_ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;
        break;
      }

      /*Activate run clock gating to corresponding ports*/
      SYSCTL_REGCGC2_REG |= (1 << ConfigPtr->Pin[PIN_COUNT].port_num);

      /*dummy variable to make sure run clock gating has taken effect*/
      delay = SYSCTL_REGCGC2_REG;

      /**************************************************************************************************************************
       *                               unlocking specific pins
       ***************************************************************************************************************************/

      if (((ConfigPtr->Pin[PIN_COUNT].port_num == PORT_D) && (ConfigPtr->Pin[PIN_COUNT].pin_num == PORT_PIN7)) || ((ConfigPtr->Pin[PIN_COUNT].port_num == PORT_F) && (ConfigPtr->Pin[PIN_COUNT].pin_num == PORT_PIN0))) /* PD7 or PF0 */
      {
        *(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                                  /* Unlock the GPIOCR register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_COMMIT_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num); /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
      }
      else if ((ConfigPtr->Pin[PIN_COUNT].port_num == PORT_C) && ((ConfigPtr->Pin[PIN_COUNT].pin_num == PORT_PIN0) || (ConfigPtr->Pin[PIN_COUNT].pin_num == PORT_PIN1) || (ConfigPtr->Pin[PIN_COUNT].pin_num == PORT_PIN2) || (ConfigPtr->Pin[PIN_COUNT].pin_num == PORT_PIN3)))
      {
        /*JTAG pins not recommended to unlock*/
        continue;
      }
      else
      {
        /*do nothing*/
      }

      /****************************************************************************************************************************
       *                               check initial mode
       *****************************************************************************************************************************/

      /***********************************************DIO******************************************************/
      if (ConfigPtr->Pin[PIN_COUNT].pin_mode == PORTPIN_MODE_DIO)
      {
        /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);

        /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);

        /* Clear the PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << ((ConfigPtr->Pin[PIN_COUNT].pin_num) * 4));

        /*set direction*/
        if (ConfigPtr->Pin[PIN_COUNT].direction == PORT_PIN_OUT) /* Pin set output*/
        {
          /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_DIR_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);

          /*check initial level if pin is set to output*/
          if (ConfigPtr->Pin[PIN_COUNT].initial_value == STD_HIGH) /*pin level set to high*/
          {

            /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_DATA_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);
          }
          else /*set to LOW*/
          {

            /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_DATA_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);
          }
        }

        else if (ConfigPtr->Pin[PIN_COUNT].direction == PORT_PIN_IN) /*pin set to input*/
        {

          /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_DIR_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);

          /*internal resistance*/
          if (ConfigPtr->Pin[PIN_COUNT].resistor == PULL_UP) /*set to pull up*/
          {

            /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_PULL_UP_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);
          }
          else if (ConfigPtr->Pin[PIN_COUNT].resistor == PULL_DOWN) /*set to pull down*/
          {

            /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_PULL_DOWN_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);
          }
          else
          {

            /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_PULL_UP_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);

            /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_PULL_DOWN_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);
          }
        }
        else
        {
          /* Do Nothing */
        }

        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);

      } /*end of DIO configuration*/

      /***********************************************ADC******************************************************/
      else if (ConfigPtr->Pin[PIN_COUNT].pin_mode == PORTPIN_MODE_ADC)
      {
        /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);

        /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);

        /* Clear the PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << ((ConfigPtr->Pin[PIN_COUNT].pin_num) * 4));

        /*Start analog clock*/
        /*SYSCTL_RCGCADC_R |= 1;*/

        /* Set the corresponding bit in the GPIOAFSEL register to enable alternate modes on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);

        /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);
      }

      /***********************************************ALTERNATIVE MODES******************************************************/
      else /*alternative mode*/
      {
        /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);

        /* Enable Alternative function for this pin by setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);

        /* Set the PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_CTL_REG_OFFSET) |= (ConfigPtr->Pin[PIN_COUNT].pin_mode & 0x0000000F << ((ConfigPtr->Pin[PIN_COUNT].pin_num) * 4));

        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), ConfigPtr->Pin[PIN_COUNT].pin_num);
      }
    }
  }
}

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
    Port_PinDirectionType Direction)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /*check driver initialization status*/
  if (Port_Status == PORT_NOT_INITIALIZED)
  {
    /*report error displaying that port is not initialized*/
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIR_ID, PORT_E_UNINIT);
  }

  else if (Pin >= PORT_PINS_CONFIGURED)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIR_ID, PORT_E_PARAM_PIN);
  }
  else if (Port_ConfigTypes->Pin[Pin].Pin_Dir_Change == STD_OFF)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIR_ID, PORT_E_DIRECTION_UNCHANGEABLE);
  }
  else
  {
    /*do nothing*/
  }

#endif

  /* point to Port address*/
  volatile uint32 *Port_ptr = NULL_PTR;

  switch (Port_ConfigTypes->Pin[Pin].port_num)
  {

  case PORT_A:
    Port_ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;
    break;
  case PORT_B:
    Port_ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;
    break;
  case PORT_C:
    Port_ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;
    break;
  case PORT_D:
    Port_ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;
    break;
  case PORT_E:
    Port_ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;
    break;
  case PORT_F:
    Port_ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;
    break;
  }
  if (Direction == PORT_PIN_IN) /*Pin configured as input*/
  {
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_DIR_REG_OFFSET), Port_ConfigTypes->Pin[Pin].pin_num);
  }
  else /*Pin configured as output*/
  {
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_DIR_REG_OFFSET), Port_ConfigTypes->Pin[Pin].pin_num);
  }
}
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
void Port_RefreshPortDirection(void)
{
  boolean error = FALSE;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  if (Port_Status == PORT_NOT_INITIALIZED)
  {
    /*report error displaying that port is not initialized*/
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_REFRESH_DIR_ID, PORT_E_UNINIT);
    error = TRUE;
  }
  else
  {
    /*do nothing*/
  }

#endif

  for (Port_PinType PIN_COUNT = 0; PIN_COUNT < PORT_PINS_CONFIGURED; PIN_COUNT++)
  {

    volatile uint32 *Port_ptr = NULL_PTR;

    if (FALSE == error)
    {

      switch (Port_ConfigTypes->Pin[PIN_COUNT].port_num)
      {

      case PORT_A:
        Port_ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;
        break;
      case PORT_B:
        Port_ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;
        break;
      case PORT_C:
        Port_ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;
        break;
      case PORT_D:
        Port_ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;
        break;
      case PORT_E:
        Port_ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;
        break;
      case PORT_F:
        Port_ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;
        break;
      }
      if ((Port_ConfigTypes->Pin[PIN_COUNT].port_num == PORT_C) && (Port_ConfigTypes->Pin[PIN_COUNT].pin_num == ((PORT_PIN0) || (PORT_PIN1) || (PORT_PIN2))))
      {
        /* avoid JTAG pins */
        continue;
      }
      /*AUTOSAR SWS requires that only the port pins that cannot be changed dynamically
      are allowed to be refreshed*/
      if (Port_ConfigTypes->Pin[PIN_COUNT].Pin_Dir_Change == STD_OFF)
      {
        if (Port_ConfigTypes->Pin[PIN_COUNT].direction == PORT_PIN_IN)
        {
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_DIR_REG_OFFSET), Port_ConfigTypes->Pin[PIN_COUNT].pin_num);
        }
        else
        {
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_DIR_REG_OFFSET), Port_ConfigTypes->Pin[PIN_COUNT].pin_num);
        }
      }
      else
      {
        /*do nothing*/
      }
    }
  }
}

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
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{

#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /*check if version info is not NULL*/
  if (NULL_PTR == versioninfo)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VER_INFO_ID, PORT_E_PARAM_POINTER);
  }
  else
#endif
  {
    /*copy vendor id*/
    versioninfo->vendorID = (uint16)PORT_VENDOR_ID;

    /*copy module id*/
    versioninfo->moduleID = (uint16)PORT_MODULE_ID;

    /*copy software version*/
    versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
    versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
    versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
  }
}
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
    Port_PinModeType Mode)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /*check driver initialization status*/
  if (Port_Status == PORT_NOT_INITIALIZED)
  {
    /*report error displaying that port is not initialized*/
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_ID, PORT_E_UNINIT);
  }

  else if (Pin >= PORT_PINS_CONFIGURED)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_ID, PORT_E_PARAM_PIN);
  }
  else if (Port_ConfigTypes->Pin[Pin].Pin_Mode_change == STD_OFF)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_ID, PORT_E_MODE_UNCHANGEABLE);
  }
  else
  {
    /*do nothing*/
  }

#endif
  /* point to Port address*/
  volatile uint32 *Port_ptr = NULL_PTR;

  switch (Port_ConfigTypes->Pin[Pin].port_num)
  {

  case PORT_A:
    Port_ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;
    break;
  case PORT_B:
    Port_ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;
    break;
  case PORT_C:
    Port_ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;
    break;
  case PORT_D:
    Port_ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;
    break;
  case PORT_E:
    Port_ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;
    break;
  case PORT_F:
    Port_ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;
    break;
  }
  switch (Mode)
  {
  case PORTPIN_MODE_DIO:
    /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_ConfigTypes->Pin[Pin].pin_num);

    /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_ALT_FUNC_REG_OFFSET), Port_ConfigTypes->Pin[Pin].pin_num);

    /* Clear the PMCx bits for this pin */
    *(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << ((Port_ConfigTypes->Pin[Pin].pin_num) * 4));
    /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_ConfigTypes->Pin[Pin].pin_num);
    break;
  case PORTPIN_MODE_ADC:
    /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_ConfigTypes->Pin[Pin].pin_num);

    /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_ALT_FUNC_REG_OFFSET), Port_ConfigTypes->Pin[Pin].pin_num);

    /* Clear the PMCx bits for this pin */
    *(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << ((Port_ConfigTypes->Pin[Pin].pin_num) * 4));

    /*Start analog clock*/
    /*SYSCTL_RCGCADC_R |= 1;*/

    /* Set the corresponding bit in the GPIOAFSEL register to enable alternate modes on this pin */
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_ALT_FUNC_REG_OFFSET), Port_ConfigTypes->Pin[Pin].pin_num);

    /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_ConfigTypes->Pin[Pin].pin_num);
    break;

  default:
    /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_ConfigTypes->Pin[Pin].pin_num);

    /* Enable Alternative function for this pin by setting the corresponding bit in GPIOAFSEL register */
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_ALT_FUNC_REG_OFFSET), Port_ConfigTypes->Pin[Pin].pin_num);

    /* Set the PMCx bits for this pin */
    *(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_CTL_REG_OFFSET) |= (Port_ConfigTypes->Pin[Pin].pin_mode & 0x0000000F << ((Port_ConfigTypes->Pin[Pin].pin_num) * 4));

    /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_ConfigTypes->Pin[Pin].pin_num);
    break;
  }
}
#endif

/************************************************************************************
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
 *              - Provide initial value for o/p pin
 *              - Setup the internal resistor for i/p pin
 ************************************************************************************/
