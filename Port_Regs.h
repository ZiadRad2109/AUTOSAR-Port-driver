/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Regs.h
 *
 * Description: Pre-compilation Header file for TM4C123GH6PM Microcontroller - Port Driver Registers
 *
 * Author: Ziad Hisham
 ******************************************************************************/

#ifndef PORT_REGS_H
#define PORT_REGS_H

#include "Std_Types.h"
/****************************************************
 *                      Register Definitions
 *****************************************************/
/*GPIO ports base addresses*/
#define GPIO_PORTA_BASE_ADDRESS 0x40004000
#define GPIO_PORTB_BASE_ADDRESS 0x40005000
#define GPIO_PORTC_BASE_ADDRESS 0x40006000
#define GPIO_PORTD_BASE_ADDRESS 0x40007000
#define GPIO_PORTE_BASE_ADDRESS 0x40024000
#define GPIO_PORTF_BASE_ADDRESS 0x40025000

/*address offsets*/
#define PORT_DATA_REG_OFFSET 0x3FC
#define PORT_DIR_REG_OFFSET 0x400
#define PORT_ALT_FUNC_REG_OFFSET 0x420
#define PORT_PULL_UP_REG_OFFSET 0x510
#define PORT_PULL_DOWN_REG_OFFSET 0x514
#define PORT_DIGITAL_ENABLE_REG_OFFSET 0x51C
#define PORT_LOCK_REG_OFFSET 0x520
#define PORT_COMMIT_REG_OFFSET 0x524
#define PORT_ANALOG_MODE_SEL_REG_OFFSET 0x528
#define PORT_CTL_REG_OFFSET 0x52C

#define GPIO_PORTA_DATA_REG (*((volatile uint32 *)0x400043FC))
#define GPIO_PORTB_DATA_REG (*((volatile uint32 *)0x400053FC))
#define GPIO_PORTC_DATA_REG (*((volatile uint32 *)0x400063FC))
#define GPIO_PORTD_DATA_REG (*((volatile uint32 *)0x400073FC))
#define GPIO_PORTE_DATA_REG (*((volatile uint32 *)0x400243FC))
#define GPIO_PORTF_DATA_REG (*((volatile uint32 *)0x400253FC))

/*run clock gating register*/
#define SYSCTL_REGCGC2_REG (*((volatile uint32 *)0x400FE108))

#endif /* PORT_REGS_H */
