 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Peter Nabil
 ******************************************************************************/

#ifndef PORT_CFG_H_
#define PORT_CFG_H_

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION             (1U)
#define PORT_CFG_SW_MINOR_VERSION             (0U)
#define PORT_CFG_SW_PATCH_VERSION             (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                 (STD_ON)

/* Pre-compile option for setting the pin direction */
#define PORT_SET_PIN_DIRECTION_API            (STD_ON)

/* Pre-compile option for setting the pin mode */
#define PORT_SET_PIN_MODE_API                 (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                 (STD_OFF)

/* Number of the configured Port Channels */
#define PORT_CONFIGURED_CHANNLES              (39U)

/* Ports indices */
#define PORTA_ID      (uint8)0
#define PORTB_ID      (uint8)1
#define PORTC_ID      (uint8)2
#define PORTD_ID      (uint8)3
#define PORTE_ID      (uint8)4
#define PORTF_ID      (uint8)5

/* Pins indices */
#define PIN0_ID    (uint8)0
#define PIN1_ID    (uint8)1
#define PIN2_ID    (uint8)2
#define PIN3_ID    (uint8)3
#define PIN4_ID    (uint8)4
#define PIN5_ID    (uint8)5
#define PIN6_ID    (uint8)6
#define PIN7_ID    (uint8)7

/* Pin modes */
#define PORT_MODE    (Port_PinModeType)0
#define ANALOG_MODE  (Port_PinModeType)0
#define MODE1        (Port_PinModeType)1
#define MODE2        (Port_PinModeType)2
#define MODE3        (Port_PinModeType)3
#define MODE4        (Port_PinModeType)4
#define MODE5        (Port_PinModeType)5
#define MODE6        (Port_PinModeType)6
#define MODE7        (Port_PinModeType)7
#define MODE8        (Port_PinModeType)8
#define MODE9        (Port_PinModeType)9
#define MODE10       (Port_PinModeType)10
#define MODE11       (Port_PinModeType)11
#define MODE12       (Port_PinModeType)12
#define MODE13       (Port_PinModeType)13
#define MODE14       (Port_PinModeType)14
#define MODE15       (Port_PinModeType)15

#endif /* PORT_CFG_H_ */
