 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_PBcfg.c
 *
 * Description: Post Build Configuration Source file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Peter Nabil
 ******************************************************************************/

#include "Port.h"

/*
 * Module Version 1.0.0
 */
#define PORT_PBCFG_SW_MAJOR_VERSION              (1U)
#define PORT_PBCFG_SW_MINOR_VERSION              (0U)
#define PORT_PBCFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_PBCFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_PBCFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_PBCFG_AR_RELEASE_PATCH_VERSION     (3U)

/* AUTOSAR Version checking between PORT_PBcfg.c and PORT.h files */
#if ((PORT_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of PBcfg.c does not match the expected version"
#endif

/* Software Version checking between PORT_PBcfg.c and PORT.h files */
#if ((PORT_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_PBCFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of PBcfg.c does not match the expected version"
#endif

/* PB structure used with Port_Init API */
const Port_ConfigType Port_Configuration = {
                                                /******************************PORTA_ID*******************************/
                                                PORTA_ID, PIN0_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTA_ID, PIN1_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTA_ID, PIN2_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTA_ID, PIN3_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTA_ID, PIN4_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTA_ID, PIN5_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTA_ID, PIN6_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTA_ID, PIN7_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,

                                                /******************************PORTB_ID*******************************/
                                                PORTB_ID, PIN0_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTB_ID, PIN1_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTB_ID, PIN2_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTB_ID, PIN3_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTB_ID, PIN4_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTB_ID, PIN5_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTB_ID, PIN6_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTB_ID, PIN7_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,

                                                /******************************PORTC_ID*******************************/
                                                PORTC_ID, PIN4_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTC_ID, PIN5_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTC_ID, PIN6_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTC_ID, PIN7_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,

                                                /******************************PORTD_ID*******************************/
                                                PORTD_ID, PIN0_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTD_ID, PIN1_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTD_ID, PIN2_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTD_ID, PIN3_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTD_ID, PIN4_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTD_ID, PIN5_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTD_ID, PIN6_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTD_ID, PIN7_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,

                                                /******************************PORTE_ID*******************************/
                                                PORTE_ID, PIN0_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTE_ID, PIN1_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTE_ID, PIN2_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTE_ID, PIN3_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTE_ID, PIN4_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTE_ID, PIN5_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,

                                                /******************************PORTF_ID*******************************/
                                                PORTF_ID, PIN0_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTF_ID, PIN1_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTF_ID, PIN2_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTF_ID, PIN3_ID, PORT_PIN_OUT, OFF, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                                PORTF_ID, PIN4_ID, PORT_PIN_IN, PULL_UP, STD_LOW, PORT_MODE, STD_OFF, STD_OFF,
                                           };
