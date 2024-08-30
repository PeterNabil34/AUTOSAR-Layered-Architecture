 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Peter Nabil
 ******************************************************************************/

#include "Port.h"
#include "Port_Regs.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif

/***********************************************************************************
 *                                Definitions                                      *
 ***********************************************************************************/
#define MODE_MASK        (0x0000000F)
#define NO_OF_MODE_BITS  (4U)

/***********************************************************************************
 *                             Global Variables                                    *
 ***********************************************************************************/
STATIC const Port_ConfigChannel * Port_channelsPtr = NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

/************************************************************************************
 *                          Functions Definitions                                   *
 ***********************************************************************************/

/************************************************************************************
* Service Name: Port_Init
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Initialize the Port Driver module.
************************************************************************************/
void Port_Init(const Port_ConfigType* ConfigPtr)
{
    uint8 index = 0;
    volatile uint32 * Port_registerBaseAddress = NULL_PTR; /* point to the required Port Registers base address */

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the input configuration pointer is not a NULL_PTR */
    if(NULL_PTR == ConfigPtr)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID, PORT_E_PARAM_CONFIG);
    }
    else
#endif
    {
        /*
         * Set the module state to initialized and point to the PB configuration structure using a global pointer.
         * This global pointer is global to be used by other functions to read the PB configuration structures
         */
        Port_Status = PORT_INITIALIZED;
        Port_channelsPtr = &(ConfigPtr->Channels[0]); /* address of the first Channels structure --> Channels[0] */
    }

    while(index<PORT_CONFIGURED_CHANNLES)
    {
        switch(Port_channelsPtr[index].port_num)
        {
            case  PORTA_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                     break;
            case  PORTB_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                     break;
            case  PORTC_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                     break;
            case  PORTD_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                     break;
            case  PORTE_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                     break;
            case  PORTF_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                     break;
        }
        if( ((PORTD_ID == Port_channelsPtr[index].port_num) && (PIN7_ID == Port_channelsPtr[index].pin_num)) || ((PORTF_ID == Port_channelsPtr[index].port_num) && (PIN0_ID == Port_channelsPtr[index].pin_num)) ) /* PD7 or PF0 */
        {
            *(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                                   /* Unlock the GPIOCR register */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_COMMIT_REG_OFFSET) , Port_channelsPtr[index].port_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
        }
        else if( (PORTC_ID == Port_channelsPtr[index].port_num) && (Port_channelsPtr[index].pin_num <= PIN3_ID) ) /* PC0 to PC3 */
        {
            /* Do Nothing ...  this is the JTAG pins */
        }
        else
        {
            /* Do Nothing ... No need to unlock the commit register for this pin */
        }

        if(PORT_PIN_OUT == Port_channelsPtr[index].direction)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DIR_REG_OFFSET) , Port_channelsPtr[index].pin_num);            /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

            if(STD_HIGH == Port_channelsPtr[index].initial_value)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DATA_REG_OFFSET) , Port_channelsPtr[index].pin_num);       /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
            }
            else
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DATA_REG_OFFSET) , Port_channelsPtr[index].pin_num);     /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
            }
        }
        else if(PORT_PIN_IN == Port_channelsPtr[index].direction)
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DIR_REG_OFFSET) , Port_channelsPtr[index].pin_num);          /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

            if(PULL_UP == Port_channelsPtr[index].resistor)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_PULL_UP_REG_OFFSET) , Port_channelsPtr[index].pin_num);    /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
            }
            else if(PULL_DOWN == Port_channelsPtr[index].resistor)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_PULL_DOWN_REG_OFFSET) , Port_channelsPtr[index].pin_num);  /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
            }
            else
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_PULL_UP_REG_OFFSET) , Port_channelsPtr[index].pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_PULL_DOWN_REG_OFFSET) , Port_channelsPtr[index].pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
            }
        }
        else
        {
            /* Do Nothing */
        }

        if(PORT_MODE == Port_channelsPtr[index].mode)
        {
            /* Setup the pin mode as GPIO */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_channelsPtr[index].pin_num);                 /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_ALT_FUNC_REG_OFFSET) , Port_channelsPtr[index].pin_num);                        /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
            *(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_CTL_REG_OFFSET) &= ~(MODE_MASK << (Port_channelsPtr[index].pin_num * NO_OF_MODE_BITS));   /* Clear the PMCx bits for this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_channelsPtr[index].pin_num);                    /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        }
        else if(ANALOG_MODE == Port_channelsPtr[index].mode)
        {
            /* Setup the pin as Analog pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_channelsPtr[index].pin_num);                   /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_ALT_FUNC_REG_OFFSET) , Port_channelsPtr[index].pin_num);                        /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
            *(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_CTL_REG_OFFSET) &= ~(MODE_MASK << (Port_channelsPtr[index].pin_num * NO_OF_MODE_BITS));   /* Clear the PMCx bits for this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_channelsPtr[index].pin_num);                  /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
        }
        else
        {
            /* Setup the pin mode for a special purpose */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_channelsPtr[index].pin_num);                                  /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_ALT_FUNC_REG_OFFSET) , Port_channelsPtr[index].pin_num);                                           /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
            *(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_CTL_REG_OFFSET) &= ~(MODE_MASK << (Port_channelsPtr[index].pin_num * NO_OF_MODE_BITS));                    /* Clear the PMCx bits for this pin */
            *(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_CTL_REG_OFFSET) |= (Port_channelsPtr[index].mode << (Port_channelsPtr[index].pin_num * NO_OF_MODE_BITS));  /* Assign the mode value to the corresponding PMCx bits for this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_channelsPtr[index].pin_num);                                     /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        }

        index++;
    }
}

#if (STD_ON == PORT_SET_PIN_DIRECTION_API)
/************************************************************************************
* Service Name: Port_SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin       - Port Pin ID number.
*                  Direction - Port Pin ID direction.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to set the port pin direction.
************************************************************************************/
void Port_SetPinDirection(Port_PinType Pin, Port_PinDirectionType Direction)
{
    volatile uint32 * Port_registerBaseAddress = NULL_PTR; /* point to the required Port Registers base address */
    boolean error = FALSE;

#if (STD_ON == PORT_DEV_ERROR_DETECT)
    /* Check if the Driver is initialized before using this function */
    if(PORT_NOT_INITIALIZED == Port_Status)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the used pin is within the valid range */
    if(PORT_CONFIGURED_CHANNLES<=Pin)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the used pin has the Direction Changeable option on */
    if(STD_OFF == Port_channelsPtr[Pin].directionChangeable)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
#endif

    /* In-case there are no errors */
    if(FALSE == error)
    {
        switch(Port_channelsPtr[Pin].port_num)
        {
            case  PORTA_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                     break;
            case  PORTB_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                     break;
            case  PORTC_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                     break;
            case  PORTD_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                     break;
            case  PORTE_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                     break;
            case  PORTF_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                     break;
        }
        if( ((PORTD_ID == Port_channelsPtr[Pin].port_num) && (PIN7_ID == Port_channelsPtr[Pin].pin_num)) || ((PORTF_ID == Port_channelsPtr[Pin].port_num) && (PIN0_ID == Port_channelsPtr[Pin].pin_num)) ) /* PD7 or PF0 */
        {
            *(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                                   /* Unlock the GPIOCR register */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_COMMIT_REG_OFFSET) , Port_channelsPtr[Pin].port_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
        }
        else if( (PORTC_ID == Port_channelsPtr[Pin].port_num) && (Port_channelsPtr[Pin].pin_num <= PIN3_ID) ) /* PC0 to PC3 */
        {
            /* Do Nothing ...  this is the JTAG pins */
        }
        else
        {
            /* Do Nothing ... No need to unlock the commit register for this pin */
        }

        if(PORT_PIN_OUT == Direction)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DIR_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);            /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

            if(STD_HIGH == Port_channelsPtr[Pin].initial_value)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DATA_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);       /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
            }
            else
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DATA_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);     /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
            }
        }
        else if(PORT_PIN_IN == Direction)
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DIR_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);          /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

            if(PULL_UP == Port_channelsPtr[Pin].resistor)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_PULL_UP_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);    /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
            }
            else if(PULL_DOWN == Port_channelsPtr[Pin].resistor)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_PULL_DOWN_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);  /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
            }
            else
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_PULL_UP_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_PULL_DOWN_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
            }
        }
        else
        {
            /* Do Nothing */
        }
    }
    else
    {
        /* No Action Required */
    }
}
#endif

#if (STD_ON == PORT_SET_PIN_MODE_API)
/************************************************************************************
* Service Name: Port_SetPinMode
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin  - Port Pin ID number.
*                  Mode - New Port Pin mode to be set on port pin.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to set the port pin mode.
************************************************************************************/
void Port_SetPinMode(Port_PinType Pin, Port_PinModeType Mode)
{
    volatile uint32 * Port_registerBaseAddress = NULL_PTR; /* point to the required Port Registers base address */
    boolean error = FALSE;

#if (STD_ON == PORT_DEV_ERROR_DETECT)
    /* Check if the Driver is initialized before using this function */
    if(PORT_NOT_INITIALIZED == Port_Status)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_UNINIT);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the used pin is within the valid range */
    if(PORT_CONFIGURED_CHANNLES<=Pin)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the used pin has the Direction Changeable option on */
    if(STD_OFF == Port_channelsPtr[Pin].modeChangeable)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }

    if(MODE15<Mode)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
#endif

    /* In-case there are no errors */
    if(FALSE == error)
    {
        switch(Port_channelsPtr[Pin].port_num)
        {
            case  PORTA_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                     break;
            case  PORTB_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                     break;
            case  PORTC_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                     break;
            case  PORTD_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                     break;
            case  PORTE_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                     break;
            case  PORTF_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                     break;
        }
        if( ((PORTD_ID == Port_channelsPtr[Pin].port_num) && (PIN7_ID == Port_channelsPtr[Pin].pin_num)) || ((PORTF_ID == Port_channelsPtr[Pin].port_num) && (PIN0_ID == Port_channelsPtr[Pin].pin_num)) ) /* PD7 or PF0 */
        {
            *(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                                   /* Unlock the GPIOCR register */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_COMMIT_REG_OFFSET) , Port_channelsPtr[Pin].port_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
        }
        else if( (PORTC_ID == Port_channelsPtr[Pin].port_num) && (Port_channelsPtr[Pin].pin_num <= PIN3_ID) ) /* PC0 to PC3 */
        {
            /* Do Nothing ...  this is the JTAG pins */
        }
        else
        {
            /* Do Nothing ... No need to unlock the commit register for this pin */
        }
        if(PORT_MODE == Mode)
        {
            /* Setup the pin mode as GPIO */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);                 /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_ALT_FUNC_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);                        /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
            *(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_CTL_REG_OFFSET) &= ~(MODE_MASK << (Port_channelsPtr[Pin].pin_num * NO_OF_MODE_BITS));   /* Clear the PMCx bits for this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);                    /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        }
        else if(ANALOG_MODE == Mode)
        {
            /* Setup the pin as Analog pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);                   /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_ALT_FUNC_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);                        /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
            *(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_CTL_REG_OFFSET) &= ~(MODE_MASK << (Port_channelsPtr[Pin].pin_num * NO_OF_MODE_BITS));   /* Clear the PMCx bits for this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);                  /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
        }
        else
        {
            /* Setup the pin mode for a special purpose */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);                                  /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_ALT_FUNC_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);                                           /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
            *(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_CTL_REG_OFFSET) &= ~(MODE_MASK << (Port_channelsPtr[Pin].pin_num * NO_OF_MODE_BITS));                    /* Clear the PMCx bits for this pin */
            *(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_CTL_REG_OFFSET) |= (Port_channelsPtr[Pin].mode << (Port_channelsPtr[Pin].pin_num * NO_OF_MODE_BITS));  /* Assign the mode value to the corresponding PMCx bits for this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_channelsPtr[Pin].pin_num);                                     /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        }
    }
    else
    {
        /* No Action Required */
    }

}
#endif

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to refreshes port direction.
************************************************************************************/
void Port_RefreshPortDirection(void)
{
    volatile uint32 * Port_registerBaseAddress = NULL_PTR; /* point to the required Port Registers base address */
    uint8 index = 0;

#if (STD_ON == PORT_DEV_ERROR_DETECT)
    /* Check if the Driver is initialized before using this function */
    if(PORT_NOT_INITIALIZED == Port_Status)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_REFRESH_PORT_DIRECTION_SID, PORT_E_UNINIT);
    }
    else
#endif
    {
        while(index<PORT_CONFIGURED_CHANNLES)
        {
            if(STD_OFF == Port_channelsPtr[index].directionChangeable)
            {
                switch(Port_channelsPtr[index].port_num)
                {
                    case  PORTA_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                             break;
                    case  PORTB_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                             break;
                    case  PORTC_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                             break;
                    case  PORTD_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                             break;
                    case  PORTE_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                             break;
                    case  PORTF_ID: Port_registerBaseAddress = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                             break;
                }
                if( ((PORTD_ID == Port_channelsPtr[index].port_num) && (PIN7_ID == Port_channelsPtr[index].pin_num)) || ((PORTF_ID == Port_channelsPtr[index].port_num) && (PIN0_ID == Port_channelsPtr[index].pin_num)) ) /* PD7 or PF0 */
                {
                    *(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                                   /* Unlock the GPIOCR register */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_COMMIT_REG_OFFSET) , Port_channelsPtr[index].port_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                }
                else if( (PORTC_ID == Port_channelsPtr[index].port_num) && (Port_channelsPtr[index].pin_num <= PIN3_ID) ) /* PC0 to PC3 */
                {
                    /* Do Nothing ...  this is the JTAG pins */
                }
                else
                {
                    /* Do Nothing ... No need to unlock the commit register for this pin */
                }
                if(PORT_PIN_OUT == Port_channelsPtr[index].direction)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DIR_REG_OFFSET) , Port_channelsPtr[index].pin_num);            /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

                    if(STD_HIGH == Port_channelsPtr[index].initial_value)
                    {
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DATA_REG_OFFSET) , Port_channelsPtr[index].pin_num);       /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
                    }
                    else
                    {
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DATA_REG_OFFSET) , Port_channelsPtr[index].pin_num);     /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
                    }
                }
                else if(PORT_PIN_IN == Port_channelsPtr[index].direction)
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_DIR_REG_OFFSET) , Port_channelsPtr[index].pin_num);          /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

                    if(PULL_UP == Port_channelsPtr[index].resistor)
                    {
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_PULL_UP_REG_OFFSET) , Port_channelsPtr[index].pin_num);    /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
                    }
                    else if(PULL_DOWN == Port_channelsPtr[index].resistor)
                    {
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_PULL_DOWN_REG_OFFSET) , Port_channelsPtr[index].pin_num);  /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
                    }
                    else
                    {
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_PULL_UP_REG_OFFSET) , Port_channelsPtr[index].pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_registerBaseAddress + PORT_PULL_DOWN_REG_OFFSET) , Port_channelsPtr[index].pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
                    }
                }
                else
                {
                    /* Do Nothing */
                }
            }
            else
            {
                /* No Action Required */
            }
            index++;
        }
    }
}

#if (STD_ON == PORT_VERSION_INFO_API)
/************************************************************************************
* Service Name: Port_GetVersionInfo
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo - Pointer to where to store the version information of this module.
* Return value: None
* Description: Function to return the version information of this module.
************************************************************************************/
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo)
{
#if (STD_ON == PORT_DEV_ERROR_DETECT)
    /* Check if the Driver is initialized before using this function */
    if(PORT_NOT_INITIALIZED == Port_Status)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_UNINIT);
    }
    else
    {
        /* No Action Required */
    }
    if(NULL_PTR == versioninfo)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
    }
    else /* In-case there are no errors */
#endif /* (STD_ON == PORT_DEV_ERROR_DETECT) */
    {
        /* Copy the vendor Id */
        versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
        /* Copy the module Id */
        versioninfo->moduleID = (uint16)PORT_MODULE_ID;
        /* Copy Software Major Version */
        versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
        /* Copy Software Minor Version */
        versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
        /* Copy Software Patch Version */
        versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
    }
    else
    {
        /* No Action Required */
    }
}
#endif /* (STD_ON == PORT_VERSION_INFO_API) */


