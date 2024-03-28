/******************************************************************************
* File Name:    main.c
*
* Description: This is the source code for the PSoC 6 MCU: GPIO Pins 
* example. This code example demonstrates the use of GPIO pins configured
* as inputs, outputs and GPIO interrupts in the PSoC 6 MCU.
*
* Related Document: See README.md 
*
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/******************************************************************************
 * Code Example selection
 *****************************************************************************/
#define EXAMPLE_SELECT HAL_EXAMPLE /* Choose "PDL_EXAMPLE" or "HAL_Example" */

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/******************************************************************************
 * Macros for both examples
 *****************************************************************************/
#define PDL_EXAMPLE 0
#define HAL_EXAMPLE 1
#define GPIO_INTERRUPT_PRIORITY (7u)

/*
 * This code example assumes that the Device Configurator will automatically configure
 * all GPIO pins of the device. To see how PDL drivers are used to manually
 * configure GPIO pins, set the PDL_PIN_CONFIGURATION #define to 1, otherwise leave
 * set to 0. The names of ports and pins will also need to be changed in the PDL code example.
 * This can be done by changing CYBSP_USER_BTN_PORT to P0_4_PORT and CYBSP_USER_BTN_PIN to P0_4_PIN.
 */
#define PDL_PIN_CONFIGURATION 0


/******************************************************************************
 * Definitions and functions for PDL example
 *****************************************************************************/
#if PDL_PIN_CONFIGURATION
/* This structure is used to initialize a single GPIO pin using PDL configuration. */
const cy_stc_gpio_pin_config_t P0_4_Pin_Init =
    {
        .outVal = 1u,                    /* Pin output state */
        .driveMode = CY_GPIO_DM_PULLUP,  /* Drive mode */
        .hsiom = HSIOM_SEL_GPIO,         /* HSIOM selection */
        .intEdge = CY_GPIO_INTR_FALLING, /* Interrupt Edge type */
        .intMask = CY_GPIO_INTR_EN_MASK, /* Interrupt enable mask */
        .vtrip = CY_GPIO_VTRIP_CMOS,     /* Input buffer voltage trip type */
        .slewRate = CY_GPIO_SLEW_FAST,   /* Output buffer slew rate */
        .driveSel = CY_GPIO_DRIVE_FULL,  /* Drive strength */
        .vregEn = 0u,                    /* SIO pair output buffer mode */
        .ibufMode = 0u,                  /* SIO pair input buffer mode */
        .vtripSel = 0u,                  /* SIO pair input buffer trip point */
        .vrefSel = 0u,                   /* SIO pair reference voltage for input buffer trip point */
        .vohSel = 0u                     /* SIO pair regulated voltage output level */
};
#endif
/* This structure is used to initialize a full GPIO Port using PDL configuration */
const cy_stc_gpio_prt_config_t port5_Init =
    {
        .out = 0x000000FFu,        /* Initial output data for the IO pins in the port */
        .intrMask = 0x00000000u,   /* Interrupt enable mask for the port interrupt */
        .intrCfg = 0x00000000u,    /* Port interrupt edge detection configuration */
        .cfg = 0xEEEEEEEEu,        /* Port drive modes and input buffer enable configuration */
        .cfgIn = 0x00000000u,      /* Port input buffer configuration */
        .cfgOut = 0x00000000u,     /* Port output buffer configuration */
        .cfgSIO = 0x00000000u,     /* Port SIO configuration */
        .sel0Active = 0x00000000u, /* HSIOM selection for port pins 0,1,2,3 */
        .sel1Active = 0x00000000u, /* HSIOM selection for port pins 4,5,6,7 */
};
/* This structure initializes the Port0 interrupt for the NVIC */
cy_stc_sysint_t intrCfg =
    {
        .intrSrc = ioss_interrupts_gpio_0_IRQn, /* Interrupt source is GPIO port 0 interrupt */
        .intrPriority = 2UL                     /* Interrupt priority is 2 */
};
uint32 pinState = 0ul;

/*******************************************************************************
* Function Prototypes
********************************************************************************/

static void pdl_code_example();
static void hal_code_example();

static void gpio_interrupt_handler_HAL(void *arg, cyhal_gpio_event_t event);

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Semaphore from interrupt handler to background process */
volatile bool gpio_intr_flag = false;

/*This structure is used to initialize callback*/
cyhal_gpio_callback_data_t cb_data =
    {
        .callback = gpio_interrupt_handler_HAL,
        .callback_arg = NULL
 };

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function runs the example selected by the user.
*
* Return: int
*
*******************************************************************************/
int main(void)
{
    if (EXAMPLE_SELECT == HAL_EXAMPLE)
    {
        hal_code_example();
    }
    if (EXAMPLE_SELECT == PDL_EXAMPLE)
    {
        pdl_code_example();
    }
    else
    {
        CY_ASSERT(0);
    }
}

/*******************************************************************************
* Function Name: gpio_interrupt_handler_HAL
********************************************************************************
* Summary:
*   GPIO interrupt handler for the HAL example.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_irq_event_t (unused)
*
*******************************************************************************/
static void gpio_interrupt_handler_HAL(void *arg, cyhal_gpio_event_t event)
{
    gpio_intr_flag = true;
}

/*******************************************************************************
* Function Name: GPIO_Interrupt_handler_PDL
********************************************************************************
*
*  Summary:
*  GPIO interrupt handler for the PDL example.
*
*  Parameters:
*  None
*
*  Return:
*  None
*
**********************************************************************************/
static void gpio_interrupt_handler_PDL()
{
    gpio_intr_flag = true;

    /* Clear pin interrupt logic. Required to detect next interrupt */
    Cy_GPIO_ClearInterrupt(CYHAL_GET_PORTADDR(CYBSP_USER_BTN), CYHAL_GET_PIN(CYBSP_USER_BTN));
}

/*******************************************************************************
* Function Name: pdl_code_example
********************************************************************************
* Summary:
*   PDL version of the GPIO code example. This function is run if
*   #EXAMPLE_SELECT is set to "PDL_EXAMPLE"
*
* Parameters:
*  None
*
*******************************************************************************/
static void pdl_code_example()
{
    cy_rslt_t result;
    volatile bool read_val = false;
    uint32 portReadValue = 0ul;

/*This section defines the port and pins for the USER_LED and USER_BTN.
      The CYBSP macros for these resources are used in the HAL section. The macros
      are translated using the "CYHAL_GET_PORTADDR" and "CYHAL_GET_PIN" macro expansions to
      translate the CYBSP macros into the port and pin types that are used in PDL functions.
      This is not needed normally unless you are wanting to use the HAL BSP names.
    */

/* Port and pin translations for the USER_BTN */
#if PDL_PIN_CONFIGURATION
    GPIO_PRT_Type *CYBSP_USER_LED_PORT = P0_4_PORT;
    uint8_t CYBSP_USER_LED_PIN = P0_4_PIN;
#else
    GPIO_PRT_Type *CYBSP_USER_BTN_PORT = CYHAL_GET_PORTADDR(CYBSP_USER_BTN);
    uint8_t CYBSP_USER_BTN_PIN = CYHAL_GET_PIN(CYBSP_USER_BTN);
#endif

    /* Port and pin translations for the USER_LED */
    GPIO_PRT_Type *CYBSP_USER_LED_PORT = CYHAL_GET_PORTADDR(CYBSP_USER_LED);
    uint8_t CYBSP_USER_LED_PIN = CYHAL_GET_PIN(CYBSP_USER_LED);

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Enable global interrupts */
    __enable_irq();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* The most code efficient method to configure all attributes for a full port of pins */
    /* is to use the Cy_GPIO_Port_Init() API function and configuration structure. It packs all */
    /* the configuration data into direct register writes for the whole port. Its limitation */
    /* is that it must configure all pins in a port and the user must calculate the */
    /* combined register values for all pins. */
    Cy_GPIO_Port_Init(GPIO_PRT5, &port5_Init);

    /* Initialize USER_LED */
    Cy_GPIO_Pin_FastInit(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CY_GPIO_DM_STRONG, 1UL, HSIOM_SEL_GPIO);

    /* Initialize the user button */
    Cy_GPIO_Pin_FastInit(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN, CY_GPIO_DM_STRONG, 1UL, HSIOM_SEL_GPIO);

    /* Pin Interrupts */
    /* Configure GPIO pin to generate interrupts */
    Cy_GPIO_SetInterruptEdge(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN, CY_GPIO_INTR_RISING);
    Cy_GPIO_SetInterruptMask(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN, CY_GPIO_INTR_EN_MASK);

    /* Configure CM4+ CPU GPIO interrupt vector for Port 0 */
    Cy_SysInt_Init(&intrCfg, gpio_interrupt_handler_PDL);
    NVIC_ClearPendingIRQ(intrCfg.intrSrc);
    NVIC_EnableIRQ((IRQn_Type)intrCfg.intrSrc);



    for (;;)
    {
        /* Read current button state from the user button on pin 0_4 */
        read_val = Cy_GPIO_Read(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN);

        /* If button released, LED OFF */
        if (read_val == true)
        {
            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_OFF);
        }
        /* If button pressed, LED ON */
        if (read_val == false)
        {
            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_ON);
        }

        /* Check the interrupt status */
        if (true == gpio_intr_flag)
        {
            /* Reset interrupt flag */
            gpio_intr_flag = false;

            /* Flash LED twice */

                /* LED OFF */
                /* Using general PDL Write function */
                Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_OFF);
                Cy_SysLib_Delay(500);

                /* LED ON */
                /* Using general PDL Write function */
                Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_ON);
                Cy_SysLib_Delay(500);

                /* LED OFF */
                /* Using pin invert function, inverts the current state of the pin */
                Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
                Cy_SysLib_Delay(500);

                /* LED ON */
                /* Using pin Clear function, sets the pin output to logic state Low. */
                Cy_GPIO_Clr(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
                Cy_SysLib_Delay(500);

                /* LED OFF */
                /* Using pin Set function, sets the pin output to logic state High. */
                Cy_GPIO_Set(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
        }

        /* Simultaneous Port Pin access */
        /*******************************************************************************/
        /* Direct register access is used to interface with multiple pins in one port */
        /* at the same time. May not be thread or multi-core safe due to possible */
        /* read-modify-write operations. All pins in a Port under direct register */
        /* control should only be accessed by a single CPU core. */

        portReadValue = GPIO_PRT5->IN;
        portReadValue++;
        GPIO_PRT5->OUT = portReadValue;
    }
}

/*******************************************************************************
* Function Name: hal_code_example
********************************************************************************
* Summary:
*    HAL version of the GPIO code example. This function is run if
*    #EXAMPLE_SELECT is set to "HAL_EXAMPLE"
*
* Parameters:
*  None
*
*******************************************************************************/
static void hal_code_example()
{
    cy_rslt_t result;
    bool read_val = false;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the user LED  */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
                             CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Initialize the user button */
    result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                             CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);

    /* Configure GPIO interrupt */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &cb_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_RISE,
                            GPIO_INTERRUPT_PRIORITY, true);

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
        /* Read current button state from the user button */
        read_val = cyhal_gpio_read(CYBSP_USER_BTN);

        /* If button released, LED OFF */
        if (read_val == true)
        {
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
        }
        /* If button pressed, LED ON */
        if (read_val == false)
        {
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
        }

        /* Check the interrupt status */
        if (true == gpio_intr_flag)
        {
            /* Reset interrupt flag */
            gpio_intr_flag = false;

            /* Flash LED twice */

                /* Turn OFF user LED */
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
                cyhal_system_delay_ms(500);

                /* HAL Write Function, LED ON */
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
                cyhal_system_delay_ms(500);

                /* HAL toggle Function, LED OFF */
                /* Invert CYBSP_USER_LED2 (out value = ~(out value)) */
                cyhal_gpio_toggle(CYBSP_USER_LED);
                cyhal_system_delay_ms(500);

                /* HAL Write Function, LED ON */
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
                cyhal_system_delay_ms(500);

                /* HAL Write Function, LED OFF */
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
                cyhal_system_delay_ms(500);

            /* All of the functions above can also be used with the port
             * and pin number, such as:
             * cyhal_gpio_write(P7_1, CYBSP_LED_STATE_OFF);
             */
        }
    }
}
/* [] END OF FILE */
