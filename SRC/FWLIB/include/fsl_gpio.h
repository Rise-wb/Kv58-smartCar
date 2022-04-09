/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SDRVL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FSL_GPIO_H_
#define _FSL_GPIO_H_

#include "fsl_common.h"
#include "fsl_port.h"
/*!
 * @addtogroup gpio
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief GPIO driver version 2.1.0. */
#define FSL_GPIO_DRIVER_VERSION (MAKE_VERSION(2, 1, 0))
/*@}*/

/*! @brief GPIO direction definition */
typedef enum _gpio_pin_direction
{
    kGPIO_DigitalInput = 0U,  /*!< Set current pin as digital input*/
    kGPIO_DigitalOutput = 1U, /*!< Set current pin as digital output*/
} gpio_pin_direction_t;

#if defined(FSL_FEATURE_GPIO_HAS_ATTRIBUTE_CHECKER) && FSL_FEATURE_GPIO_HAS_ATTRIBUTE_CHECKER
/*! @brief GPIO checker attribute */
typedef enum _gpio_checker_attribute
{
    kGPIO_UsernonsecureRWUsersecureRWPrivilegedsecureRW =
        0x00U, /*!< User nonsecure:Read+Write; User Secure:Read+Write; Privileged Secure:Read+Write */
    kGPIO_UsernonsecureRUsersecureRWPrivilegedsecureRW =
        0x01U, /*!< User nonsecure:Read;       User Secure:Read+Write; Privileged Secure:Read+Write */
    kGPIO_UsernonsecureNUsersecureRWPrivilegedsecureRW =
        0x02U, /*!< User nonsecure:None;       User Secure:Read+Write; Privileged Secure:Read+Write */
    kGPIO_UsernonsecureRUsersecureRPrivilegedsecureRW =
        0x03U, /*!< User nonsecure:Read;       User Secure:Read;       Privileged Secure:Read+Write */
    kGPIO_UsernonsecureNUsersecureRPrivilegedsecureRW =
        0x04U, /*!< User nonsecure:None;       User Secure:Read;       Privileged Secure:Read+Write */
    kGPIO_UsernonsecureNUsersecureNPrivilegedsecureRW =
        0x05U, /*!< User nonsecure:None;       User Secure:None;       Privileged Secure:Read+Write */
    kGPIO_UsernonsecureNUsersecureNPrivilegedsecureR =
        0x06U, /*!< User nonsecure:None;       User Secure:None;       Privileged Secure:Read */
    kGPIO_UsernonsecureNUsersecureNPrivilegedsecureN =
        0x07U, /*!< User nonsecure:None;       User Secure:None;       Privileged Secure:None */
    kGPIO_IgnoreAttributeCheck = 0x10U, /*!< Ignores the attribute check */
} gpio_checker_attribute_t;
#endif

/*!
 * @brief The GPIO pin configuration structure.
 *
 * Each pin can only be configured as either an output pin or an input pin at a time.
 * If configured as an input pin, leave the outputConfig unused.
 * Note : In some use cases, the corresponding port property should be configured in advance
 *        with the PORT_SetPinConfig().
 */
typedef struct _gpio_pin_config
{
    gpio_pin_direction_t pinDirection; /*!< GPIO direction, input or output */
    /* Output configurations; ignore if configured as an input pin */
    uint8_t outputLogic; /*!< Set a default output logic, which has no use in input */
} gpio_pin_config_t;

/*! @} */
typedef enum GPIO_CFG
{
    
    GPI         = 0,                                
    GPO         = 1,                                
} GPIO_CFG;

#define HIGH  1u
#define LOW   0u

typedef struct
{
	  GPIO_Type* pinGPIO;
		uint32_t   GPIO_bit;
		port_mux_t  PinFunc;
		PORT_Type* pinPORT;
 }GPIO_port;

//-----------------------¹Ü½Å-----------------
extern  GPIO_port  PTA0 ;
extern  GPIO_port  PTA1 ;
extern  GPIO_port  PTA2 ;
extern  GPIO_port  PTA3 ;
extern  GPIO_port  PTA4 ;
extern  GPIO_port  PTA5 ;
extern  GPIO_port  PTA6 ;
extern  GPIO_port  PTA7 ;
extern  GPIO_port PTA8 ;
extern  GPIO_port PTA9 ;
extern  GPIO_port PTA10 ;
extern  GPIO_port PTA11 ;
extern  GPIO_port PTA12 ;
extern  GPIO_port PTA13 ;
extern  GPIO_port PTA14 ;
extern  GPIO_port PTA15 ;
extern  GPIO_port PTA16 ;
extern  GPIO_port PTA17 ;
extern  GPIO_port PTA18 ;
extern  GPIO_port PTA19 ;
extern  GPIO_port PTA24 ;
extern  GPIO_port PTA25 ;
extern  GPIO_port PTA26 ;
extern  GPIO_port PTA27 ;
extern  GPIO_port PTA28 ;
extern  GPIO_port PTA29 ;
extern  GPIO_port PTB0 ;
extern  GPIO_port PTB1 ;
extern  GPIO_port PTB2 ;
extern  GPIO_port PTB3 ;
extern  GPIO_port PTB4 ;
extern  GPIO_port PTB5 ;
extern  GPIO_port PTB6 ;
extern  GPIO_port PTB7 ;
extern  GPIO_port PTB8 ;
extern  GPIO_port PTB9 ;
extern  GPIO_port PTB10 ;
extern  GPIO_port PTB11 ;
extern  GPIO_port PTB16 ;
extern  GPIO_port PTB17 ;
extern  GPIO_port PTB18 ;
extern  GPIO_port PTB19 ;
extern  GPIO_port PTB20 ;
extern  GPIO_port PTB21 ;
extern  GPIO_port PTB22 ;
extern  GPIO_port PTB23 ;
extern  GPIO_port PTC0 ;
extern  GPIO_port PTC1 ;
extern  GPIO_port PTC2 ;
extern  GPIO_port PTC3 ;
extern  GPIO_port PTC4 ;
extern  GPIO_port PTC5 ;
extern  GPIO_port PTC6 ;
extern  GPIO_port PTC7 ;
extern  GPIO_port PTC8 ;
extern  GPIO_port PTC9 ;
extern  GPIO_port PTC10 ;
extern  GPIO_port PTC11 ;
extern  GPIO_port PTC12 ;
extern  GPIO_port PTC13 ;
extern  GPIO_port PTC14 ;
extern  GPIO_port PTC15 ;
extern  GPIO_port PTC16 ;
extern  GPIO_port PTC17 ;
extern  GPIO_port PTC18 ;
extern  GPIO_port PTC19 ;
extern  GPIO_port PTD0 ;
extern  GPIO_port PTD1 ;
extern  GPIO_port PTD2 ;
extern  GPIO_port PTD3 ;
extern  GPIO_port PTD4 ;
extern  GPIO_port PTD5 ;
extern  GPIO_port PTD6 ;
extern  GPIO_port PTD7 ;
extern  GPIO_port PTD8 ;
extern  GPIO_port PTD9 ;
extern  GPIO_port PTD10 ;
extern  GPIO_port PTD11 ;
extern  GPIO_port PTD12 ;
extern  GPIO_port PTD13 ;
extern  GPIO_port PTD14 ;
extern  GPIO_port PTD15 ;
extern  GPIO_port PTE0 ;
extern  GPIO_port PTE1 ;
extern  GPIO_port PTE2 ;
extern  GPIO_port PTE3 ;
extern  GPIO_port PTE4 ;
extern  GPIO_port PTE5 ;
extern  GPIO_port PTE6 ;
extern  GPIO_port PTE7 ;
extern  GPIO_port PTE8 ;
extern  GPIO_port PTE9 ;
extern  GPIO_port PTE10 ;
extern  GPIO_port PTE11 ;
extern  GPIO_port PTE12 ;
extern  GPIO_port PTE13 ;
extern  GPIO_port PTE16 ;
extern  GPIO_port PTE17 ;
extern  GPIO_port PTE18 ;
extern  GPIO_port PTE19 ;
extern  GPIO_port PTE20 ;
extern  GPIO_port PTE21 ;
extern  GPIO_port PTE22 ;
extern  GPIO_port PTE23 ;
extern  GPIO_port PTE24 ;
extern  GPIO_port PTE25 ;
extern  GPIO_port PTE26 ;
extern  GPIO_port PTE27 ;
extern  GPIO_port PTE28 ;
extern  GPIO_port PTE29 ;
extern  GPIO_port PTE30 ;
/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif
//API for user
extern	void gpio_init (GPIO_port* P_port, GPIO_CFG gpio_cfg, uint8_t data);
extern  uint8_t gpio_get(GPIO_port* P_port);
extern  void gpio_set (GPIO_port* P_port, uint8_t data);
extern  void gpio_ddr (GPIO_port* P_port, GPIO_CFG gpio_cfg);
extern  void port_init_NoALT(GPIO_port* P_port,port_cfg cfg);
extern  void port_init(GPIO_port* P_port,port_cfg cfg);
/*!
 * @addtogroup gpio_driver
 * @{
 */

/*! @name GPIO Configuration */
/*@{*/

/*!
 * @brief Initializes a GPIO pin used by the board.
 *
 * To initialize the GPIO, define a pin configuration, as either input or output, in the user file.
 * Then, call the GPIO_PinInit() function.
 *
 * This is an example to define an input pin or an output pin configuration:
 * @code
 * // Define a digital input pin configuration,
 * gpio_pin_config_t config =
 * {
 *   kGPIO_DigitalInput,
 *   0,
 * }
 * //Define a digital output pin configuration,
 * gpio_pin_config_t config =
 * {
 *   kGPIO_DigitalOutput,
 *   0,
 * }
 * @endcode
 *
 * @param base   GPIO peripheral base pointer (GPIOA, GPIOB, GPIOC, and so on.)
 * @param pin    GPIO port pin number
 * @param config GPIO pin configuration pointer
 */
void GPIO_PinInit(GPIO_Type* base, uint32_t pin, const gpio_pin_config_t* config);

/*@}*/

/*! @name GPIO Output Operations */
/*@{*/

/*!
 * @brief Sets the output level of the multiple GPIO pins to the logic 1 or 0.
 *
 * @param base    GPIO peripheral base pointer (GPIOA, GPIOB, GPIOC, and so on.)
 * @param pin     GPIO pin number
 * @param output  GPIO pin output logic level.
 *        - 0: corresponding pin output low-logic level.
 *        - 1: corresponding pin output high-logic level.
 */
static inline void GPIO_WritePinOutput(GPIO_Type* base, uint32_t pin, uint8_t output)
{
    if (output == 0U)
    {
        base->PCOR = 1U << pin;
    }
    else
    {
        base->PSOR = 1U << pin;
    }
}

/*!
 * @brief Sets the output level of the multiple GPIO pins to the logic 1.
 *
 * @param base GPIO peripheral base pointer (GPIOA, GPIOB, GPIOC, and so on.)
 * @param mask GPIO pin number macro
 */
static inline void GPIO_SetPinsOutput(GPIO_Type* base, uint32_t mask)
{
    base->PSOR = mask;
}

/*!
 * @brief Sets the output level of the multiple GPIO pins to the logic 0.
 *
 * @param base GPIO peripheral base pointer (GPIOA, GPIOB, GPIOC, and so on.)
 * @param mask GPIO pin number macro
 */
static inline void GPIO_ClearPinsOutput(GPIO_Type* base, uint32_t mask)
{
    base->PCOR = mask;
}

/*!
 * @brief Reverses the current output logic of the multiple GPIO pins.
 *
 * @param base GPIO peripheral base pointer (GPIOA, GPIOB, GPIOC, and so on.)
 * @param mask GPIO pin number macro
 */
static inline void GPIO_TogglePinsOutput(GPIO_Type* base, uint32_t mask)
{
    base->PTOR = mask;
}
/*@}*/

/*! @name GPIO Input Operations */
/*@{*/

/*!
 * @brief Reads the current input value of the GPIO port.
 *
 * @param base GPIO peripheral base pointer (GPIOA, GPIOB, GPIOC, and so on.)
 * @param pin     GPIO pin number
 * @retval GPIO port input value
 *        - 0: corresponding pin input low-logic level.
 *        - 1: corresponding pin input high-logic level.
 */
static inline uint32_t GPIO_ReadPinInput(GPIO_Type* base, uint32_t pin)
{
    return (((base->PDIR) >> pin) & 0x01U);
}
/*@}*/

/*! @name GPIO Interrupt */
/*@{*/

/*!
 * @brief Reads the GPIO port interrupt status flag.
 *
 * If a pin is configured to generate the DMA request, the corresponding flag
 * is cleared automatically at the completion of the requested DMA transfer.
 * Otherwise, the flag remains set until a logic one is written to that flag.
 * If configured for a level sensitive interrupt that remains asserted, the flag
 * is set again immediately.
 *
 * @param base GPIO peripheral base pointer (GPIOA, GPIOB, GPIOC, and so on.)
 * @retval The current GPIO port interrupt status flag, for example, 0x00010001 means the
 *         pin 0 and 17 have the interrupt.
 */
uint32_t GPIO_GetPinsInterruptFlags(GPIO_Type* base);

/*!
 * @brief Clears multiple GPIO pin interrupt status flags.
 *
 * @param base GPIO peripheral base pointer (GPIOA, GPIOB, GPIOC, and so on.)
 * @param mask GPIO pin number macro
 */
void GPIO_ClearPinsInterruptFlags(GPIO_Type* base, uint32_t mask);

#if defined(FSL_FEATURE_GPIO_HAS_ATTRIBUTE_CHECKER) && FSL_FEATURE_GPIO_HAS_ATTRIBUTE_CHECKER
/*!
 * @brief The GPIO module supports a device-specific number of data ports, organized as 32-bit
 * words. Each 32-bit data port includes a GACR register, which defines the byte-level
 * attributes required for a successful access to the GPIO programming model. The attribute controls for the 4 data
 * bytes in the GACR follow a standard little endian
 * data convention.
 *
 * @param base GPIO peripheral base pointer (GPIOA, GPIOB, GPIOC, and so on.)
 * @param mask GPIO pin number macro
 */
void GPIO_CheckAttributeBytes(GPIO_Type* base, gpio_checker_attribute_t attribute);
#endif

/*@}*/
/*! @} */

/*!
 * @addtogroup fgpio_driver
 * @{
 */

/*
 * Introduces the FGPIO feature.
 *
 * The FGPIO features are only support on some Kinetis MCUs. The FGPIO registers are aliased to the IOPORT
 * interface. Accesses via the IOPORT interface occur in parallel with any instruction fetches and
 * complete in a single cycle. This aliased Fast GPIO memory map is called FGPIO.
 */

#if defined(FSL_FEATURE_SOC_FGPIO_COUNT) && FSL_FEATURE_SOC_FGPIO_COUNT

/*! @name FGPIO Configuration */
/*@{*/

/*!
 * @brief Initializes a FGPIO pin used by the board.
 *
 * To initialize the FGPIO driver, define a pin configuration, as either input or output, in the user file.
 * Then, call the FGPIO_PinInit() function.
 *
 * This is an example to define an input pin or an output pin configuration:
 * @code
 * // Define a digital input pin configuration,
 * gpio_pin_config_t config =
 * {
 *   kGPIO_DigitalInput,
 *   0,
 * }
 * //Define a digital output pin configuration,
 * gpio_pin_config_t config =
 * {
 *   kGPIO_DigitalOutput,
 *   0,
 * }
 * @endcode
 *
 * @param base   FGPIO peripheral base pointer (FGPIOA, FGPIOB, FGPIOC, and so on.)
 * @param pin    FGPIO port pin number
 * @param config FGPIO pin configuration pointer
 */
void FGPIO_PinInit(FGPIO_Type* base, uint32_t pin, const gpio_pin_config_t* config);

/*@}*/

/*! @name FGPIO Output Operations */
/*@{*/

/*!
 * @brief Sets the output level of the multiple FGPIO pins to the logic 1 or 0.
 *
 * @param base    FGPIO peripheral base pointer (FGPIOA, FGPIOB, FGPIOC, and so on.)
 * @param pin     FGPIO pin number
 * @param output  FGPIOpin output logic level.
 *        - 0: corresponding pin output low-logic level.
 *        - 1: corresponding pin output high-logic level.
 */
static inline void FGPIO_WritePinOutput(FGPIO_Type* base, uint32_t pin, uint8_t output)
{
    if (output == 0U)
    {
        base->PCOR = 1 << pin;
    }
    else
    {
        base->PSOR = 1 << pin;
    }
}

/*!
 * @brief Sets the output level of the multiple FGPIO pins to the logic 1.
 *
 * @param base FGPIO peripheral base pointer (FGPIOA, FGPIOB, FGPIOC, and so on.)
 * @param mask FGPIO pin number macro
 */
static inline void FGPIO_SetPinsOutput(FGPIO_Type* base, uint32_t mask)
{
    base->PSOR = mask;
}

/*!
 * @brief Sets the output level of the multiple FGPIO pins to the logic 0.
 *
 * @param base FGPIO peripheral base pointer (FGPIOA, FGPIOB, FGPIOC, and so on.)
 * @param mask FGPIO pin number macro
 */
static inline void FGPIO_ClearPinsOutput(FGPIO_Type* base, uint32_t mask)
{
    base->PCOR = mask;
}

/*!
 * @brief Reverses the current output logic of the multiple FGPIO pins.
 *
 * @param base FGPIO peripheral base pointer (FGPIOA, FGPIOB, FGPIOC, and so on.)
 * @param mask FGPIO pin number macro
 */
static inline void FGPIO_TogglePinsOutput(FGPIO_Type* base, uint32_t mask)
{
    base->PTOR = mask;
}
/*@}*/

/*! @name FGPIO Input Operations */
/*@{*/

/*!
 * @brief Reads the current input value of the FGPIO port.
 *
 * @param base FGPIO peripheral base pointer (FGPIOA, FGPIOB, FGPIOC, and so on.)
 * @param pin  FGPIO pin number
 * @retval FGPIO port input value
 *        - 0: corresponding pin input low-logic level.
 *        - 1: corresponding pin input high-logic level.
 */
static inline uint32_t FGPIO_ReadPinInput(FGPIO_Type* base, uint32_t pin)
{
    return (((base->PDIR) >> pin) & 0x01U);
}
/*@}*/

/*! @name FGPIO Interrupt */
/*@{*/

/*!
 * @brief Reads the FGPIO port interrupt status flag.
 *
 * If a pin is configured to generate the DMA request, the corresponding flag
 * is cleared automatically at the completion of the requested DMA transfer.
 * Otherwise, the flag remains set until a logic one is written to that flag.
 * If configured for a level-sensitive interrupt that remains asserted, the flag
 * is set again immediately.
 *
 * @param base FGPIO peripheral base pointer (FGPIOA, FGPIOB, FGPIOC, and so on.)
 * @retval The current FGPIO port interrupt status flags, for example, 0x00010001 means the
 *         pin 0 and 17 have the interrupt.
 */
uint32_t FGPIO_GetPinsInterruptFlags(FGPIO_Type* base);

/*!
 * @brief Clears the multiple FGPIO pin interrupt status flag.
 *
 * @param base FGPIO peripheral base pointer (FGPIOA, FGPIOB, FGPIOC, and so on.)
 * @param mask FGPIO pin number macro
 */
void FGPIO_ClearPinsInterruptFlags(FGPIO_Type* base, uint32_t mask);

#if defined(FSL_FEATURE_GPIO_HAS_ATTRIBUTE_CHECKER) && FSL_FEATURE_GPIO_HAS_ATTRIBUTE_CHECKER
/*!
 * @brief The FGPIO module supports a device-specific number of data ports, organized as 32-bit
 * words. Each 32-bit data port includes a GACR register, which defines the byte-level
 * attributes required for a successful access to the GPIO programming model. The attribute controls for the 4 data
 * bytes in the GACR follow a standard little endian
 * data convention
 *
 * @param base FGPIO peripheral base pointer (FGPIOA, FGPIOB, FGPIOC, and so on.)
 * @param mask FGPIO pin number macro
 */
void FGPIO_CheckAttributeBytes(FGPIO_Type* base, gpio_checker_attribute_t attribute);
#endif

/*@}*/

#endif /* FSL_FEATURE_SOC_FGPIO_COUNT */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */

#endif /* _FSL_GPIO_H_*/
