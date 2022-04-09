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
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_gpio.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
static PORT_Type* const s_portBases[] = PORT_BASE_PTRS;
static GPIO_Type* const s_gpioBases[] = GPIO_BASE_PTRS;

//                GPIO_port   GPIO_bit   Func						PORT
 GPIO_port PTA0 = {GPIOA ,     0u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA1 = {GPIOA ,     1u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA2 = {GPIOA ,     2u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA3 = {GPIOA ,     3u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA4 = {GPIOA ,     4u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA5 = {GPIOA ,     5u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA6 = {GPIOA ,     6u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA7 = {GPIOA ,     7u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA8 = {GPIOA ,     8u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA9 = {GPIOA ,     9u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA10 = {GPIOA ,     10u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA11 = {GPIOA ,     11u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA12 = {GPIOA ,     12u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA13 = {GPIOA ,     13u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA14 = {GPIOA ,     14u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA15 = {GPIOA ,     15u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA16 = {GPIOA ,     16u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA17 = {GPIOA ,     17u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA18 = {GPIOA ,     18u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA19 = {GPIOA ,     19u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA24 = {GPIOA ,     24u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA25 = {GPIOA ,     25u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA26 = {GPIOA ,     26u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA27 = {GPIOA ,     27u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA28 = {GPIOA ,     28u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTA29 = {GPIOA ,     29u,      kPORT_MuxAsGpio , PORTA};
 GPIO_port PTB0 = {GPIOB ,     0u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB1 = {GPIOB ,     1u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB2 = {GPIOB ,     2u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB3 = {GPIOB ,     3u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB4 = {GPIOB ,     4u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB5 = {GPIOB ,     5u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB6 = {GPIOB ,     6u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB7 = {GPIOB ,     7u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB8 = {GPIOB ,     8u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB9 = {GPIOB ,     9u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB10 = {GPIOB ,     10u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB11 = {GPIOB ,     11u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB16 = {GPIOB ,     16u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB17 = {GPIOB ,     17u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB18 = {GPIOB ,     18u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB19 = {GPIOB ,     19u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB20 = {GPIOB ,     20u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB21 = {GPIOB ,     21u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB22 = {GPIOB ,     22u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTB23 = {GPIOB ,     23u,      kPORT_MuxAsGpio , PORTB};
 GPIO_port PTC0 = {GPIOC ,     0u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC1 = {GPIOC ,     1u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC2 = {GPIOC ,     2u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC3 = {GPIOC ,     3u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC4 = {GPIOC ,     4u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC5 = {GPIOC ,     5u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC6 = {GPIOC ,     6u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC7 = {GPIOC ,     7u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC8 = {GPIOC ,     8u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC9 = {GPIOC ,     9u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC10 = {GPIOC ,     10u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC11 = {GPIOC ,     11u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC12 = {GPIOC ,     12u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC13 = {GPIOC ,     13u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC14 = {GPIOC ,     14u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC15 = {GPIOC ,     15u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC16 = {GPIOC ,     16u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC17 = {GPIOC ,     17u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC18 = {GPIOC ,     18u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTC19 = {GPIOC ,     19u,      kPORT_MuxAsGpio , PORTC};
 GPIO_port PTD0 = {GPIOD ,     0u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTD1 = {GPIOD ,     1u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTD2 = {GPIOD ,     2u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTD3 = {GPIOD ,     3u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTD4 = {GPIOD ,     4u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTD5 = {GPIOD ,     5u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTD6 = {GPIOD ,     6u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTD7 = {GPIOD ,     7u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTD8 = {GPIOD ,     8u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTD9 = {GPIOD ,     9u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTD10 = {GPIOD ,     10u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTD11 = {GPIOD ,     11u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTD12 = {GPIOD ,     12u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTD13 = {GPIOD ,     13u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTD14 = {GPIOD ,     14u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTD15 = {GPIOD ,     15u,      kPORT_MuxAsGpio , PORTD};
 GPIO_port PTE0 = {GPIOE ,     0u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE1 = {GPIOE ,     1u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE2 = {GPIOE ,     2u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE3 = {GPIOE ,     3u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE4 = {GPIOE ,     4u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE5 = {GPIOE ,     5u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE6 = {GPIOE ,     6u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE7 = {GPIOE ,     7u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE8 = {GPIOE ,     8u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE9 = {GPIOE ,     9u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE10 = {GPIOE ,     10u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE11 = {GPIOE ,     11u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE12 = {GPIOE ,     12u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE13 = {GPIOE ,     13u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE16 = {GPIOE ,     16u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE17 = {GPIOE ,     17u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE18 = {GPIOE ,     18u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE19 = {GPIOE ,     19u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE20 = {GPIOE ,     20u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE21 = {GPIOE ,     21u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE22 = {GPIOE ,     22u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE23 = {GPIOE ,     23u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE24 = {GPIOE ,     24u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE25 = {GPIOE ,     25u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE26 = {GPIOE ,     26u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE27 = {GPIOE ,     27u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE28 = {GPIOE ,     28u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE29 = {GPIOE ,     29u,      kPORT_MuxAsGpio , PORTE};
 GPIO_port PTE30 = {GPIOE ,     30u,      kPORT_MuxAsGpio , PORTE};


/*******************************************************************************
* Prototypes
******************************************************************************/
void gpio_init  (GPIO_port* P_port, GPIO_CFG gpio_cfg, uint8_t data)   
{
		gpio_pin_config_t GPconfig_t;
	  if(gpio_cfg == GPI) GPconfig_t.pinDirection = kGPIO_DigitalInput;
	  else                GPconfig_t.pinDirection = kGPIO_DigitalOutput;
	  if(data == HIGH)    GPconfig_t.outputLogic  =  1u;
	  else       					GPconfig_t.outputLogic  =  0u;
	  port_init(P_port,ALT1);
	  GPIO_PinInit(P_port->pinGPIO, P_port->GPIO_bit, &GPconfig_t);
}

uint8_t gpio_get(GPIO_port* P_port)
{
		if( GPIO_ReadPinInput(P_port->pinGPIO, P_port->GPIO_bit) )
	     return 1;
	  else
			 return 0;
}

void gpio_set(GPIO_port* P_port, uint8_t data)
{
	  GPIO_WritePinOutput((GPIO_Type*)P_port->pinGPIO,  P_port->GPIO_bit, data);
}

void gpio_ddr (GPIO_port* P_port, GPIO_CFG gpio_cfg)
{
			if(gpio_cfg == GPI)
			{
				  P_port->pinGPIO->PDDR &= ~(1U << (P_port->GPIO_bit));
			}
			else 
			{
					P_port->pinGPIO->PDDR |=  (1U << (P_port->GPIO_bit));
			}
}
/*!
 *  Sample usage:       port_init_NoALT (&PTA8, IRQ_RISING | PF | PULLUP );    //
 */
void port_init_NoALT(GPIO_port* P_port,port_cfg cfg)
{			
	    cfg &= ~PORT_PCR_MUX_MASK;                       //清空MUX
	
	    cfg |=  ((((PORT_Type*)(P_port->pinPORT))->PCR[P_port->GPIO_bit]) & PORT_PCR_MUX_MASK);  //保持原来的MUX
	    
			((PORT_Type*)(P_port->pinPORT))->ISFR = (1<<P_port->GPIO_bit);   //清空标志位
	
	    port_cfg* cfg_u16 =&cfg;  	
	
			PORT_SetPinConfig(P_port->pinPORT, P_port->GPIO_bit, (port_pin_config_t*)cfg_u16);
}
/*!
 *  Sample usage:       port_init (&PTA8, IRQ_RISING | PF | ALT1 | PULLUP );    //
 */
void  port_init(GPIO_port* P_port,port_cfg cfg)
{			
			CLOCK_EnablePortClock(P_port->pinPORT);    //使能时钟
	
			port_cfg* cfg_u16 =&cfg;  	 
	
			((PORT_Type*)(P_port->pinPORT))->ISFR = (1<<P_port->GPIO_bit);   //清空标志位
	
			PORT_SetPinConfig(P_port->pinPORT, P_port->GPIO_bit, (port_pin_config_t*)cfg_u16);
}
/*!
* @brief Gets the GPIO instance according to the GPIO base
*
* @param base    GPIO peripheral base pointer(PTA, PTB, PTC, etc.)
* @retval GPIO instance
*/
static uint32_t GPIO_GetInstance(GPIO_Type* base);

/*******************************************************************************
 * Code
 ******************************************************************************/

static uint32_t GPIO_GetInstance(GPIO_Type* base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < FSL_FEATURE_SOC_GPIO_COUNT; instance++)
    {
        if (s_gpioBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < FSL_FEATURE_SOC_GPIO_COUNT);

    return instance;
}

void GPIO_PinInit(GPIO_Type* base, uint32_t pin, const gpio_pin_config_t* config)
{
    assert(config);

    if (config->pinDirection == kGPIO_DigitalInput)
    {
        base->PDDR &= ~(1U << pin);
    }
    else
    {
        GPIO_WritePinOutput(base, pin, config->outputLogic);
        base->PDDR |= (1U << pin);
    }
}

uint32_t GPIO_GetPinsInterruptFlags(GPIO_Type* base)
{
    uint8_t instance;
    PORT_Type* portBase;
    instance = GPIO_GetInstance(base);
    portBase = s_portBases[instance];
    return portBase->ISFR;
}

void GPIO_ClearPinsInterruptFlags(GPIO_Type* base, uint32_t mask)
{
    uint8_t instance;
    PORT_Type* portBase;
    instance = GPIO_GetInstance(base);
    portBase = s_portBases[instance];
    portBase->ISFR = mask;
}

#if defined(FSL_FEATURE_GPIO_HAS_ATTRIBUTE_CHECKER) && FSL_FEATURE_GPIO_HAS_ATTRIBUTE_CHECKER
void GPIO_CheckAttributeBytes(GPIO_Type* base, gpio_checker_attribute_t attribute)
{
    base->GACR = (attribute << GPIO_GACR_ACB0_SHIFT) | (attribute << GPIO_GACR_ACB1_SHIFT) |
                 (attribute << GPIO_GACR_ACB2_SHIFT) | (attribute << GPIO_GACR_ACB3_SHIFT);
}
#endif

#if defined(FSL_FEATURE_SOC_FGPIO_COUNT) && FSL_FEATURE_SOC_FGPIO_COUNT

/*******************************************************************************
 * Variables
 ******************************************************************************/
static FGPIO_Type* const s_fgpioBases[] = FGPIO_BASE_PTRS;

/*******************************************************************************
* Prototypes
******************************************************************************/
/*!
* @brief Gets the FGPIO instance according to the GPIO base
*
* @param base    FGPIO peripheral base pointer(PTA, PTB, PTC, etc.)
* @retval FGPIO instance
*/
static uint32_t FGPIO_GetInstance(FGPIO_Type* base);

/*******************************************************************************
 * Code
 ******************************************************************************/

static uint32_t FGPIO_GetInstance(FGPIO_Type* base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < FSL_FEATURE_SOC_FGPIO_COUNT; instance++)
    {
        if (s_fgpioBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < FSL_FEATURE_SOC_FGPIO_COUNT);

    return instance;
}

void FGPIO_PinInit(FGPIO_Type* base, uint32_t pin, const gpio_pin_config_t* config)
{
    assert(config);

    if (config->pinDirection == kGPIO_DigitalInput)
    {
        base->PDDR &= ~(1U << pin);
    }
    else
    {
        FGPIO_WritePinOutput(base, pin, config->outputLogic);
        base->PDDR |= (1U << pin);
    }
}

uint32_t FGPIO_GetPinsInterruptFlags(FGPIO_Type* base)
{
    uint8_t instance;
    instance = FGPIO_GetInstance(base);
    PORT_Type* portBase;
    portBase = s_portBases[instance];
    return portBase->ISFR;
}

void FGPIO_ClearPinsInterruptFlags(FGPIO_Type* base, uint32_t mask)
{
    uint8_t instance;
    instance = FGPIO_GetInstance(base);
    PORT_Type* portBase;
    portBase = s_portBases[instance];
    portBase->ISFR = mask;
}

#if defined(FSL_FEATURE_FGPIO_HAS_ATTRIBUTE_CHECKER) && FSL_FEATURE_FGPIO_HAS_ATTRIBUTE_CHECKER
void FGPIO_CheckAttributeBytes(FGPIO_Type* base, gpio_checker_attribute_t attribute)
{
    base->GACR = (attribute << FGPIO_GACR_ACB0_SHIFT) | (attribute << FGPIO_GACR_ACB1_SHIFT) |
                 (attribute << FGPIO_GACR_ACB2_SHIFT) | (attribute << FGPIO_GACR_ACB3_SHIFT);
}
#endif

#endif /* FSL_FEATURE_SOC_FGPIO_COUNT */
