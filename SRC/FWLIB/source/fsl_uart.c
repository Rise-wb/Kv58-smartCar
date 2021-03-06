
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

#include "fsl_uart.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

UART_MemMapPtr UARTN[UART_MAX] = {UART0_BASE_PTR, UART1_BASE_PTR, UART2_BASE_PTR, UART3_BASE_PTR, UART4_BASE_PTR, UART5_BASE_PTR}; 

/* UART transfer state. */
enum _uart_tansfer_states
{
    kUART_TxIdle, /* TX idle. */
    kUART_TxBusy, /* TX busy. */
    kUART_RxIdle, /* RX idle. */
    kUART_RxBusy  /* RX busy. */
};

/* Typedef for interrupt handler. */
typedef void (*uart_isr_t)(UART_Type *base, uart_handle_t *handle);

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Get the UART instance from peripheral base address.
 *
 * @param base UART peripheral base address.
 * @return UART instance.
 */
uint32_t UART_GetInstance(UART_Type *base);

/*!
 * @brief Get the length of received data in RX ring buffer.
 *
 * @param handle UART handle pointer.
 * @return Length of received data in RX ring buffer.
 */
static size_t UART_TransferGetRxRingBufferLength(uart_handle_t *handle);

/*!
 * @brief Check whether the RX ring buffer is full.
 *
 * @param handle UART handle pointer.
 * @retval true  RX ring buffer is full.
 * @retval false RX ring buffer is not full.
 */
static bool UART_TransferIsRxRingBufferFull(uart_handle_t *handle);

/*!
 * @brief Read RX register using non-blocking method.
 *
 * This function reads data from the TX register directly, upper layer must make
 * sure the RX register is full or TX FIFO has data before calling this function.
 *
 * @param base UART peripheral base address.
 * @param data Start addresss of the buffer to store the received data.
 * @param length Size of the buffer.
 */
static void UART_ReadNonBlocking(UART_Type *base, uint8_t *data, size_t length);

/*!
 * @brief Write to TX register using non-blocking method.
 *
 * This function writes data to the TX register directly, upper layer must make
 * sure the TX register is empty or TX FIFO has empty room before calling this function.
 *
 * @note This function does not check whether all the data has been sent out to bus,
 * so before disable TX, check kUART_TransmissionCompleteFlag to ensure the TX is
 * finished.
 *
 * @param base UART peripheral base address.
 * @param data Start addresss of the data to write.
 * @param length Size of the buffer to be sent.
 */
static void UART_WriteNonBlocking(UART_Type *base, const uint8_t *data, size_t length);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Array of UART handle. */
#if (defined(UART5))
#define UART_HANDLE_ARRAY_SIZE 6
#else /* UART5 */
#if (defined(UART4))
#define UART_HANDLE_ARRAY_SIZE 5
#else /* UART4 */
#if (defined(UART3))
#define UART_HANDLE_ARRAY_SIZE 4
#else /* UART3 */
#if (defined(UART2))
#define UART_HANDLE_ARRAY_SIZE 3
#else /* UART2 */
#if (defined(UART1))
#define UART_HANDLE_ARRAY_SIZE 2
#else /* UART1 */
#if (defined(UART0))
#define UART_HANDLE_ARRAY_SIZE 1
#else /* UART0 */
#error No UART instance.
#endif /* UART 0 */
#endif /* UART 1 */
#endif /* UART 2 */
#endif /* UART 3 */
#endif /* UART 4 */
#endif /* UART 5 */
uart_handle_t *s_uartHandle[UART_HANDLE_ARRAY_SIZE];
/* Array of UART peripheral base address. */
static UART_Type *const s_uartBases[] = UART_BASE_PTRS;

/* Array of UART IRQ number. */
static const IRQn_Type s_uartIRQ[] = UART_RX_TX_IRQS;
/* Array of UART clock name. */
static const clock_ip_name_t s_uartClock[] = UART_CLOCKS;

/* UART ISR for transactional APIs. */
static uart_isr_t s_uartIsr;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 *  @brief      ?????,?????
 *  @param      UARTn_e       ???(UART0~UART5)
 *  @param      baud        ???,?9600?19200?56000?115200?
 *  @since      v5.0
 *  @note       UART?????? App\Inc\PORT_cfg.h ?????,
                printf?????????? App\Inc\MK60_conf.h ?????
 *  Sample usage:       uart_init (UART3, 9600);        //?????3,????9600
 */
void uart_init (UART_Type* uratn, uint32_t baud)
{
    register uint16_t sbr, brfa;
    uint8_t temp;
    uint32_t sysclk;     //??

    /* ?? UART??? ???? */
	
    
    if	(uratn == UART0)
        SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;      //?? UART0 ??        
    else if(uratn == UART1)
        SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;              
    else if(uratn == UART2)
        SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;       
    else if(uratn == UART3)
        SIM->SCGC4 |= SIM_SCGC4_UART3_MASK;
             

    //?????,????????
    uratn->C2 &= ~(0
                                   | UART_C2_TE_MASK
                                   | UART_C2_RE_MASK
                                  );


    //???8??????
    //?? UART ???????????????????? UART ??????? C1 ??;
    uratn->C1								 |= (0
                                  //| UART_C2_M_MASK                    //9 ?? 8 ????? : 0 ? 8? ,1 ? 9?(?????0,?8?) (???9?,?8?UARTx_C3?)
                                  //| UART_C2_PE_MASK                   //??????(???????)
                                  //| UART_C2_PT_MASK                   //????? : 0 ? ??? ,1 ? ???
                                 );

    //?????,??0?1??????,??????bus??
    if ((uratn == UART0) || (uratn == UART1))
    {
        sysclk = CLOCK_GetFreq(kCLOCK_FastPeriphClk)>>1;      //       kCLOCK_CoreSysClk                      //????
    }
    else
    {
        sysclk = CLOCK_GetFreq(kCLOCK_BusClk);                                    //bus??
    }

    //UART ??? = UART ???? / (16 ? (SBR[12:0] + BRFA))
    //??? BRFA ????, SBR = UART ???? / (16 * UART ???)
    sbr = (uint16_t)(sysclk / (baud * 16));
    if(sbr > 0x1FFF)sbr = 0x1FFF;                                       //SBR ? 13bit,??? 0x1FFF

    //?? SBR ,? BRFA =  = UART ???? / UART ??? - 16 ?SBR[12:0]
    brfa = (sysclk / baud)  - (sbr * 16);
    assert( brfa <= 0x1F);                  //??,?????????,???????????????
                                            //????????????????

    //? SBR
    temp = uratn->BDH & (~UART_BDH_SBR_MASK);           //?? ?? SBR ? UARTx_BDH??
    uratn->BDH = temp |  UART_BDH_SBR(sbr >> 8);        //???SBR??
    uratn->BDL = UART_BDL_SBR(sbr);                     //???SBR??

    //? BRFD
    temp = uratn->C4 & (~UART_C4_BRFA_MASK) ;           //?? ?? BRFA ? UARTx_C4 ??
    uratn->C4 = temp |  UART_C4_BRFA(brfa);             //??BRFA



    //??FIFO(FIFO??????????,??????)
    uratn->PFIFO |= (0
                                     | UART_PFIFO_TXFE_MASK               //??TX FIFO(??????)
                                     //| UART_PFIFO_TXFIFOSIZE(0)         //(??)TX FIFO ??,0?1??,1~6? 2^(n+1)??
                                     | UART_PFIFO_RXFE_MASK               //??RX FIFO(??????)
                                     //| UART_PFIFO_RXFIFOSIZE(0)         //(??)RX FIFO ??,0?1??,1~6? 2^(n+1)??
                                    );

    /* ??????? */
    uratn->C2 |= (0
                                  | UART_C2_TE_MASK                     //????
                                  | UART_C2_RE_MASK                     //????
                                  //| UART_C2_TIE_MASK                  //?????DMA??????(???????)
                                  //| UART_C2_TCIE_MASK                 //????????(???????)
                                  //| UART_C2_RIE_MASK                  //??????DMA??????(???????)
                                 );

    //?????????????????? UART ??? C2 ????
    //RIE ? TIE ??????????,????????????;
}



//-------------------------------------------------------------------------*
//???: uart_init                                                        
//?  ?: ???UART                                                        
//?  ?: uratn:????:UART0
//        baud: ???
//?  ?: ?                                                              
//?  ?: uart_init(UART4,115200);UART4??????UART.H??                                   
//-------------------------------------------------------------------------*

void uart_Init (UARTn_e uratn, uint32_t baud)
{
    register uint16_t sbr, brfa;
    uint8_t temp;
    uint32_t sysclk;     //??

    /* ?? UART??? ???? */
    switch(uratn)
    {
    case UART_0:
        SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;      //?? UART0 ??

        if( (UART0_RX.GPIO_bit == PTA1.GPIO_bit) && (UART0_RX.pinGPIO == PTA1.pinGPIO) )//GPIO_bit pinGPIO
        {                    
            PORT_PCR_REG(PORTA,1) = PORT_PCR_MUX(2);       //??PTA1??????,?UART0_RXD
        }
        else if( (UART0_RX.GPIO_bit == PTA15.GPIO_bit) && (UART0_RX.pinGPIO == PTA15.pinGPIO) )
        {
             PORT_PCR_REG(PORTA,15) = PORT_PCR_MUX(3);       //??PTA15???3??
        }
        else if( (UART0_RX.GPIO_bit == PTB16.GPIO_bit) && (UART0_RX.pinGPIO == PTB16.pinGPIO) )
        {
             PORT_PCR_REG(PORTB,16) = PORT_PCR_MUX(3);       //??PTB16???3??
        }
        else if( (UART0_RX.GPIO_bit == PTC6.GPIO_bit) && (UART0_RX.pinGPIO == PTC6.pinGPIO) )
        {
             PORT_PCR_REG(PORTC,6) = PORT_PCR_MUX(6);       //??PTC6???5??
        }
        else if( (UART0_RX.GPIO_bit == PTD6.GPIO_bit) && (UART0_RX.pinGPIO == PTD6.pinGPIO) )
        {
             PORT_PCR_REG(PORTD,6) = PORT_PCR_MUX(3);       //??PTD6???3??
        }
        else if( (UART0_RX.GPIO_bit == PTE21.GPIO_bit) && (UART0_RX.pinGPIO == PTE21.pinGPIO) )
        {
             PORT_PCR_REG(PORTE,21) = PORT_PCR_MUX(4);      
        }
        else if( (UART0_RX.GPIO_bit == PTB1.GPIO_bit) && (UART0_RX.pinGPIO == PTB1.pinGPIO) )
        {
             PORT_PCR_REG(PORTB,1) = PORT_PCR_MUX(7);      
        }
        else
        {
            break;
        }

        if( (UART0_RX.GPIO_bit == PTA2.GPIO_bit) && (UART0_RX.pinGPIO == PTA2.pinGPIO) )
        {
             PORT_PCR_REG(PORTA,2)= PORT_PCR_MUX(2);       //??PTA2??????
        }
        else if( (UART0_RX.GPIO_bit == PTA14.GPIO_bit) && (UART0_RX.pinGPIO == PTA14.pinGPIO) )
        {
             PORT_PCR_REG(PORTA,14)= PORT_PCR_MUX(3);       //PTA14
        }
        else if( (UART0_RX.GPIO_bit == PTB17.GPIO_bit) && (UART0_RX.pinGPIO == PTB17.pinGPIO) )
        {
             PORT_PCR_REG(PORTB,17)= PORT_PCR_MUX(3);       //PTB17
        }
        else if( (UART0_RX.GPIO_bit == PTD7.GPIO_bit) && (UART0_RX.pinGPIO == PTD7.pinGPIO) )
        {
             PORT_PCR_REG(PORTD,7)= PORT_PCR_MUX(3);       //PTD7
        }
        else if( (UART0_RX.GPIO_bit == PTC7.GPIO_bit) && (UART0_RX.pinGPIO == PTC7.pinGPIO) )
        {
             PORT_PCR_REG(PORTC,7)= PORT_PCR_MUX(5);       //PTC7
        }
        else if( (UART0_RX.GPIO_bit == PTE20.GPIO_bit) && (UART0_RX.pinGPIO == PTE20.pinGPIO) )
        {
             PORT_PCR_REG(PORTE,20)= PORT_PCR_MUX(4);       
        }
        else if( (UART0_RX.GPIO_bit == PTB0.GPIO_bit) && (UART0_RX.pinGPIO == PTB0.pinGPIO) )
        {
             PORT_PCR_REG(PORTB,0)= PORT_PCR_MUX(7);       
        }
        else
        {
             break;
        }
        break;
    default:
        break;
    }

    //?????,????????
    UART_C2_REG(UARTN[uratn]) &= ~(0
                                   | UART_C2_TE_MASK
                                   | UART_C2_RE_MASK
                                  );


    //???8??????
    //?? UART ???????????????????? UART ??????? C1 ??;
    UART_C1_REG(UARTN[uratn]) |= (0
                                  //| UART_C2_M_MASK                    //9 ?? 8 ????? : 0 ? 8? ,1 ? 9?(?????0,?8?) (???9?,?8?UARTx_C3?)
                                  //| UART_C2_PE_MASK                   //??????(???????)
                                  //| UART_C2_PT_MASK                   //????? : 0 ? ??? ,1 ? ???
                                 );

    //?????,??0-5??bus??   
    sysclk = CLOCK_GetFreq(kCLOCK_FastPeriphClk)>>1;
   

    //UART ??? = UART ???? / (16 ? (SBR[12:0] + BRFA))
    //??? BRFA ????, SBR = UART ???? / (16 * UART ???)
    sbr = (uint16_t)(sysclk / (baud * 16));
    if(sbr > 0x1FFF)sbr = 0x1FFF;                                       //SBR ? 13bit,??? 0x1FFF

    //?? SBR ,? BRFA =  = UART ???? / UART ??? - 16 ?SBR[12:0]
    brfa = (sysclk / baud)  - (sbr * 16);    

    //? SBR
    temp = UART_BDH_REG(UARTN[uratn]) & (~UART_BDH_SBR_MASK);           //?? ?? SBR ? UARTx_BDH??
    UART_BDH_REG(UARTN[uratn]) = temp |  UART_BDH_SBR(sbr >> 8);        //???SBR??
    UART_BDL_REG(UARTN[uratn]) = UART_BDL_SBR(sbr);                     //???SBR??

    //? BRFD
    temp = UART_C4_REG(UARTN[uratn]) & (~UART_C4_BRFA_MASK) ;           //?? ?? BRFA ? UARTx_C4 ??
    UART_C4_REG(UARTN[uratn]) = temp |  UART_C4_BRFA(brfa);             //??BRFA


    //??FIFO(FIFO??????????,??????)
    UART_PFIFO_REG(UARTN[uratn]) |= (0
                                     | UART_PFIFO_TXFE_MASK               //??TX FIFO(??????)
                                     //| UART_PFIFO_TXFIFOSIZE(0)         //(??)TX FIFO ??,0?1??,1~6? 2^(n+1)??
                                     | UART_PFIFO_RXFE_MASK               //??RX FIFO(??????)
                                     //| UART_PFIFO_RXFIFOSIZE(0)         //(??)RX FIFO ??,0?1??,1~6? 2^(n+1)??
                                    );

    /* ??????? */
    UART_C2_REG(UARTN[uratn]) |= (0
                                  | UART_C2_TE_MASK                     //????
                                  | UART_C2_RE_MASK                     //????
                                  //| UART_C2_TIE_MASK                  //?????DMA??????(???????)
                                  //| UART_C2_TCIE_MASK                 //????????(???????)
                                  //| UART_C2_RIE_MASK                  //??????DMA??????(???????)
                                 );
}



//-------------------------------------------------------------------------*
//???: UART_Irq_En                                                        
//?  ?: ???????                                                      
//?  ?: uratn:????:UART0      
//?  ?: ?                                                              
//?  ?: UART_Irq_En (UART4);                             
//-------------------------------------------------------------------------*
void UART_Irq_En(UARTn_e uratn)
{
    switch(uratn)
    {
      case UART_0:
          UART0->C2 |= UART_C2_RIE_MASK;    //??UART????
         // enable_irq((uratn << 1) + 45);	//??????IRQ??
          NVIC_EnableIRQ(UART0_RX_TX_IRQn);
      break;
      case UART_1:
          UART1->C2 |= UART_C2_RIE_MASK;    //??UART????         
          NVIC_EnableIRQ(UART1_RX_TX_IRQn);
      break;
      case UART_2:
          UART2->C2 |= UART_C2_RIE_MASK;    //??UART????         
          NVIC_EnableIRQ(UART2_RX_TX_IRQn);
      break;
      case UART_3:
          UART3->C2 |= UART_C2_RIE_MASK;    //??UART????         
          NVIC_EnableIRQ(UART3_RX_TX_IRQn);
      break;
      case UART_4:
          UART4->C2 |= UART_C2_RIE_MASK;    //??UART????         
          NVIC_EnableIRQ(UART4_RX_TX_IRQn);
      break;
      case UART_5:
          UART5->C2 |= UART_C2_RIE_MASK;    //??UART????         
          NVIC_EnableIRQ(UART5_RX_TX_IRQn);
      break;
      default:
      break;
    }
}


//-------------------------------------------------------------------------*
//???: UART_Irq_Dis                                                        
//?  ?: ???????                                                      
//?  ?: uratn:????:UART0      
//?  ?: ?                                                              
//?  ?: UART_Irq_Dis (UART4);                             
//-------------------------------------------------------------------------*
void UART_Irq_Dis(UARTn_e uratn)
{
    switch(uratn)
    {
      case UART_0:
          UART0->C2 &= ~UART_C2_RIE_MASK;    //??UART????
         // enable_irq((uratn << 1) + 45);	//??????IRQ??
          NVIC_DisableIRQ(UART0_RX_TX_IRQn);
      break;
      case UART_1:
          UART1->C2 &= ~UART_C2_RIE_MASK;    //??UART????         
          NVIC_DisableIRQ(UART1_RX_TX_IRQn);
      break;
      case UART_2:
          UART2->C2 &= ~UART_C2_RIE_MASK;    //??UART????         
          NVIC_DisableIRQ(UART2_RX_TX_IRQn);
      break;
      case UART_3:
          UART3->C2 &= ~UART_C2_RIE_MASK;    //??UART????         
          NVIC_DisableIRQ(UART3_RX_TX_IRQn);
      break;
      case UART_4:
          UART4->C2 &= ~UART_C2_RIE_MASK;    //??UART????         
          NVIC_DisableIRQ(UART4_RX_TX_IRQn);
      break;
      case UART_5:
          UART5->C2 &= ~UART_C2_RIE_MASK;    //??UART????         
          NVIC_DisableIRQ(UART5_RX_TX_IRQn);
      break;
      default:
      break;    
    }
}


//-------------------------------------------------------------------------*
//???: uart_putstr                                                        
//?  ?: ?????(? NULL ????)                                                       
//?  ?: uratn:????:UART0 
//        str: ??????
//?  ?: ?                                                              
//?  ?: uart_putchar (UART4, "123456789");????9???                              
//-------------------------------------------------------------------------*
void UART_Put_Str (UARTn_e uratn, const uint8_t *str)
{
    while(*str)
    {
        UART_Put_Char(uratn, *str++);
    }
}


//-------------------------------------------------------------------------*
//???: uart_putchar                                                        
//?  ?: ??????                                                       
//?  ?: uratn:????:UART0 
//         ch: ?????
//?  ?: ?                                                              
//?  ?: uart_putchar (UART4, 0x66);                               
//-------------------------------------------------------------------------*
void UART_Put_Char (UARTn_e uratn, char ch)
{
    //????????
    while(!(UART_S1_REG(UARTN[uratn]) & UART_S1_TDRE_MASK));

    //????
    UART_D_REG(UARTN[uratn]) = (uint8_t)ch;
}


/*!
 *  @brief      ????????
 *  @param      UARTn_e       ???(UART0~UART5)
 *  @param      ch          ???????
 *  @since      v5.0
 *  @note       printf???????
 *  @see        fputc
 *  Sample usage:       uart_putchar (UART3, 'A');  //????'A'
 */
void uart_putchar (UART_Type* uratn, char ch)
{
    //????????
    while(!(uratn->S1 & UART_S1_TDRE_MASK));

    //????
    uratn->D = (uint8_t)ch;
}
/*!
 *  @brief      ????len??????? (?? NULL ????)
 *  @param      UARTn_e       ???(UART0~UART5)
 *  @param      buff        ????
 *  @param      len         ???????
 *  @since      v5.0
 *  Sample usage:       uart_putbuff (UART3,"1234567", 3); //?????3???'1','2','3'
 */
void uart_putbuff (UART_Type* uratn, uint8_t *buff, uint32_t len)
{
    while(len--)
    {
        uart_putchar(uratn, *buff);
        buff++;
    }
}

uint32_t UART_GetInstance(UART_Type *base)
{
    uint32_t instance;
    uint32_t uartArrayCount = (sizeof(s_uartBases) / sizeof(s_uartBases[0]));

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < uartArrayCount; instance++)
    {
        if (s_uartBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < uartArrayCount);

    return instance;
}

static size_t UART_TransferGetRxRingBufferLength(uart_handle_t *handle)
{
    assert(handle);

    size_t size;

    if (handle->rxRingBufferTail > handle->rxRingBufferHead)
    {
        size = (size_t)(handle->rxRingBufferHead + handle->rxRingBufferSize - handle->rxRingBufferTail);
    }
    else
    {
        size = (size_t)(handle->rxRingBufferHead - handle->rxRingBufferTail);
    }

    return size;
}

static bool UART_TransferIsRxRingBufferFull(uart_handle_t *handle)
{
    assert(handle);

    bool full;

    if (UART_TransferGetRxRingBufferLength(handle) == (handle->rxRingBufferSize - 1U))
    {
        full = true;
    }
    else
    {
        full = false;
    }

    return full;
}

status_t UART_Init(UART_Type *base, const uart_config_t *config, uint32_t srcClock_Hz)
{
    assert(config);
    assert(config->baudRate_Bps);
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    assert(FSL_FEATURE_UART_FIFO_SIZEn(base) >= config->txFifoWatermark);
    assert(FSL_FEATURE_UART_FIFO_SIZEn(base) >= config->rxFifoWatermark);
#endif

    uint16_t sbr = 0;
    uint8_t temp = 0;
    uint32_t baudDiff = 0;

    /* Calculate the baud rate modulo divisor, sbr*/
    sbr = srcClock_Hz / (config->baudRate_Bps * 16);
    /* set sbrTemp to 1 if the sourceClockInHz can not satisfy the desired baud rate */
    if (sbr == 0)
    {
        sbr = 1;
    }
#if defined(FSL_FEATURE_UART_HAS_BAUD_RATE_FINE_ADJUST_SUPPORT) && FSL_FEATURE_UART_HAS_BAUD_RATE_FINE_ADJUST_SUPPORT
    /* Determine if a fractional divider is needed to fine tune closer to the
     * desired baud, each value of brfa is in 1/32 increments,
     * hence the multiply-by-32. */
    uint16_t brfa = (32 * srcClock_Hz / (config->baudRate_Bps * 16)) - 32 * sbr;

    /* Calculate the baud rate based on the temporary SBR values and BRFA */
    baudDiff = (srcClock_Hz * 2 / ((sbr * 32 + brfa))) - config->baudRate_Bps;

#else
    /* Calculate the baud rate based on the temporary SBR values */
    baudDiff = (srcClock_Hz / (sbr * 16)) - config->baudRate_Bps;

    /* Select the better value between sbr and (sbr + 1) */
    if (baudDiff > (config->baudRate_Bps - (srcClock_Hz / (16 * (sbr + 1)))))
    {
        baudDiff = config->baudRate_Bps - (srcClock_Hz / (16 * (sbr + 1)));
        sbr++;
    }
#endif

    /* next, check to see if actual baud rate is within 3% of desired baud rate
     * based on the calculate SBR value */
    if (baudDiff > ((config->baudRate_Bps / 100) * 3))
    {
        /* Unacceptable baud rate difference of more than 3%*/
        return kStatus_UART_BaudrateNotSupport;
    }

    /* Enable uart clock */
    CLOCK_EnableClock(s_uartClock[UART_GetInstance(base)]);

    /* Disable UART TX RX before setting. */
    base->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

    /* Write the sbr value to the BDH and BDL registers*/
    base->BDH = (base->BDH & ~UART_BDH_SBR_MASK) | (uint8_t)(sbr >> 8);
    base->BDL = (uint8_t)sbr;

#if defined(FSL_FEATURE_UART_HAS_BAUD_RATE_FINE_ADJUST_SUPPORT) && FSL_FEATURE_UART_HAS_BAUD_RATE_FINE_ADJUST_SUPPORT
    /* Write the brfa value to the register*/
    base->C4 = (base->C4 & ~UART_C4_BRFA_MASK) | (brfa & UART_C4_BRFA_MASK);
#endif

    /* Set bit count and parity mode. */
    temp = base->C1 & ~(UART_C1_PE_MASK | UART_C1_PT_MASK | UART_C1_M_MASK);

    if (kUART_ParityDisabled != config->parityMode)
    {
        temp |= (UART_C1_M_MASK | (uint8_t)config->parityMode);
    }

    base->C1 = temp;

#if defined(FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT) && FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT
    /* Set stop bit per char */
    base->BDH = (base->BDH & ~UART_BDH_SBNS_MASK) | UART_BDH_SBNS((uint8_t)config->stopBitCount);
#endif

#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    /* Set tx/rx FIFO watermark */
    base->TWFIFO = config->txFifoWatermark;
    base->RWFIFO = config->rxFifoWatermark;

    /* Enable tx/rx FIFO */
    base->PFIFO |= (UART_PFIFO_TXFE_MASK | UART_PFIFO_RXFE_MASK);

    /* Flush FIFO */
    base->CFIFO |= (UART_CFIFO_TXFLUSH_MASK | UART_CFIFO_RXFLUSH_MASK);
#endif

    /* Enable TX/RX base on configure structure. */
    temp = base->C2;

    if (config->enableTx)
    {
        temp |= UART_C2_TE_MASK;
    }

    if (config->enableRx)
    {
        temp |= UART_C2_RE_MASK;
    }

    base->C2 = temp;

    return kStatus_Success;
}

void UART_Deinit(UART_Type *base)
{
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    /* Wait tx FIFO send out*/
    while (0 != base->TCFIFO)
    {
    }
#endif
    /* Wait last char shoft out */
    while (0 == (base->S1 & UART_S1_TC_MASK))
    {
    }

    /* Disable the module. */
    base->C2 = 0;

    /* Disable uart clock */
    CLOCK_DisableClock(s_uartClock[UART_GetInstance(base)]);
}

void UART_GetDefaultConfig(uart_config_t *config)
{
    assert(config);

    config->baudRate_Bps = 115200U;
    config->parityMode = kUART_ParityDisabled;
#if defined(FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT) && FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT
    config->stopBitCount = kUART_OneStopBit;
#endif
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    config->txFifoWatermark = 0;
    config->rxFifoWatermark = 1;
#endif
    config->enableTx = false;
    config->enableRx = false;
}

status_t UART_SetBaudRate(UART_Type *base, uint32_t baudRate_Bps, uint32_t srcClock_Hz)
{
    assert(baudRate_Bps);

    uint16_t sbr = 0;
    uint32_t baudDiff = 0;
    uint8_t oldCtrl;

    /* Calculate the baud rate modulo divisor, sbr*/
    sbr = srcClock_Hz / (baudRate_Bps * 16);
    /* set sbrTemp to 1 if the sourceClockInHz can not satisfy the desired baud rate */
    if (sbr == 0)
    {
        sbr = 1;
    }
#if defined(FSL_FEATURE_UART_HAS_BAUD_RATE_FINE_ADJUST_SUPPORT) && FSL_FEATURE_UART_HAS_BAUD_RATE_FINE_ADJUST_SUPPORT
    /* Determine if a fractional divider is needed to fine tune closer to the
     * desired baud, each value of brfa is in 1/32 increments,
     * hence the multiply-by-32. */
    uint16_t brfa = (32 * srcClock_Hz / (baudRate_Bps * 16)) - 32 * sbr;

    /* Calculate the baud rate based on the temporary SBR values and BRFA */
    baudDiff = (srcClock_Hz * 2 / ((sbr * 32 + brfa))) - baudRate_Bps;

#else
    /* Calculate the baud rate based on the temporary SBR values */
    baudDiff = (srcClock_Hz / (sbr * 16)) - baudRate_Bps;

    /* Select the better value between sbr and (sbr + 1) */
    if (baudDiff > (baudRate_Bps - (srcClock_Hz / (16 * (sbr + 1)))))
    {
        baudDiff = baudRate_Bps - (srcClock_Hz / (16 * (sbr + 1)));
        sbr++;
    }
#endif

    /* next, check to see if actual baud rate is within 3% of desired baud rate
     * based on the calculate SBR value */
    if (baudDiff < ((baudRate_Bps / 100) * 3))
    {
        /* Store C2 before disable Tx and Rx */
        oldCtrl = base->C2;

        /* Disable UART TX RX before setting. */
        base->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

        /* Write the sbr value to the BDH and BDL registers*/
        base->BDH = (base->BDH & ~UART_BDH_SBR_MASK) | (uint8_t)(sbr >> 8);
        base->BDL = (uint8_t)sbr;

#if defined(FSL_FEATURE_UART_HAS_BAUD_RATE_FINE_ADJUST_SUPPORT) && FSL_FEATURE_UART_HAS_BAUD_RATE_FINE_ADJUST_SUPPORT
        /* Write the brfa value to the register*/
        base->C4 = (base->C4 & ~UART_C4_BRFA_MASK) | (brfa & UART_C4_BRFA_MASK);
#endif
        /* Restore C2. */
        base->C2 = oldCtrl;

        return kStatus_Success;
    }
    else
    {
        /* Unacceptable baud rate difference of more than 3%*/
        return kStatus_UART_BaudrateNotSupport;
    }
}

void UART_EnableInterrupts(UART_Type *base, uint32_t mask)
{
    mask &= kUART_AllInterruptsEnable;

    /* The interrupt mask is combined by control bits from several register: ((CFIFO<<24) | (C3<<16) | (C2<<8) |(BDH))
     */
    base->BDH |= mask;
    base->C2 |= (mask >> 8);
    base->C3 |= (mask >> 16);

#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    base->CFIFO |= (mask >> 24);
#endif
}

void UART_DisableInterrupts(UART_Type *base, uint32_t mask)
{
    mask &= kUART_AllInterruptsEnable;

    /* The interrupt mask is combined by control bits from several register: ((CFIFO<<24) | (C3<<16) | (C2<<8) |(BDH))
     */
    base->BDH &= ~mask;
    base->C2 &= ~(mask >> 8);
    base->C3 &= ~(mask >> 16);

#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    base->CFIFO &= ~(mask >> 24);
#endif
}

uint32_t UART_GetEnabledInterrupts(UART_Type *base)
{
    uint32_t temp;

    temp = base->BDH | ((uint32_t)(base->C2) << 8) | ((uint32_t)(base->C3) << 16);

#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    temp |= ((uint32_t)(base->CFIFO) << 24);
#endif

    return temp & kUART_AllInterruptsEnable;
}

uint32_t UART_GetStatusFlags(UART_Type *base)
{
    uint32_t status_flag;

    status_flag = base->S1 | ((uint32_t)(base->S2) << 8);

#if defined(FSL_FEATURE_UART_HAS_EXTENDED_DATA_REGISTER_FLAGS) && FSL_FEATURE_UART_HAS_EXTENDED_DATA_REGISTER_FLAGS
    status_flag |= ((uint32_t)(base->ED) << 16);
#endif

#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    status_flag |= ((uint32_t)(base->SFIFO) << 24);
#endif

    return status_flag;
}

status_t UART_ClearStatusFlags(UART_Type *base, uint32_t mask)
{
    uint8_t reg = base->S2;
    status_t status;

#if defined(FSL_FEATURE_UART_HAS_LIN_BREAK_DETECT) && FSL_FEATURE_UART_HAS_LIN_BREAK_DETECT
    reg &= ~(UART_S2_RXEDGIF_MASK | UART_S2_LBKDIF_MASK);//1100 0000
#else
    reg &= ~UART_S2_RXEDGIF_MASK;//0100 0000		UART_S1_OR_MASK
#endif

    base->S2 = reg | (uint8_t)(mask >> 8);//1100 0000 | xxxx xxxx

#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    base->SFIFO = (uint8_t)(mask >> 24);
#endif

    if (mask & (kUART_IdleLineFlag | kUART_NoiseErrorFlag | kUART_FramingErrorFlag |
                kUART_ParityErrorFlag))
    {
        /* Read base->D to clear the flags. */
        (void)base->S1;
        (void)base->D;
    }

    if (mask & kUART_RxOverrunFlag)
    {
        /* Read base->D to clear the flags and Flush all data in FIFO. */
        (void)base->S1;
        (void)base->D;
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
        /* Flush FIFO date, otherwise FIFO pointer will be in unknown state. */
        base->CFIFO |= UART_CFIFO_RXFLUSH_MASK;
#endif
    }

    /* If some flags still pending. */
    if (mask & UART_GetStatusFlags(base))
    {
        /* Some flags can only clear or set by the hardware itself, these flags are: kUART_TxDataRegEmptyFlag,
        kUART_TransmissionCompleteFlag, kUART_RxDataRegFullFlag, kUART_RxActiveFlag, kUART_NoiseErrorInRxDataRegFlag,
        kUART_ParityErrorInRxDataRegFlag, kUART_TxFifoEmptyFlag, kUART_RxFifoEmptyFlag. */
        status = kStatus_UART_FlagCannotClearManually;
    }
    else
    {
        status = kStatus_Success;
    }

    return status;
}

void UART_WriteBlocking(UART_Type *base, const uint8_t *data, size_t length)
{
    /* This API can only ensure that the data is written into the data buffer but can't
    ensure all data in the data buffer are sent into the transmit shift buffer. */
    while (length--)
    {
        while (!(base->S1 & UART_S1_TDRE_MASK))
        {
        }
        base->D = *(data++);
    }
}

static void UART_WriteNonBlocking(UART_Type *base, const uint8_t *data, size_t length)
{
    assert(data);

    size_t i;

    /* The Non Blocking write data API assume user have ensured there is enough space in
    peripheral to write. */
    for (i = 0; i < length; i++)
    {
        base->D = data[i];
    }
}

status_t UART_ReadBlocking(UART_Type *base, uint8_t *data, size_t length)
{
    assert(data);

    uint32_t statusFlag;

    while (length--)
    {
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
        while (!base->RCFIFO)
#else
        while (!(base->S1 & UART_S1_RDRF_MASK))
#endif
        {
            statusFlag = UART_GetStatusFlags(base);

            if (statusFlag & kUART_RxOverrunFlag)
            {
                return kStatus_UART_RxHardwareOverrun;
            }

            if (statusFlag & kUART_NoiseErrorFlag)
            {
                return kStatus_UART_NoiseError;
            }

            if (statusFlag & kUART_FramingErrorFlag)
            {
                return kStatus_UART_FramingError;
            }

            if (statusFlag & kUART_ParityErrorFlag)
            {
                return kStatus_UART_ParityError;
            }
        }
        *(data++) = base->D;
    }

    return kStatus_Success;
}

static void UART_ReadNonBlocking(UART_Type *base, uint8_t *data, size_t length)
{
    assert(data);

    size_t i;

    /* The Non Blocking read data API assume user have ensured there is enough space in
    peripheral to write. */
    for (i = 0; i < length; i++)
    {
        data[i] = base->D;
    }
}

void UART_TransferCreateHandle(UART_Type *base,
                               uart_handle_t *handle,
                               uart_transfer_callback_t callback,
                               void *userData)
{
    assert(handle);

    uint32_t instance;

    /* Zero the handle. */
    memset(handle, 0, sizeof(*handle));

    /* Set the TX/RX state. */
    handle->rxState = kUART_RxIdle;
    handle->txState = kUART_TxIdle;

    /* Set the callback and user data. */
    handle->callback = callback;
    handle->userData = userData;

#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
    /* Note:
       Take care of the RX FIFO, RX interrupt request only assert when received bytes
       equal or more than RX water mark, there is potential issue if RX water
       mark larger than 1.
       For example, if RX FIFO water mark is 2, upper layer needs 5 bytes and
       5 bytes are received. the last byte will be saved in FIFO but not trigger
       RX interrupt because the water mark is 2.
     */
    base->RWFIFO = 1U;
#endif

    /* Get instance from peripheral base address. */
    instance = UART_GetInstance(base);

    /* Save the handle in global variables to support the double weak mechanism. */
    s_uartHandle[instance] = handle;

    s_uartIsr = UART_TransferHandleIRQ;

    /* Enable interrupt in NVIC. */
    EnableIRQ(s_uartIRQ[instance]);
}

void UART_TransferStartRingBuffer(UART_Type *base, uart_handle_t *handle, uint8_t *ringBuffer, size_t ringBufferSize)
{
    assert(handle);
    assert(ringBuffer);

    /* Setup the ringbuffer address */
    handle->rxRingBuffer = ringBuffer;
    handle->rxRingBufferSize = ringBufferSize;
    handle->rxRingBufferHead = 0U;
    handle->rxRingBufferTail = 0U;

    /* Enable the interrupt to accept the data when user need the ring buffer. */
    UART_EnableInterrupts(base, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
}

void UART_TransferStopRingBuffer(UART_Type *base, uart_handle_t *handle)
{
    assert(handle);

    if (handle->rxState == kUART_RxIdle)
    {
        UART_DisableInterrupts(base, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
    }

    handle->rxRingBuffer = NULL;
    handle->rxRingBufferSize = 0U;
    handle->rxRingBufferHead = 0U;
    handle->rxRingBufferTail = 0U;
}

status_t UART_TransferSendNonBlocking(UART_Type *base, uart_handle_t *handle, uart_transfer_t *xfer)
{
    assert(handle);
    assert(xfer);
    assert(xfer->dataSize);
    assert(xfer->data);

    status_t status;

    /* Return error if current TX busy. */
    if (kUART_TxBusy == handle->txState)
    {
        status = kStatus_UART_TxBusy;
    }
    else
    {
        handle->txData = xfer->data;
        handle->txDataSize = xfer->dataSize;
        handle->txDataSizeAll = xfer->dataSize;
        handle->txState = kUART_TxBusy;

        /* Enable transmiter interrupt. */
        UART_EnableInterrupts(base, kUART_TxDataRegEmptyInterruptEnable);

        status = kStatus_Success;
    }

    return status;
}

void UART_TransferAbortSend(UART_Type *base, uart_handle_t *handle)
{
    assert(handle);

    UART_DisableInterrupts(base, kUART_TxDataRegEmptyInterruptEnable | kUART_TransmissionCompleteInterruptEnable);

    handle->txDataSize = 0;
    handle->txState = kUART_TxIdle;
}

status_t UART_TransferGetSendCount(UART_Type *base, uart_handle_t *handle, uint32_t *count)
{
    assert(handle);
    assert(count);

    if (kUART_TxIdle == handle->txState)
    {
        return kStatus_NoTransferInProgress;
    }

    *count = handle->txDataSizeAll - handle->txDataSize;

    return kStatus_Success;
}

status_t UART_TransferReceiveNonBlocking(UART_Type *base,
                                         uart_handle_t *handle,
                                         uart_transfer_t *xfer,
                                         size_t *receivedBytes)
{
    assert(handle);
    assert(xfer);
    assert(xfer->data);
    assert(xfer->dataSize);

    uint32_t i;
    status_t status;
    /* How many bytes to copy from ring buffer to user memory. */
    size_t bytesToCopy = 0U;
    /* How many bytes to receive. */
    size_t bytesToReceive;
    /* How many bytes currently have received. */
    size_t bytesCurrentReceived;
    uint32_t regPrimask = 0U;

    /* How to get data:
       1. If RX ring buffer is not enabled, then save xfer->data and xfer->dataSize
          to uart handle, enable interrupt to store received data to xfer->data. When
          all data received, trigger callback.
       2. If RX ring buffer is enabled and not empty, get data from ring buffer first.
          If there are enough data in ring buffer, copy them to xfer->data and return.
          If there are not enough data in ring buffer, copy all of them to xfer->data,
          save the xfer->data remained empty space to uart handle, receive data
          to this empty space and trigger callback when finished. */

    if (kUART_RxBusy == handle->rxState)
    {
        status = kStatus_UART_RxBusy;
    }
    else
    {
        bytesToReceive = xfer->dataSize;
        bytesCurrentReceived = 0U;

        /* If RX ring buffer is used. */
        if (handle->rxRingBuffer)
        {
            /* Disable IRQ, protect ring buffer. */
            regPrimask = DisableGlobalIRQ();

            /* How many bytes in RX ring buffer currently. */
            bytesToCopy = UART_TransferGetRxRingBufferLength(handle);

            if (bytesToCopy)
            {
                bytesToCopy = MIN(bytesToReceive, bytesToCopy);

                bytesToReceive -= bytesToCopy;

                /* Copy data from ring buffer to user memory. */
                for (i = 0U; i < bytesToCopy; i++)
                {
                    xfer->data[bytesCurrentReceived++] = handle->rxRingBuffer[handle->rxRingBufferTail];

                    /* Wrap to 0. Not use modulo (%) because it might be large and slow. */
                    if (handle->rxRingBufferTail + 1U == handle->rxRingBufferSize)
                    {
                        handle->rxRingBufferTail = 0U;
                    }
                    else
                    {
                        handle->rxRingBufferTail++;
                    }
                }
            }

            /* If ring buffer does not have enough data, still need to read more data. */
            if (bytesToReceive)
            {
                /* No data in ring buffer, save the request to UART handle. */
                handle->rxData = xfer->data + bytesCurrentReceived;
                handle->rxDataSize = bytesToReceive;
                handle->rxDataSizeAll = bytesToReceive;
                handle->rxState = kUART_RxBusy;
            }

            /* Enable IRQ if previously enabled. */
            EnableGlobalIRQ(regPrimask);

            /* Call user callback since all data are received. */
            if (0 == bytesToReceive)
            {
                if (handle->callback)
                {
                    handle->callback(base, handle, kStatus_UART_RxIdle, handle->userData);
                }
            }
        }
        /* Ring buffer not used. */
        else
        {
            handle->rxData = xfer->data + bytesCurrentReceived;
            handle->rxDataSize = bytesToReceive;
            handle->rxDataSizeAll = bytesToReceive;
            handle->rxState = kUART_RxBusy;

            /* Enable RX interrupt. */
            UART_EnableInterrupts(base, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
        }

        /* Return the how many bytes have read. */
        if (receivedBytes)
        {
            *receivedBytes = bytesCurrentReceived;
        }

        status = kStatus_Success;
    }

    return status;
}

void UART_TransferAbortReceive(UART_Type *base, uart_handle_t *handle)
{
    assert(handle);

    /* Only abort the receive to handle->rxData, the RX ring buffer is still working. */
    if (!handle->rxRingBuffer)
    {
        /* Disable RX interrupt. */
        UART_DisableInterrupts(base, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
    }

    handle->rxDataSize = 0U;
    handle->rxState = kUART_RxIdle;
}

status_t UART_TransferGetReceiveCount(UART_Type *base, uart_handle_t *handle, uint32_t *count)
{
    assert(handle);
    assert(count);

    if (kUART_RxIdle == handle->rxState)
    {
        return kStatus_NoTransferInProgress;
    }

    if (!count)
    {
        return kStatus_InvalidArgument;
    }

    *count = handle->rxDataSizeAll - handle->rxDataSize;

    return kStatus_Success;
}

void UART_TransferHandleIRQ(UART_Type *base, uart_handle_t *handle)
{
    assert(handle);

    uint8_t count;
    uint8_t tempCount;

    /* If RX overrun. */
    if (UART_S1_OR_MASK & base->S1)
    {
        /* Read base->D to clear overrun flag, otherwise the RX does not work. */
        (void)base->D;
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
        /* Flush FIFO date, otherwise FIFO pointer will be in unknown state. */
        base->CFIFO |= UART_CFIFO_RXFLUSH_MASK;
#endif

        /* Trigger callback. */
        if (handle->callback)
        {
            handle->callback(base, handle, kStatus_UART_RxHardwareOverrun, handle->userData);
        }
    }

    /* Receive data register full */
    if ((UART_S1_RDRF_MASK & base->S1) && (UART_C2_RIE_MASK & base->C2))
    {
/* Get the size that can be stored into buffer for this interrupt. */
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
        count = base->RCFIFO;
#else
        count = 1;
#endif

        /* If handle->rxDataSize is not 0, first save data to handle->rxData. */
        while ((count) && (handle->rxDataSize))
        {
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
            tempCount = MIN(handle->rxDataSize, count);
#else
            tempCount = 1;
#endif

            /* Using non block API to read the data from the registers. */
            UART_ReadNonBlocking(base, handle->rxData, tempCount);
            handle->rxData += tempCount;
            handle->rxDataSize -= tempCount;
            count -= tempCount;

            /* If all the data required for upper layer is ready, trigger callback. */
            if (!handle->rxDataSize)
            {
                handle->rxState = kUART_RxIdle;

                if (handle->callback)
                {
                    handle->callback(base, handle, kStatus_UART_RxIdle, handle->userData);
                }
            }
        }

        /* If use RX ring buffer, receive data to ring buffer. */
        if (handle->rxRingBuffer)
        {
            while (count--)
            {
                /* If RX ring buffer is full, trigger callback to notify over run. */
                if (UART_TransferIsRxRingBufferFull(handle))
                {
                    if (handle->callback)
                    {
                        handle->callback(base, handle, kStatus_UART_RxRingBufferOverrun, handle->userData);
                    }
                }

                /* If ring buffer is still full after callback function, the oldest data is overrided. */
                if (UART_TransferIsRxRingBufferFull(handle))
                {
                    /* Increase handle->rxRingBufferTail to make room for new data. */
                    if (handle->rxRingBufferTail + 1U == handle->rxRingBufferSize)
                    {
                        handle->rxRingBufferTail = 0U;
                    }
                    else
                    {
                        handle->rxRingBufferTail++;
                    }
                }

                /* Read data. */
                handle->rxRingBuffer[handle->rxRingBufferHead] = base->D;

                /* Increase handle->rxRingBufferHead. */
                if (handle->rxRingBufferHead + 1U == handle->rxRingBufferSize)
                {
                    handle->rxRingBufferHead = 0U;
                }
                else
                {
                    handle->rxRingBufferHead++;
                }
            }
        }
        /* If no receive requst pending, stop RX interrupt. */
        else if (!handle->rxDataSize)
        {
            UART_DisableInterrupts(base, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
        }
        else
        {
        }
    }

    /* Send data register empty and the interrupt is enabled. */
    if ((base->S1 & UART_S1_TDRE_MASK) && (base->C2 & UART_C2_TIE_MASK))
    {
/* Get the bytes that available at this moment. */
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
        count = FSL_FEATURE_UART_FIFO_SIZEn(base) - base->TCFIFO;
#else
        count = 1;
#endif

        while ((count) && (handle->txDataSize))
        {
#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
            tempCount = MIN(handle->txDataSize, count);
#else
            tempCount = 1;
#endif

            /* Using non block API to write the data to the registers. */
            UART_WriteNonBlocking(base, handle->txData, tempCount);
            handle->txData += tempCount;
            handle->txDataSize -= tempCount;
            count -= tempCount;

            /* If all the data are written to data register, TX finished. */
            if (!handle->txDataSize)
            {
                handle->txState = kUART_TxIdle;

                /* Disable TX register empty interrupt. */
                base->C2 = (base->C2 & ~UART_C2_TIE_MASK);

                /* Trigger callback. */
                if (handle->callback)
                {
                    handle->callback(base, handle, kStatus_UART_TxIdle, handle->userData);
                }
            }
        }
    }
}

void UART_TransferHandleErrorIRQ(UART_Type *base, uart_handle_t *handle)
{
    /* To be implemented by User. */
}

#if defined(UART0)
#if ((!(defined(FSL_FEATURE_SOC_LPSCI_COUNT))) || \
     ((defined(FSL_FEATURE_SOC_LPSCI_COUNT)) && (FSL_FEATURE_SOC_LPSCI_COUNT == 0)))
void UART0_DriverIRQHandler(void)
{
    s_uartIsr(UART0, s_uartHandle[0]);
}

void UART0_RX_TX_DriverIRQHandler(void)
{
    UART0_DriverIRQHandler();
}
#endif
#endif

#if defined(UART1)
void UART1_DriverIRQHandler(void)
{
    s_uartIsr(UART1, s_uartHandle[1]);
}

void UART1_RX_TX_DriverIRQHandler(void)
{
    UART1_DriverIRQHandler();
}
#endif

#if defined(UART2)
void UART2_DriverIRQHandler(void)
{
    s_uartIsr(UART2, s_uartHandle[2]);
}

void UART2_RX_TX_DriverIRQHandler(void)
{
    UART2_DriverIRQHandler();
}

#endif

#if defined(UART3)
void UART3_DriverIRQHandler(void)
{
    s_uartIsr(UART3, s_uartHandle[3]);
}

void UART3_RX_TX_DriverIRQHandler(void)
{
    UART3_DriverIRQHandler();
}
#endif

#if defined(UART4)
void UART4_DriverIRQHandler(void)
{
    s_uartIsr(UART4, s_uartHandle[4]);
}

void UART4_RX_TX_DriverIRQHandler(void)
{
    UART4_DriverIRQHandler();
}
#endif

#if defined(UART5)
void UART5_DriverIRQHandler(void)
{
    s_uartIsr(UART5, s_uartHandle[5]);
}

void UART5_RX_TX_DriverIRQHandler(void)
{
    UART5_DriverIRQHandler();
}
#endif



//-------------------------------------------------------------------------*
//???: uart_querychar                                                        
//?  ?: ???????                                                       
//?  ?: uratn:????:UART0 
//         str: ????????
//     max_len: ??????
//?  ?: i??????(???0~max_len)                                                              
//?  ?: uart_querystr (UART4,a[],100) ??????????a???
//-------------------------------------------------------------------------*
char UART_Query_Str (UARTn_e uratn, char *str, uint32_t max_len)
{
    uint32_t i = 0;
    while(UART_Query_Char(uratn, str + i)  )
    {
        if( *(str + i) == NULL )    //?????????
        {
            return i;
        }

        i++;
        if(i >= max_len)            //????????,??
        {
            return i;
        }
    };

    return i;
}


//-------------------------------------------------------------------------*
//???: uart_querychar                                                        
//?  ?: ????1???                                                       
//?  ?: uratn:????:UART0 
//          ch: ????????
//?  ?: 1???? 0????                                                              
//?  ?: uart_querychar (UART4,a[]) ??????????a???                                
//-------------------------------------------------------------------------*
char UART_Query_Char (UARTn_e uratn, char *ch)
{
    if( UART_RCFIFO_REG(UARTN[uratn]) )         //?????????
    {
        *ch  =   UART_D_REG(UARTN[uratn]);      //???8????
        return  1;                              //?? 1 ??????
    }

    *ch = 0;                                    //????,????????
    return 0;                                   //??0??????
}


//-------------------------------------------------------------------------*
//???: uart_getchar                                                        
//?  ?: ??????                                                        
//?  ?: uratn:????:UART0 
//        ch: ????????
//?  ?: ?                                                              
//?  ?: uart_getchar (UART4,a[]) ??????????a???                                
//-------------------------------------------------------------------------*
char UART_Get_Char (UARTn_e uratn,char *ch)
{
		#define UART_S1_REG(base)	((base)->S1)
		#define UART_D_REG(base)	((base)->D)
    while (!(UART_S1_REG(UARTN[uratn]) & UART_S1_RDRF_MASK));       //??????

    *ch =  UART_D_REG(UARTN[uratn]);
    return(UART_D_REG(UARTN[uratn]));

}
