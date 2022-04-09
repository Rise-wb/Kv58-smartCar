#include "include.h"


/*-------------------------MAIN FUNCTION------------------------------*/
/*********************************************************************//**
 * @brief		Main program body
 * @param[in]	None
 * @return 		int
 **********************************************************************/


int main (void) {                       /* Main Program                       */
 		
	  BOARD_BootClockRUN_240M();    //≈‰÷√ ±÷”‘¥
	  gpio_init(&PTA4,GPO,1);  //∆¡±ŒNMI÷–∂œ
		gpio_init(&PTA8,GPO,0);
		gpio_init(&PTA9,GPO,0);
		
		uartPut_init();  
		UART_Irq_En(UART_0);
		NVIC_SetPriorityGrouping(0x02);
		NVIC_SetPriority(UART0_RX_TX_IRQn, NVIC_EncodePriority(0x02, 2, 1));
		NVIC_SetPriority(PIT0_IRQn, NVIC_EncodePriority(0x02, 2, 2));

	
	  Par_init();
		MotorPWMInit();
		ServoPWMInit();
		UI_init();
		ADC0_Init();	
	  //flash_init();
	  lptmr_delay_ms(100);
		Beer_init();
		std_io_init();
	  FFT_arm_init();
	  //CommunicationInit();
		CounterInit();
		
	  uartPut_init();
		CtrlPeriodTimerInit();
	  //Beer_ON;	    	  	  
	  		
	while (1)
	{                           					// Loop forever					
       		UI_display();
		
//         if(PC==1&& mykey.go_flag==1) 
//         {            
             wireless_task();
//         }
					
	}
}

void CtrlTask()
{

//				ReceviceData(); 
//				MotorPWMChange1(2000);
//				MotorPWMChange2(2000);
				set_Angle();
				set_Speed();		
//				isDoubleWirelessTask();
//        anotherCar = anotCarState;
//        if(WirelessTask == 1 && singleRun == 0)
//        {
//            Beer_ON;
//            TransmitData();            
//            WirelessTask = 0;
//        }
}

void UART0_RX_TX_IRQHandler(void)
{
//		OLED_P6x8Str(0,75,"hhhh");
    Shownum_OLED_P6x8Str(25,0,uwb_X);
		UART_Query_Str(UART_0, uwb_data, 11);
	  UWB_RNG();//UART_SFIFO_RXUF_MASK
		UART_ClearStatusFlags(UART0, kUART_RxActiveEdgeFlag);// kUART_RxFifoEmptyFlag kUART_RxFifoOverflowFlag kUART_RxFifoUnderflowFlag
		//kUART_RxActiveEdgeFlag
}



