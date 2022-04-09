#include "UWB.h"
#include "fsl_uart.h"

//UWB²â¾àÄ£¿é
void UWB_RNG(void)
{
	
				for(int i=0; i<10; i++)
				{
					date=uwb_data[i];

					if(date==121 && uwb_flag==0)	//y
					{
//						uart_finish_flag=0;
						uwb_flag=1;
						uwb_x=0;
					}
					else	if(date==122)	//z
					{
						//Shownum_OLED_P6x8Str(12,0,uwb_x);
						uwb_X=uwb_x;
						uwb_flag=0;
//						uart_finish_flag=1;
					}
					else	if( ((date>=48&&date<58) || (date>=97&&date<=102)) && uwb_flag==1 )
					{
						if(date>=48&&date<58)
						{
							uwb_x=uwb_x*16+date-48;
						}
						else  if(date>=97&&date<=102)
						{
							uwb_x=uwb_x*16+(date-'a')+10;
						}
					}
					
				}
			
}
