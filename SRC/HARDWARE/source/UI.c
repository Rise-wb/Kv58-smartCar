/*iyaxian has compiled UI on 2015-07-06 */

#include "UI.h"

#define N_adjust   20
#define N_display  31
  
 int P_adjust;
 
 int P_display;

struct keystate mykey={1,1,0,0,0,1,0};

void delay_ms (unsigned long zms) 
{
 	unsigned long i,j;
	for(j=zms;j>0;j--)
		for(i=20000;i>0;i--);
}
/**************************************************
*INstrument:  enable the keys,         
*        up down left right  go   mode     
*        
*
*
**************************************************/
void UI_init()
{
      //enable keys
      gpio_init  (uppper, GPI, 0);
	    port_init_NoALT(uppper,PULLUP);
	
      gpio_init  (down, GPI, 0);
	    port_init_NoALT(down,PULLUP);
	
      gpio_init  (left, GPI, 0);
			port_init_NoALT(left,PULLUP);
	
      gpio_init  (right, GPI, 0);
			port_init_NoALT(right,PULLUP);
	
      gpio_init  (page, GPI, 0);
			port_init_NoALT(page,PULLUP);
	
      gpio_init  (mode, GPI, 0);
	    port_init_NoALT(mode,PULLUP);
	
      gpio_init  (go, GPI, 0);
			port_init_NoALT(go,PULLUP);
      //OLED
      OLED_Init();
      Draw_BMP(0,0,127,7,HDUlogo);
			delayToGet;
			delayToGet;
			delayToGet;
			delayToGet;
			delayToGet;
			delayToGet;
			delayToGet;
      OLED_CLS();
      //caculelate size of array
     
     if(N_adjust%7==0) P_adjust=N_adjust/7 -1;
     else P_adjust=N_adjust/7 ;
     
     
     
     if(N_display%7==0) P_display=N_display/7 -1;
     else P_display=N_display/7 ;
     
}
/**************************************************
*INstrument: scan keys        
*           
*        keys has been pullup.
*
*
**************************************************/
void Key_scan(struct keystate* pp)
{
    pp->addstate=0;
    if(pp->mode_flag)
    {
      if(gpio_get(uppper)==0)  
      {
       delayToGet;
       if(gpio_get(uppper)==0)
        pp->Ypos_a--;
      }
                 
      if(gpio_get(down)==0)
      {
        delayToGet;
        if(gpio_get(down)==0)
          pp->Ypos_a++;        
      }
      if(pp->Ypos_a>=8) 
      {
          pp->page_a++;
          pp->Ypos_a=1;          
      }
      if(pp->Ypos_a<=0) 
      {
          pp->page_a--;
          pp->Ypos_a=7;
      }
      if(gpio_get(page)==0)
      {
        delayToGet;
        if(gpio_get(page)==0)
        { 
          pp->page_a++;
          pp->Ypos_a=1;
        }
      } 
      if(pp->page_a>P_adjust)
          pp->page_a=0;
      if(pp->page_a<0)
          pp->page_a=P_adjust;
      if(pp->page_a*7+pp->Ypos_a>N_adjust)
      {
        pp->Ypos_a= 1;
        pp->page_a= 0;
      }
          
    }
    else
    {
        if(gpio_get(uppper)==0) 
      {
       delayToGet;
       if(gpio_get(uppper)==0)
        pp->Ypos_d--;
      }
                 
      if(gpio_get(down)==0)
      {
        delayToGet;
        if(gpio_get(down)==0)
          pp->Ypos_d++;
      }
      if(pp->Ypos_d>=8) 
      {
          pp->page_d++;
          pp->Ypos_d=1;          
      }
      if(pp->Ypos_d<=0) 
      {
          pp->page_d--;
          pp->Ypos_d=7;
      }
      if(gpio_get(page)==0)
      {
        delayToGet;
        if(gpio_get(page)==0)
        {
          pp->page_d++;
          pp->Ypos_d=1;  
        }
      }
      if(pp->page_d>P_display)
          pp->page_d=0; 
      if(pp->page_d<0)
          pp->page_d=P_display;
      if(pp->page_d*7+pp->Ypos_d>N_display)
      {
        pp->Ypos_d=1;
        pp->page_d=0;
      }
    }

    if(gpio_get(left)==0)
    {
        delayToGet;
        if(gpio_get(left)==0)
        pp->addstate=-9;
    }
    if(gpio_get(right)==0)
    {
        delayToGet;
        if(gpio_get(right)==0)
        pp->addstate=10;
    } 

    if(gpio_get(mode)==0)
    {
         delayToGet;
         if(gpio_get(mode)==0)
         {
           pp->mode_flag=!pp->mode_flag;           
           OLED_CLS();
         }
         
    } 

    if(gpio_get(go)==0)
    {
         delayToGet;
         if(gpio_get(go)==0)
         pp->go_flag=1;
    }        
}
/**************************************************
*INstrument: show the name and num         
*                 
*        
*
*
**************************************************/
void Shownum_OLED_P6x8Str(uint8_t x,uint8_t y,int str)//x=36 or 95
{
      char cc[5];
      OLED_P6x8Str(x,y,"     ");
      sprintf(cc,"%d",str);
      OLED_P6x8Str(x,y,cc);
}
void Showname_OLED_P6x8Str(uint8_t x,int page_now,int N,char** ch,int** num)//x=12 或者65
{
    int m;
    m=page_now*7;
    for(int i=1;i<8;i++)
    {
        if((m+i)>N) 
        {
          OLED_P6x8Str(x,i,"        "); 
          continue;
        }
        OLED_P6x8Str(x,i,"   ");        
        OLED_P6x8Str(x,i,*(ch+m+i-1));
        OLED_P6x8Str(x+18,i,":");
        Shownum_OLED_P6x8Str(x+24,i,**(num+m+i-1));
    }
}
/**************************************************
*INstrument:  cursor set on position         
*       
*        
*
*
**************************************************/
void cursor_set(uint8_t x,uint8_t y)
{
                
               for(uint8_t i=0;i<7;i++)
               {         
                 if(y==(i+1)) continue;
                 OLED_P6x8Str(x,i+1," ");
               }
               OLED_P6x8Str(x,y,">");
}
void value_set(int *pp)
{
      (*pp)+=mykey.addstate;
}
/**************************************************
*INstrument:UI display         
*                 
*        
*
*
**************************************************/
void UI_display()
{
    static int *Array_to_adjust[N_adjust]={
                           
                           &time_s,
													 &sum_flag,
													 &huan_flag,
													 &huan_flag2,
													 &huan_time,
													 &huan_time1,
                           &huan_plus,                           
                           &servomid,
													 &speed_mid,
													 &speed_mid_turn,
													 &speed_mid_huan,
													 &N,
													 //&fan_k,
													 //&servo_k,
                           //&speed_K,//?
                           //&speed_P,//?
                           //&speed_P1,//?                         
                           //&basicspeed,
                           //&curvespeed,
                           &servo.Proportion,
                           &servo.Derivative, 
													 &servo_t.Proportion,
													 &servo_t.Derivative,
                           &Speed_P,
                           &Speed_I,
                           &Speed_D,
													 &bug_flag
                           //&C_Zhi,
                           //&D_Zhi
                           };
     static char *name_adjust[N_adjust]={"run","s_f","cf0","cf2","ct0","ct1","cip","mds","mdm","mmt","mmh"," N ","S_P","S_D","STP","STD","M_P","M_I","M_D","b_f"
                                      };  //can also store name by yourself

     static int *Array_to_display[N_display]={&AD_result[0],
                                              &AD_result[1],
                                              &AD_result[2],
                                              &AD_result[3],
																							&AD_result[5],
                                              &AD_result[7],
																							&huan_flag1,
																							&speed_L,
																							&speed_R,
																							&Motor_Lshow,
																							&Motor_Rshow,
			                                        &pwm_dif,
																							&PID_Answer,
																							&ORG_servo,
																							&PWM_servo,
//																					    &AD_result[4],
//			                                        &AD_result[5],
//                                              &AD_result[6],
//                                              &AD_result[7],
																							&P_L,
																							&P_R,
                                              &speed,
                                              &E,                                                                                          
                                              &EW,                                              
                                              &dif_W,
                                              &sum_W,
                                              &sum_all,
                                              &vol,
                                              &GyX.Finaldata_int,
                                              &GyY.Finaldata_int,
                                              &GyZ.Finaldata_int,
                                              &ACX.Finaldata_int,
                                              &ACY.Finaldata_int,
                                              &ACZ.Finaldata_int,
                                              &Ang_Y
                                              };
     static char *name_display[N_display]={"AD1","AD2","AD3","AD4","AD5","AD6","H_f","S_L","S_R","M_L","M_R",/*"AD5","AD6","AD7","AD8",*/"p_d","aws","org","p_s","spe","P_L","P_R","  E","EW","diW",
                                            "suW","sua","vol","GyX","GyY","GyZ","ACX","ACY","ACZ","AnY"};
     
     
     
     
     
          Key_scan(&mykey);
   
          //OLED_P6x8Str(0,0,"Come on.");
          
          if(mykey.mode_flag == 1)
          {
              cursor_set(0,(uint8_t)mykey.Ypos_a);
              value_set(*(Array_to_adjust+mykey.page_a*7+mykey.Ypos_a-1));

           }
          else if(mykey.mode_flag == 0)
          {
               cursor_set(65,(uint8_t)mykey.Ypos_d);
           } 

          Showname_OLED_P6x8Str(12,mykey.page_a,N_adjust,name_adjust,Array_to_adjust);
          Showname_OLED_P6x8Str(77,mykey.page_d,N_display,name_display,Array_to_display);
    
    

}
