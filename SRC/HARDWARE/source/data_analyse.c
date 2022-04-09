
#include "data_analyse.h"
#include "math.h"

//-------------------------归一化———————————
void normal(int *swm)
{
//    static uint8_t    K[4]={255,255,255,255};
//    static char   mflag[4]={0} , isRdacOk=0;

    static int write_flag=0,read_flag=0; 
    
#define precision  1000
    //以下调节数控电位器
    /******************************
    if(*swm==1 && (mflag[0]+mflag[1]+mflag[2]+mflag[3] < 4))
    {
      
        for(i=0;i<4;i++)
        {
       	  if(mflag[i]==0)
          {
            
            if( (ADC_Mid(AD_chanel[i], ADC_10bit)*4/5 > 405)  &&  (ADC_Mid(AD_chanel[i], ADC_10bit)*4/5 <450)) mflag[i]=1;
            if(mflag[i]==0)                                          
            { 
              AD5254_WriteRdac(RdacID[i],RdacChn[i],(uint8_t)(K[i]--));
            }
          }
        }
          
        if(mflag[0]+mflag[1]+mflag[2]+mflag[3] ==4)  
        {
         for(i=0;i<4;i++)
           AD5254_WriteCmd(RdacID[i],RdacChn[i],store);
         *swm=0;
         isRdacOk=1;
        }
    }
    ****************************/
     //以下线性归一化
    if(write_flag == 0 && *swm==1)
    { 
       
            for(uint8_t i = 0;i <8;i++)
            {
                f_k[i] = 405 * precision / AD_result[i];
            }
            //ADC_Write(); //写入flash
            write_flag = 1;
            *swm=0;
       
    }
    
    if(0 == read_flag)
       {         
           //ADC_Read(); //读取flash
           read_flag = 1;
       }
    if(*swm > -1)
    {
        for(uint8_t k = 0;k <8;k++)
        {
            AD_result[k] = AD_result[k]*f_k[k]/precision;
        }
    }
         
}

void guihua(int* swm){
		int write_flag=0;
		if(write_flag==0&&*swm==1){
				for(uint8_t i = 0;i <8;i++)
        {
            f_k[i] = 500 * precision / AD_result[i];
        }
				for(uint8_t k = 0;k <8;k++)
        {
            AD_result[k] = AD_result[k]*f_k[k]/precision;
        }
		}
}

//----------------------------滤波--------------------
void filter()                                                
{
#define n_loop   8
  uint32_t AD_get[n_loop] = {0};
   static int ADf[n_loop][10]={0};
  /***************采集滤波******************/
   ADC_get(AD_get);
  
   for(uint8_t j=0;j<n_loop;j++) 
   {
   	  for(uint8_t u=0;u<9;u++)
       ADf[j][u]=ADf[j][u+1];
   }
   for(uint8_t i=0;i<n_loop;i++)
   {
   	   ADf[i][9]=AD_get[i];
   }
  for(uint8_t v=0;v<n_loop;v++)
  {
  	//AD_result[v]=(ADf[v][0]+ADf[v][1]+ADf[v][2]+ADf[v][3]+ADf[v][4]+ADf[v][5]+ADf[v][6]+ADf[v][7]+ADf[v][8]+ADf[v][9])/10;
        AD_result[v]=(ADf[v][5]+ADf[v][6]+ADf[v][7]+ADf[v][8]+ADf[v][9])/5;
  }
  
  /*************限幅*************/
  
//        for (uint8_t k=0;k<n_loop;k++)
//        {
//          
//           if (AD_result[k]<1) AD_result[k]=1; 
//        }
}
void GetPAD_result()
{
    static int deltaAD[ADN];
    static int lastAD[ADN];
    for(uint8_t i=0;i<ADN;i++)
    {
        *(deltaAD+i) = *(AD_result+i) - *(lastAD+i);
        *(lastAD+i) = *(AD_result+i);
        *(pAD_result+i) = *(AD_result+i) +  (*(deltaAD+i))*5;
    }
}
//-----------获取角度---------
void GetSiTa()
{
     //陀螺仪已经初始化  灵敏度 sensitivity  2000dps
     //  250dps    0.00875
     //  500dps    0.0175
     //  2000dps   0.07
    //static float Sensitivity =  0.07;   
    float fDeltaValue;
    
    fDeltaValue = (ACZ.Finaldata - Ang_Y) / 4.0;
    Ang_Y += (int)(GyY.Finaldata + fDeltaValue) /10 ;   //陀螺仪积分
}
//-----------------------信号处理--------------------
//---------------return the servo duty---------------
//--------the function has called the filer()----------
//#pragma optimize=none

void ramp_process()
{
		if((sum_W-sum_L)<=-300&&ramp_flag==0)
		{
				ramp_flag=1;
				ramp_flag1=1;
		}
		if(ramp_flag==1)
		{
				ramp_count++;
				Beer_ON;
				if(ramp_count>=ramp_time)
				{
						ramp_flag1=0;
						ramp_flag =0;
						ramp_count=0;
						Straight_Count=0;
						Beer_OFF;
				}
		}
}

int signal2()
{   
  static int turn_flag=0;
	static int max=0;
	static int sum=0;
	static int dif_r_L=0;
	static int errr=0;
  static int last_W=0,dif_WC=0,last_L=0,dif_LC=0;
  static int D2=0,ec=0;
  static float  p_cha=0,P2=0;
  static uint16_t podaocount=0;
  static uint16_t podaocount2=0,podaoend=0;
//  static uint8_t  circleTrceFlag = 0;
  static uint8_t  circleTrceEnd  = 0;
  static uint8_t  circleCount = 0;
  static uint16_t  circleCount2 = 0;
  static int EWsetpoint = 0,EWset = 0;
  static int Psetpoint = 0;
  static int Dsetpoint = 0;
  static int anotDirection = 0;
	static int T=0;
	static int EW_dif_W=0;
	static int huan_empty_t=0;
	static int ramp_t=0;
	
//    vol=VOL_get();               //电池电压采集

//  L3G4200_XYZ();               //陀螺仪加速度计
//  MMA8451_XYZ(); 
//
//  
//  kalman_fliter(&GyX,Gyro_X);
//  kalman_fliter(&GyY,Gyro_Y);
//  kalman_fliter(&GyZ,Gyro_Z);
//  
//  kalman_fliter(&ACX,ACC_X);
//  kalman_fliter(&ACY,ACC_Y);
//  kalman_fliter(&ACZ,ACC_Z);
  
//  speed = -(int)CounterGet()*2; 
  
//  kalman_fliter(&Spe,-speed);
//  
  //计算实际加速度
//  ace = speed - last_speed;
//  last_speed = speed;
  
//  GetSiTa();
   
  filter();
  
  //normal(&swm);
    

    //大坡道判断
   
    //判断环形弯道
  
   // 判断圆环
   
	 //圆环选择方向
	 
  //坡道判断结束
	

  dif_W = (AD_result[0] - AD_result[3]);
  sum_W = (AD_result[0] + AD_result[3]);
  dif_L = (AD_result[1] - AD_result[2]);
  sum_L = (AD_result[1] + AD_result[2]);
  dif_huan= (AD_result[5] - AD_result[7]);
	sum_huan= (AD_result[5] + AD_result[7]);
  sum_all = sum_W + sum_L;
  sum = sum_L;
	
	L_sum=AD_result[0] + AD_result[1];
	R_sum=AD_result[2] + AD_result[3];
	if(L_sum>=R_sum){
			Double_min=R_sum;
	}
	else{
			Double_min=L_sum;
	}
	if(AD_result[0]>=AD_result[3]){
			Single_max=AD_result[0];
	}
	else{
			Single_max=AD_result[3];
	}
	
	if(AD_result[1]>=AD_result[2]){
			Single_min=AD_result[2];
//			Single_max_L=AD_result[1];
	}
	else{
			Single_min=AD_result[1];
//			Single_max_L=AD_result[2];
	}
	
	

	
	ramp_process();
	
	if(sum_huan>=1000&&sum_all<1800&&dif_W<=380)
	{
			cross_flag=1;
	}
	else
	{
			cross_flag=0;
	}

  if(sum_L<500){
			turn_flag=1;
	}
	else 
	{
      if(dif_W+dif_L/4<0)  
          direction=-1;
      else if(dif_W+dif_L/4>0) 
          direction=1;
//			if(dif_W<=0){
//					direction=-1;
//					//errr=AD_result[0]-AD_result[2];
//			}
//			else{
//					direction=1;
//					//errr=AD_result[3]-AD_result[1];
//			}	
//			turn_flag=0;
	}
  if(dif_L<0){
			dif_r_L=-dif_L;
	}  
    
	if(sum_all>=2400&&abs(dif_huan)>100&&sum_L>800&&huan_empty_t==0)
	{
			if(dif_huan>0){
					huan_flag1=1;
					huan_flag3=1;
					direction=1;
			}
			else{
					huan_flag1=-1;
					huan_flag3=1;
					direction=-1;
			}
	}
	else{
			huan_flag1=0;
	}
	if(huan_flag1==0&&huan_flag3==1)
	{
			huan_empty_t++;
	}
	
	if(huan_empty_t!=0)
	{
			huan_empty_t++;
			if(huan_empty_t>=huan_empty_time)
			{
					huan_empty_t=0;
					huan_flag3=0;
			}
	}
  //方向判断结束
//	//双车模式
//  if(singleRun < 1)
//  {
//      //判断两车相对位置
//      
//      //两车判断发车
//      
//      //判断开始超车  前车
//      
//      //摩擦超车
//      
//      
//      //后车
//      
//      
//  }
					if(Confluence_flag==1)
					{
							EW_dif_W=300;
					}
					if(Confluence_flag==2)
					{
							EW_dif_W=0;
					}
					
					EH=dif_huan;
					EHC=EH-last_EH;
					last_EH=EH;
				
					if(sum>sum_flag) sum = sum_flag;
					E = sum_flag - sum ;//+ abs(cEWsetL);   //- abs(EWsetpoint)
					EC = E - last_E;
					last_E = E;
					ec = EC;
						
						  EW_dif_W = dif_W;  //
							if(direction == -1) EW =  650*(dif_W)/(sum_W+1) ;//+1    + EWsetpoint     dif_W - cEWset
							else if(direction == 1) EW =  650*(dif_W)/(sum_W+1);
							dif_WC = (EW - last_W);
							last_W = EW;
				
				  P = Fuzzy_control(E,EC,1,0)*4 + Psetpoint;  //??SD5?? *4
					D = Fuzzy_control(E,EC,0,0)*4 + Dsetpoint;
					D2 = Se->Derivative * D /1200;
  
					P2 = 1.0*Se->Proportion*10;

					
					
					if(direction == -1){
							if(huan_flag1==-1)
							{
									EW_dif_W=dif_huan;
									ec=EHC;
									P = Fuzzy_control(2*abs(EH),abs(EHC),1,0)*4 + Psetpoint;  //??SD5?? *4
									D = Fuzzy_control(2*abs(EH),abs(EHC),0,0)*4 + Dsetpoint;
									D2 = Se->Derivative * D /1200;
									ORG_servo=(P * (-(EW_dif_W+EWset)) / (P2)  + ec*D2/100)*1.35;
							}
							else{
//									if(dif_WC*dif_WC > ec*ec ) 
//									ec = dif_WC;
									if((-dif_W <= E - N)&&Single_max>Single_min){
											ORG_servo=(P * (-(-E+EWset)) / (P2)  + ec*D2/100);
									}
									else{
											ORG_servo=(P * (-(EW_dif_W+EWset)) / (P2)  + ec*D2/100);
									}
									
							}
					}
					else if(direction == 1){
							if(huan_flag1==1)
							{
									EW_dif_W=dif_huan;
									ec=EHC;
									ec=-ec;
									P = Fuzzy_control(2*abs(EH),abs(EHC),1,0)*4 + Psetpoint;  //??SD5?? *4
									D = Fuzzy_control(2*abs(EH),abs(EHC),0,0)*4 + Dsetpoint;
									D2 = Se->Derivative * D /1200;
									ORG_servo=(P * (-(EW_dif_W+EWset)) / (P2)  + ec*D2/100)*1.35;
							}
							else{
//									if(dif_WC*dif_WC > ec*ec ) 
//									ec = dif_WC;
									ec=-ec;
									if((dif_W <= E - N)&&Single_max>Single_min){
											ORG_servo=(P * (-(E+EWset)) / (P2)  + ec*D2/100);
									}
									else{
											ORG_servo=(P * (-(EW_dif_W+EWset)) / (P2)  + ec*D2/100);
									}
							}
					}
					
					
					if(ramp_flag==1)
					{
							
							if(ORG_servo>=5){
									ORG_servo=5;
							}
							if(ORG_servo<=-5){
									ORG_servo=-5;
							}
					}
					
					if(cross_flag==1)
					{
							if(ORG_servo>=5){
									ORG_servo=5;
							}
							if(ORG_servo<=-5){
									ORG_servo=-5;
							}
					}

					ORG_chaspeed=ORG_servo;
					
//	if(sum_L<400){
//			if(direction==1)
//					ORG_servo=420;
//			else if(direction==-1)
//					ORG_servo=400;
//	}
	if(ORG_servo<=0){
			ORG_servo=-ORG_servo;
	}


	if(direction==1||direction==2){
			ORG_servo=ORG_servo;//*1.05
	}
	else if(direction==-1||direction==-2){
			ORG_servo=ORG_servo;//*1.04
	}

	if(bug_flag<0){
	    ORG_servo=0;
	}
	ORG_chaspeed=ORG_servo;
	
	
  if (direction==-1||direction==-2) //右转 
  {        
		  Speed_Lmodu = (1.0+1.0*ORG_servo/Steer_L_Range/5.0);
      Speed_Rmodu = (1.0-1.0*ORG_servo/Steer_L_Range/5.0);
			Servoduty=servomid-ORG_servo;//*0.93;//*1.25;//*1.17*0.9;
  }
  else if(direction==1||direction==2)
  { 
			Speed_Lmodu = (1.0-1.0*ORG_servo/Steer_L_Range/5.0);
      Speed_Rmodu = (1.0+1.0*ORG_servo/Steer_L_Range/5.0);
      Servoduty=servomid+ORG_servo;//*1.10;//*0.9 ;     
  }
  

  //打角计算结束
  //限幅
  if (Servoduty>SERVOMAX)
    Servoduty = SERVOMAX;
  else if (Servoduty<SERVOMIN)
    Servoduty = SERVOMIN;
  
  
  return Servoduty;
  
}

//-------------------caculate setting speed-------------------------
//#pragma optimize=none
void caculate_speed2()   //返回值为speedflag
{
		
		if((Single_min>sum_max&&direction==-1)||(Single_min>sum_max1&&direction==1))//要改sum_L>=700||
    {        
        //OFF;
        Straight_Count++;                 // ?????? now_S = AD_SL[0]-AD_SR[0];
        Motor_L = Speed_L[1];
        Motor_R = Speed_R[1];
       
			if(Straight_Count > duanzhidao_Count)          //0.75m ??>20??
			{   
								
					v_D = Fuzzy_control(E,EC,0,1);    //0~50
					v_P = Fuzzy_control(E,EC,1,1);    //0~80
					Motor_L = Speed_L[0]+5 - v_P/8 - v_D/8;//??????
					Motor_R = Speed_R[0]+5 - v_P/8 - v_D/8;
					if(Straight_Count > changzhidao_Count)      //1.8m ??>50???
					{
									 //ON;
						 Motor_L = Speed_L[0]+12 - v_P/4 - v_D/4;//?????? ?????????
						 Motor_R = Speed_R[0]+12 - v_P/4 - v_D/4;
//						 if(Straight_Count >= CZsha_Count)//&&CCD_flag==0
//						 {
//								//ON;
//								Motor_L = Speed_L[0] - v_P/4 - v_D/4;//?????????
//								Motor_R = Speed_R[0] - v_P/4 - v_D/4;
//						 }
									 //else OFF;
					}
								//else OFF;
								sum_max = wandao_Flag;//
				}
		}
    else    //??????
    {         
        //ON;
				if(Straight_Count > changzhidao_Count)  //        ???             //?????  ?????                                                         //uint16 ss1 = 6; 
																																	 //uint16 ss2 = 6;
				{
									//ON;
						speed_Flag = 1;	
						C_ZHI = Speed_Value*10/ss2-5;//
						if(C_ZHI<12 || C_ZHI>1000) 
							  C_ZHI = C_Zhi_Value;	
									
				}
				else if(Straight_Count >= duanzhidao_Count)   // 50 ??????? ?????
				{
									//ON;   //?????
						speed_Flag = 2;	
						D_ZHI = Speed_Value*10/ss1-5;//
						if(D_ZHI<8 || D_ZHI>1000) 
							  D_ZHI = D_Zhi_Value;
				}
							//else OFF;
							
				v_D = Fuzzy_control(E,EC,0,1);
				v_P = Fuzzy_control(E,EC,1,1);
				Motor_L = Speed_L[1]+10-v_P/8;
				Motor_R = Speed_R[1]+10-v_P/8;
				Straight_Count = 0;
				//Range_max = wandao_Flag1;        //Range_max<400 ????? ???<400 ??????
				//sum_max = wandao_Flag;//400
		}
		if(ramp_flag==1)
		{
				Motor_L = Speed_L[3];
        Motor_R = Speed_R[3];
		}
//		if(huan_flag1!=0)
//		{
//				Motor_L = Speed_L[5];
//        Motor_R = Speed_R[5];
//		}
	
    if(1 == speed_Flag)                
    {
        //ON;
				//Beer_ON;
				Motor_L = Speed_L[4];
				Motor_R = Speed_R[4];        
				ShaChe_Count++;
				if(ShaChe_Count > C_ZHI)        
				{
						ShaChe_Count = 0;
						speed_Flag = 0;	
						C_ZHI = 0;
						//Beer_OFF;						
				}	
    }
		//else Beer_OFF;
     //else OFF;
    if(2 == speed_Flag)                    
    {	
        //ON;
				Motor_L = Speed_L[4];
        Motor_R = Speed_R[4];	          
				ShaChe_Count++;
				if(ShaChe_Count > D_ZHI)       
				{
						ShaChe_Count = 0;
						speed_Flag = 0;
						D_ZHI = 0;
				}
    }
		if(stop_flag==1)
		{
				Motor_L = 0;
        Motor_R = 0;
		}
		if(Confluence_flag==1)
		{
				Motor_L = 120;
        Motor_R = 120;
		}
		
		
	 if(Speed_Lmodu*100>135)
			Speed_Lmodu=1.35;
   if(Speed_Lmodu*100<6.5)
      Speed_Lmodu=0.65;
   if(Speed_Rmodu*100>135)
      Speed_Rmodu=1.35;
   if(Speed_Rmodu*100<65)
      Speed_Rmodu=0.65;
    
   if(-1 == direction)             
    {
      //ON;
      Motor_L = (int16_t)(Motor_L*Speed_Lmodu);      
      Motor_R = (int16_t)(Motor_R*Speed_Rmodu);      
    }
    else if(1 == direction)        
    {
      //ON;
      Motor_R = (int16_t)(Motor_R*Speed_Rmodu);      
      Motor_L = (int16_t)(Motor_L*Speed_Lmodu);      
    }
   // else OFF;
    
//    if(Motor_L>300)           
//      Motor_L = 300;
//    else if(Motor_L<15) 
//      Motor_L = 15;
//    
//    if(Motor_R>300) 
//      Motor_R = 300;
//    else if(Motor_R<15) 
//      Motor_R = 15;
    
    Motor_Lshow = Motor_L;
    Motor_Rshow = Motor_R;
    if(Straight_Count > 1500)  
       Straight_Count = 1500;	
		
//  //static int Straight_count=0;
//  static uint8_t stopflag=0;
//  static int speed_Flag=0;
//  static uint16_t Shache_count=0;
//  static uint16_t t=0;
//  static int wandao_flag=300;
//  static int lastMoSetPoint = 0;
//  static int circleCount = 0,circleSpeedFlag=0;
//  static int realZhiCount = 0;
//  static int changeFlag = 0;
//  //static int sideRunCount = 0,sideLowSpeedCount = 0;
//  //static int isInZhidaoFlag = 0;
//  
//  t++; if(t>5000)  t=5000;
//	
//  vp=Fuzzy_control(E,EC,1,1);
//  vd=Fuzzy_control(E,EC,0,1);
//  
//	zhiDaoJiaSuFlag = 0;
//  
//  
//  //速度策略及处理
//      Mo->SetPoint;//速度给定        

//             if(E < wandao_flag)
//             {
//                   Straight_count ++;
//                   //AD67Flag_straight_Count++;
//                   //StraightC = Straight_count;
//                  
//                                      
//                   //Beer_ON;
//                   Mo->SetPoint  =  speed_K + basicspeed/((basicspeed-Straight_count)>0?(basicspeed-Straight_count):1) ;//- 10
//                   //Mo->SetPoint  =  speed_K + basicspeed;
//                    
//                    if(E < 350)    //
//                    {
//                        StraightC++;
//                    }
//                    else
//                    {
//                        if(StraightC<100) StraightC = 0;                        
//                    }
//                    
//                   if(Straight_count > 20)
//                   {
//                         //if(realZhiCount>20)Beer_ON;
//                         
//                         
//                         if(Straight_count > 34)
//                         {
//                             zhiDao52cmFlag = 1;
//                             zhiDaoJiaSuFlag = 1;
//                         }
//                         
//                         Mo->SetPoint  = speed_K + basicspeed - vp - vd/2;//2
//                                                 
//                         if(Straight_count > 120)
//                         {
//                             //Beer_ON;
//                             Mo->SetPoint  =  speed_K + basicspeed - vp - vd/1;//1

//                         }
//                   }
//                  
//                   wandao_flag = 350;  //300
//             }
//             
//             else //弯道
//             {
//                   zhiDao52cmFlag = 0;                  
//                   isInZhidaoFlag = 0;
//                   nowZhiChangeFinishFlag = 0;
//                   realZhiCount = 0;
//                   if(Straight_count > 120)
//                   {
//                         speed_Flag = 1;   //长直入弯
////                         C_Zhi = speed/11;
////                         if(C_Zhi < 15)  C_Zhi = 15;
////                         if(C_Zhi > 20)  C_Zhi = 20;
//                         
//                   }                  
//                   else if(Straight_count > 30)
//                   {
//                         speed_Flag = 2;   //短直入弯                         
////                         D_Zhi =  speed/16;
////                         if(D_Zhi < 10 ) D_Zhi = 10;
////                         if(D_Zhi > 15 ) D_Zhi = 15;
//                   }
//                   
////                   if(StraightC > 100 && podaoflag==0 && startCount>150)  //可超车点
////                   {
////                       changeFlag = 3;
////                   }
////                   else 
////                   {
////                       changeFlag = 0;
////                   }
////                   if(isFrontCar_T == POSFRONT)  //???????
////                   {
////                       if(changeFlag == 3 && sideLowSpeedFlag==0)
////                       {                           
////                               realZhiCount++;      
////                               changeFlag = 0;
////                       }
////                       else
////                       {
////                           realZhiCount = 0;
////                       }
////                       if(realZhiCount>0 && isInZhidaoFlag == 0 &&nowZhiChangeFinishFlag==0 && circleTrcePoint == 0 && circleTrceFlag==0) 
////                       {
////                           //Beer_ON;                          
////                           //if(wanChangeFlag == 1)isInZhidaoFlag = 1;                             
////                       }else
////                       {
////                           isInZhidaoFlag = 0;
////                       }
////                      // if(isInZhidaoFlag == 1)Beer_ON;
////                   }
//                   
//                      if( 0 == speed_Flag)   
//                      {
//                            if(E > 700)   //????
//                            {
//                                  //Beer_ON;
//                                  Mo->SetPoint  =  speed_K  + curvespeed - 30 - vp*100/speed_P ;
//                            }
//                            else          //????
//                            {
//                                  //Beer_ON;
//                                  Mo->SetPoint  =  speed_K + curvespeed  - 10 - vp*100/speed_P1;
//                            }
//                      }
//                    Straight_count = 0;
//                   //AD67Flag_straight_Count = 0;
//                    if(E>650)StraightC = 0;
//                    wandao_flag = 400;  //350
//             }
//              if(1 == speed_Flag)   //长直入弯
//              {
//                  Mo->SetPoint  =  20;
//                    Shache_count++;
//                    if(Shache_count > C_Zhi)
//                    { 
//                        Shache_count = 0;
//                        speed_Flag = 0;
//                    }
//              }
//              if(2 == speed_Flag) //短直入弯
//              {
//                   Mo->SetPoint  =  20;
//                    Shache_count++;
//                    if(Shache_count > D_Zhi)
//                    { 
//                        Shache_count = 0;
//                        speed_Flag = 0;
//                    }
//              }

//             //计算加速度轨迹
////             Ac->SetPoint = Mo->SetPoint - lastMoSetPoint;
////             lastMoSetPoint = Mo->SetPoint;
////  if(singleRun < 1)
////  {
////        //双车速度控制
////		 Mo->SetPoint;//速度给定   
////  }
}

void Speed_LControl(int16_t Speed_LSet,int16_t Speed_LBack)
{  
    
        
                                               //            ???      ???
    Speed_LError_Now = Speed_LSet - Speed_LBack;  // ????  Motor_L - Speed_Value     
    
//-------------------?????PID??------------------------------------------
   
    Speed_LOut=(int16_t)(Speed_P*1.0*(Speed_LError_Now-Speed_LError_Last)/100.0+Speed_I*1.0*Speed_LError_Now/100.0+Speed_D*1.0*(Speed_LError_Now-2*Speed_LError_Last+Speed_LError_Prev)/100.0); // PID???????
   		

    Speed_LOut_show = Speed_LOut;
    Speed_LError_Prev = Speed_LError_Last;       // ?????
    Speed_LError_Last = Speed_LError_Now;        // ?????
   
    Motor_LOut = Motor_LOut + 10*Speed_LOut;         // ?????
//------------------------------------------------------------------------------  
    
    if(Motor_LOut >= Motor_Max)           // ???????
       Motor_LOut = Motor_Max;    
//----------------- bang-bang   bang!??  bang!?? ---------------------------
    if(Speed_LError_Now>30&&speed_Flag == 0)                 //???????? ??? ??
    {                                                  // v_D = Fuzzy_control(E,EC,0,1);    //0~50       
                                                       // v_P = Fuzzy_control(E,EC,1,1);    //0~80
      if(Speed_LError_Now>40)
        Motor_LOut = 6500;    
    }
    else if(-Speed_LError_Now>50&&(speed_Flag == 1||speed_Flag == 2||speed_Flag == 3))    //???  ??
    {
      if(-Speed_LError_Now>70)
        Motor_LOut = -3000;
      else if(-Speed_LError_Now>100)
        Motor_LOut = -3500;
    }
    
//---------------------------------??-----------------------------------------
//    if(Motor_LOut >= 0)
//    {
//       FTM_PWM_Duty(FTM0,DianJiChannel_LF,0);
//       Motor_LOut_Show = Motor_LOut;
//    }
//   
//    if(Motor_LOut < 0)
//    {
//       //ON;
//       if(-Motor_LOut > Motor_Min)
//          Motor_LOut = -Motor_Min;
//       Motor_LOut = Motor_LOut*2;
//       Motor_LOut_Show = Motor_LOut;
//    }
//    //else OFF;
////------------------------------------------------------------------------------
//    if(Motor_LOut>999)
//       Motor_LOut = 999;
//    if(-Motor_LOut>800)
//       Motor_LOut = -800;
//    if(Motor_LOut >= 0)
//    {
//      FTM_PWM_Duty(FTM0,DianJiChannel_LZ,(uint16)(Motor_LOut));
//    }
//    else if(Motor_LOut < 0)
//    {
//      FTM_PWM_Duty(FTM0,DianJiChannel_LF,(uint16)(-Motor_LOut));
//      Motor_LOut=0;
//    }
}

void Speed_RControl(int16_t Speed_RSet,int16_t Speed_RBack)
{  
    
        
                                               //            ???      ???
    Speed_RError_Now = Speed_RSet - Speed_RBack;  // ????  Motor_L - Speed_Value     
    
//-------------------?????PID??------------------------------------------
   
    Speed_ROut=(int16_t)(Speed_P*1.0*(Speed_RError_Now-Speed_RError_Last)/100.0+Speed_I*1.0*Speed_RError_Now/100.0+Speed_D*1.0*(Speed_RError_Now-2*Speed_RError_Last+Speed_RError_Prev)/100.0); // PID???????
   		

    Speed_ROut_show = Speed_ROut;
    Speed_RError_Prev = Speed_RError_Last;       // ?????
    Speed_RError_Last = Speed_RError_Now;        // ?????
   
    Motor_ROut = Motor_ROut + 10*Speed_ROut;         // ?????
//------------------------------------------------------------------------------  
    
    if(Motor_ROut >= Motor_Max)           // ???????
       Motor_ROut = Motor_Max;    
//----------------- bang-bang   bang!??  bang!?? ---------------------------
    if(Speed_RError_Now>30&&speed_Flag == 0)                 //???????? ??? ??
    {                                                  // v_D = Fuzzy_control(E,EC,0,1);    //0~50
                                                       // v_P = Fuzzy_control(E,EC,1,1);    //0~80
      if(Speed_RError_Now>40)
        Motor_ROut = 6500;    
    }
    else if(-Speed_RError_Now>50&&(speed_Flag == 1||speed_Flag == 2||speed_Flag == 3))           //???  ??
    {
      if(-Speed_RError_Now>70)
        Motor_ROut = -3000;
      else if(-Speed_RError_Now>100)
        Motor_ROut = -3500;
    }
     

//---------------------------------??-----------------------------------------
//    if(Motor_ROut >= 0)
//    {
//       FTM_PWM_Duty(FTM0,DianJiChannel_RF,0);
//       Motor_ROut_Show = Motor_ROut;
//    }
//   
//    if(Motor_ROut < 0)
//    {
//       //ON;
//       if(-Motor_ROut > Motor_Min)
//          Motor_ROut = -Motor_Min;
//       Motor_ROut = Motor_ROut*2;
//       Motor_ROut_Show = Motor_ROut;
//    }
    //else OFF;
//------------------------------------------------------------------------------
//    if(Motor_ROut>999)
//       Motor_ROut = 999;
//    if(-Motor_ROut>800)
//       Motor_ROut = -800;
//    if(Motor_ROut >= 0)
//    {
//      FTM_PWM_Duty(FTM0,DianJiChannel_RZ,(uint16)(Motor_ROut));
//    }
//    else if(Motor_ROut < 0)
//    {
//      FTM_PWM_Duty(FTM0,DianJiChannel_RF,(uint16)(-Motor_ROut));
//      Motor_ROut=0;
//    }
}

void set_Angle()                             //舵机控制 
{ 
       
       // Beer_OFF;                         //蜂鸣器关闭
      PWM_servo = signal2();
      ServoPWMDuty1(PWM_servo);        //占空比精度是20000  50000-PWM_servo/2
}

void set_Speed()                   
{
    static int PWM_inc=0;
    static int PWM_value1=0;
		static int PWM_value2=0;
		static int v=0;
		//ConuterClean();
	speed_flag++;
	if(speed_flag>=2){
		speed_L=CounterGet_L();
		speed_R=-CounterGet_R();
		ConuterClean();
		speed_flag=0;
	} 
	Speed_Value=(speed_L+speed_R)/2;
  if(mykey.go_flag||stop_flag==1)
  {         
     startCount++;
     if(startCount > 171) 
     {
         startCount = 171;
     }
		 if(startCount==171){
				 caculate_speed2();
////				 PWM_value1=Standard_incremental_PID(Mo, speed_L/Speed_Lmodu);
////				 PWM_value2=Standard_incremental_PID(Mo, speed_R/Speed_Rmodu);
//				 PWM_inc=1.5*ORG_chaspeed;
//				 pwm_dif=PWM_inc;
//				 if(PWM_inc<=0){
//							PWM_inc=-PWM_inc;
//				 }
//				 if(PWM_inc>PWM_inc_max){
//							PWM_inc=PWM_inc_max;
//				 }
//				 if(t!=0){
//							v=speed_mid_huan;
//				 }
//				 else if(Single_min>300){
//							v=speed_mid;
//				 }
//				 else if(Single_min<=300){
//							v=speed_mid_turn;
//				 }
//				 if(direction==1){
//							//PWM_inc=-PWM_inc;
//							PWM_value1=v-PWM_inc;
//							PWM_value2=v+PWM_inc;
//				 }
//				 else if(direction==-1){
//							//PWM_inc=PWM_inc;
//							PWM_value1=v+PWM_inc;
//							PWM_value2=v-PWM_inc;
//				 }
				 Speed_LControl(Motor_L,speed_L);
				 Speed_RControl(Motor_R,speed_R);
				 MotorPWMChange1(Motor_LOut);
				 MotorPWMChange2(Motor_ROut);
		 }
//    caculate_speed2();
//   
//    PWM_inc=Standard_incremental_PID(Mo, speed);
////    PWM_inc+=Standard_incremental_PID(Ac, (int)ace);
//    PWM_value+=PWM_inc;
////      SMo->SetPoint = Mo->SetPoint;
////      SMo->FeedPoint = speed;
////      PWM_value = u_control(SMo);

//    if(PWM_value>=0)    
//    {
//       if(PWM_value>MAXDUTY)  PWM_value=MAXDUTY; 
//       MotorPWMDuty(PWM_value);      
//    }
//    
//    else 
//    {
//				if (PWM_value<-MAXDUTY) PWM_value=-MAXDUTY;
//				 MotorPWMDuty(PWM_value);
//    }
////    MOTOR_GO_PWM(speed_K);
////    MOTOR_BACK_PWM(0);
    timerset(time_s); 
  }
      
}

void timerset(int time_s)
{
    static int i=0;
    i++;  if(i>120000) i= 120000;
//		if(i>= (huiche_t * 200) && time_s>=0 )
//		{
//				Confluence_flag=1;
//		}
//		if(i>= ((huiche_t+1) * 200) && time_s>=0 )
//		{
//				Confluence_flag=2;
//		}
    if(i>= (time_s * 200) && time_s>=0 )
    {   
        i=0;
        mykey.go_flag=0;        
        //Motorstop(0);
				stop_flag=1;
//        if(SD==1)
//        {
//            SD_Close();
//            SD=0;
//        }
    }  
}

void isDoubleWirelessTask()
{   
    static uint8_t lastCarFlag = 0;    
    if(lastCarFlag != thisCarState )
    {
        WirelessTask = 1;
        lastCarFlag = thisCarState;        
    }
    
}
void wireless_task()
{
//     int FFT1,FFT3,FFT5;
//     FFT1 = ((int)fftOutput[1])>>5;
//     FFT3 = (int)fftOutput[2];
//     FFT5 = (int)fftOutput[3];
     vcan_sendware(&AD_result[0],&AD_result[1],&AD_result[2],&AD_result[3],&speed_L,&speed_R,&ORG_servo,&ORG_chaspeed);
}
