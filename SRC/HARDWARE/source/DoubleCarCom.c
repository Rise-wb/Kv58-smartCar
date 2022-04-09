#include "DoubleCarCom.h" 

CarState thisCar  = {       
                      0,//POSFRONT,
                      0,//CGSTAY,
                      0,//GOON5,
                      0,//UNSTOP,
                      0,//CHANDIS,
                      0,//RUNOVER,
                      0,//DIRLEFT,
                      0
                   };
CarState * thiC = &thisCar;
CarState anotCar ;
CarState * anoC = &anotCar;

uint8_t thisCarState;
uint8_t anotCarState = CLEAN;

uint8_t NrfTXBuff[3];
uint8_t NrfRXBuff[3];
void CommunicationInit()
{
      if(NRF2401_Init() == NRF_OK)
      {
           //Beer_ON;
           lptmr_delay_ms(50);
           NRF2401_SetRXMode();
      }
      
}

void TransmitData()    //∑¢ÀÕ
{
     NRF2401_SetTXMode();
     
     NrfTXBuff[0] = DTOP;
     //NrfTXBuff[1] = *((uint8_t *)thiC);
     NrfTXBuff[1] = thisCarState;
     NrfTXBuff[2] = DBTM;
     
     NRF2401_SendData(NrfTXBuff) ;  
     NRF2401_SetRXMode();
}

void ReceviceData()   //Ω” ’
{
    if(!NRF2401_RecData(NrfRXBuff));
    {
          if(NrfRXBuff[0] == DTOP && NrfRXBuff[2] == DBTM)
          {  
              //*((uint8_t *)anoC) =  NrfRXBuff[1]; 
              anotCarState = NrfRXBuff[1];
          }
    }
}
