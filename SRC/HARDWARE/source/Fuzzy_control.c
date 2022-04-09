
#include "Fuzzy_control.h"
#include "fsl_common.h"
//------------------------Steer-------------------------
//Ä£ºý¿ØÖÆÆ÷
/***************FUZZY_steer*????*****************/
//Input P language feature.
int32_t PFF[5]= {0,150,350,500,700};//{0,100,300,500,700};//{0,150,350,550,800};//{0,250,450,650,800};  {0,200,400,600,800};

//Input D language feature.  
int32_t DFF[5]={0,10,20,40,90};

//Output UD language feature.  
int32_t UD[5]={300,700,1100,1500,1900};//{300,700,1100,1500,1900};   //{240,560,880,1200,1520};

//Output UP language feature.
int32_t UP[5]={25,40,50,60,70};// 15 25 40 50 65{25,40,50,60,80}{15,25,40,50,65}  {25,45,55,65,80};{25,40,50,60,80};{10,20,30,50,70};

int32_t P_rule[5][5]=
{ 
//4p3p2p1p0p1p2p3p4p
  {0,1,2,3,4},//0d   0,1,2,3,4    //4

  {0,1,2,3,4},//1d   0,1,2,3,4    //5     {3,1,0,0,0,1,3,}

  {1,1,3,3,4},//2d   1,1,3,3,4    //6

  {1,2,3,4,4},//3d   1,2,3,4,4    //7

  {2,3,3,4,4} //4d   2,3,3,4,4    //8
};


int32_t D_rule[5][5]=
{  
//d0,1,2,3,4,5,6,7,8
  {0,1,1,2,2},//4p    0,1,1,2,2   //4
  
  {0,1,1,2,2},//5p    0,1,1,2,2   //5

  {1,1,2,3,3},//6p    1,1,2,3,3   //6

  {1,2,3,3,4},//7p    1,2,3,3,4   //7

  {2,2,3,4,4},//8p    2,2,3,4,4   //8
};
//////////////FUZZY_speed????/////////////////////
int32_t pff[5]={0,200,300,400,500};


int32_t dff[5]={0,8,16,32,64};

 
int32_t ud[5]={0,20,80,200,400}; //20 80 150 300

int32_t up[5]={0,10,20,40,80};

int32_t p_rule[5][5]=
{ 
  {0,1,2,3,4},

  {0,1,2,3,4},

  {0,1,2,3,4},

  {1,1,2,3,4},//{0,0,1,1,0,1,2,3,4}

  {1,2,3,3,4} // {0,1,1,2,0,1,2,3,4}

};

int32_t d_rule[5][5]=
{  
 //0,1,2,3,4,5,6,
  {0,1,1,2,2},
  
  {0,1,2,2,3},

  {0,1,2,3,4},

  {1,1,2,3,4},

  {1,2,3,4,4}
};



int Fuzzy_control(int value1,int Dvalue1,unsigned char flag,unsigned char flag1)
{//                      p            d                1-p  0-d         1-sped  0-direction
  unsigned char x1,y1,x2,y2;
  int Error_tmp,DError_tmp,U_tmp[4],UF_tmp[4],UF[4];
			Error_tmp=value1;
			DError_tmp=Dvalue1;
  
  if(Error_tmp<0)Error_tmp=-Error_tmp;
  if(DError_tmp<0)DError_tmp=-DError_tmp;
  
  //Membership function
  if(flag1==0)
  {
    if(Error_tmp<=PFF[1])
    {
      x1=0;
      x2=1;
      UF_tmp[1]=100*(Error_tmp-PFF[0])/(PFF[1]-PFF[0]);
      UF_tmp[0]=100*(PFF[1]-Error_tmp)/(PFF[1]-PFF[0]);
    }
    else if(Error_tmp<=PFF[2])
    {
      x1=1;
      x2=2;
      UF_tmp[1]=100*(Error_tmp-PFF[1])/(PFF[2]-PFF[1]);
      UF_tmp[0]=100*(PFF[2]-Error_tmp)/(PFF[2]-PFF[1]);
    }
    else if(Error_tmp<=PFF[3])
    {
      x1=2;
      x2=3;
      UF_tmp[1]=100*(Error_tmp-PFF[2])/(PFF[3]-PFF[2]);
      UF_tmp[0]=100*(PFF[3]-Error_tmp)/(PFF[3]-PFF[2]);
    }
    else if(Error_tmp<=PFF[4])
   {
      x1=3;
      x2=4;
      UF_tmp[1]=100*(Error_tmp-PFF[3])/(PFF[4]-PFF[3]);
      UF_tmp[0]=100*(PFF[4]-Error_tmp)/(PFF[4]-PFF[3]);
   }
    else 
    {
      x1=4;
      x2=4;
      UF_tmp[1]=100;
      UF_tmp[0]=100;
    //UF_tmp[0]=0;
    }
  
    if(DError_tmp<=DFF[1])                          
    {
      y1=0;
      y2=1;
      UF_tmp[3]=100*(DError_tmp-DFF[0])/(DFF[1]-DFF[0]);  //  /10
      UF_tmp[2]=100*(DFF[1]-DError_tmp)/(DFF[1]-DFF[0]);
    }
    else if(DError_tmp<=DFF[2])
    {
      y1=1;
      y2=2;
      UF_tmp[3]=100*(DError_tmp-DFF[1])/(DFF[2]-DFF[1]);  //  /10
      UF_tmp[2]=100*(DFF[2]-DError_tmp)/(DFF[2]-DFF[1]);
    }
    else if(DError_tmp<=DFF[3])
    {
      y1=2;
      y2=3;
      UF_tmp[3]=100*(DError_tmp-DFF[2])/(DFF[3]-DFF[2]);   //  /20
      UF_tmp[2]=100*(DFF[3]-DError_tmp)/(DFF[3]-DFF[2]);
    }
    else if(DError_tmp<=DFF[4])
    {
      y1=3;
      y2=4;
      UF_tmp[3]=100*(DError_tmp-DFF[3])/(DFF[4]-DFF[3]);  //  /50
      UF_tmp[2]=100*(DFF[4]-DError_tmp)/(DFF[4]-DFF[3]);
    }
    else 
    {
      y1=4;
      y2=4;
      UF_tmp[3]=100;
      UF_tmp[2]=100;
    }
  
//    x1+=4;
//    y1+=4;
//    x2+=4;
//    y2+=4;
                                    //P                   //D
    if(UF_tmp[0]<=UF_tmp[2])UF[0]=UF_tmp[0]; else UF[0]=UF_tmp[2]; 
    if(UF_tmp[0]<=UF_tmp[3])UF[1]=UF_tmp[0]; else UF[1]=UF_tmp[3]; 
    if(UF_tmp[1]<=UF_tmp[2])UF[2]=UF_tmp[1]; else UF[2]=UF_tmp[2]; 
    if(UF_tmp[1]<=UF_tmp[3])UF[3]=UF_tmp[1]; else UF[3]=UF_tmp[3]; 
  
    //Look up the form and calculate the focus.
    if(flag==1)   //P
    {
    	U_tmp[0]=UP[P_rule[y1][x1]];   //x p
    	U_tmp[1]=UP[P_rule[y2][x1]];   //y d
    	U_tmp[2]=UP[P_rule[y1][x2]];
    	U_tmp[3]=UP[P_rule[y2][x2]];
    
    	return (U_tmp[0]*UF[0]+U_tmp[1]*UF[1]+U_tmp[2]*UF[2]+U_tmp[3]*UF[3])/(UF[0]+UF[1]+UF[2]+UF[3]);
    }
    else          //D
    {
      	U_tmp[0]=UD[D_rule[x1][y1]];
      	U_tmp[1]=UD[D_rule[x2][y1]];
      	U_tmp[2]=UD[D_rule[x1][y2]];
      	U_tmp[3]=UD[D_rule[x2][y2]];
    
    	return (U_tmp[0]*UF[0]+U_tmp[1]*UF[1]+U_tmp[2]*UF[2]+U_tmp[3]*UF[3])/(UF[0]+UF[1]+UF[2]+UF[3]);
    }
  }
///////////////////////////////////////////////////////////////////////////////////////////////////??
 if(flag1==1)
   {
      if(Error_tmp<=pff[1])
    {
     x1=0;
     x2=1;
     UF_tmp[1]=100*(Error_tmp-pff[0])/(pff[1]-pff[0]);
     UF_tmp[0]=100*(pff[1]-Error_tmp)/(pff[1]-pff[0]);
    }
    else if(Error_tmp<=pff[2])
    {
      x1=1;
      x2=2;
      UF_tmp[1]=100*(Error_tmp-pff[1])/(pff[2]-pff[1]);
      UF_tmp[0]=100*(pff[2]-Error_tmp)/(pff[2]-pff[1]);
    }
    else if(Error_tmp<=pff[3])
    {
      x1=2;
      x2=3;
      UF_tmp[1]=100*(Error_tmp-pff[2])/(pff[3]-pff[2]);
      UF_tmp[0]=100*(pff[3]-Error_tmp)/(pff[3]-pff[2]);
    }
   else if(Error_tmp<=pff[4])
    {
      x1=3;
      x2=4;
      UF_tmp[1]=100*(Error_tmp-pff[3])/(pff[4]-pff[3]);
      UF_tmp[0]=100*(pff[4]-Error_tmp)/(pff[4]-pff[3]);
    }
    else 
    {
      x1=4;
      x2=4;
      UF_tmp[1]=100;
      UF_tmp[0]=100;
    //UF_tmp[0]=0;
    }
  
    if(DError_tmp<=dff[1])
    {
      y1=0;
      y2=1;
      UF_tmp[3]=100*(DError_tmp-dff[0])/(dff[1]-dff[0]);
      UF_tmp[2]=100*(dff[1]-DError_tmp)/(dff[1]-dff[0]);
    }
    else if(DError_tmp<=dff[2])
    {
      y1=1;
      y2=2;
      UF_tmp[3]=100*(DError_tmp-dff[1])/(dff[2]-dff[1]);
      UF_tmp[2]=100*(dff[2]-DError_tmp)/(dff[2]-dff[1]);
    }
    else if(DError_tmp<=dff[3])
    {
      y1=2;
      y2=3;
      UF_tmp[3]=100*(DError_tmp-dff[2])/(dff[3]-dff[2]);
      UF_tmp[2]=100*(dff[3]-DError_tmp)/(dff[3]-dff[2]);
    }
    else if(DError_tmp<=dff[4])
    {
      y1=3;
      y2=4;
      UF_tmp[3]=100*(DError_tmp-dff[3])/(dff[4]-dff[3]);
      UF_tmp[2]=100*(dff[4]-DError_tmp)/(dff[4]-dff[3]);
    }
    else 
    {
      y1=4;
      y2=4;
      UF_tmp[3]=100;
      UF_tmp[2]=100;
    }
  
//    x1+=4;
//    y1+=4;
//    x2+=4;
//    y2+=4;

    if(UF_tmp[0]<=UF_tmp[2])UF[0]=UF_tmp[0]; else UF[0]=UF_tmp[2]; 
    if(UF_tmp[0]<=UF_tmp[3])UF[1]=UF_tmp[0]; else UF[1]=UF_tmp[3]; 
    if(UF_tmp[1]<=UF_tmp[2])UF[2]=UF_tmp[1]; else UF[2]=UF_tmp[2]; 
    if(UF_tmp[1]<=UF_tmp[3])UF[3]=UF_tmp[1]; else UF[3]=UF_tmp[3]; 
  
    //Look up the form and calculate the focus.
    if(flag==1)
    {
    	U_tmp[0]=up[p_rule[y1][x1]];
    	U_tmp[1]=up[p_rule[y2][x1]];
    	U_tmp[2]=up[p_rule[y1][x2]];
    	U_tmp[3]=up[p_rule[y2][x2]];
    
    	return (U_tmp[0]*UF[0]+U_tmp[1]*UF[1]+U_tmp[2]*UF[2]+U_tmp[3]*UF[3])/(UF[0]+UF[1]+UF[2]+UF[3]);
    }
    else
    {
      	U_tmp[0]=ud[d_rule[x1][y1]];
				U_tmp[1]=ud[d_rule[x2][y1]];
      	U_tmp[2]=ud[d_rule[x1][y2]];
      	U_tmp[3]=ud[d_rule[x2][y2]];
    
    	return (U_tmp[0]*UF[0]+U_tmp[1]*UF[1]+U_tmp[2]*UF[2]+U_tmp[3]*UF[3])/(UF[0]+UF[1]+UF[2]+UF[3]);
    }
   }
  //return 0;

}
