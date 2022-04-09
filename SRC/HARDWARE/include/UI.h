//Liyaxian has compiled UI on 2015-07-06 
#ifndef UI_H
#define UI_H

#include "include.h"



#define uppper  &PTB21
#define down    &PTB17
#define left    &PTB16
#define right   &PTB20
#define page    &PTB9
#define mode    &PTB22
#define go      &PTB23

#define delayToGet  lptmr_delay_ms(80);

struct keystate
{
  int Ypos_a;  int Ypos_d;
  int page_a;  int page_d;
  int addstate;
  int mode_flag;
  int go_flag;
};

extern struct keystate mykey;


//for user

extern void UI_display(void);   //

//for inner
extern void UI_init(void);
extern void Key_scan(struct keystate*);  // keystate struct has been defined 
extern void Shownum_OLED_P6x8Str(uint8_t ,uint8_t ,int );
extern void Showname_OLED_P6x8Str(uint8_t x,int page_now,int N,char** ch,int **num);
extern void cursor_set(uint8_t x,uint8_t y);
extern void value_set(int *pp); //

extern void delay_ms (unsigned long zms);
#endif
