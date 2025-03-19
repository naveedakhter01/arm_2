//functions for generating random numbers
// - time used as input for seed

#include "random.h"
#include "rtc.h"
#include <stdlib.h>
#include <stdio.h>

static bool isSeedrandomized = FALSE;


// creates seed value from time
void RandomizeSeed(void)
{
  uint32_t seed = 0x79e3C46A; // this will be modified by xor'ing with time values
  uint8_t randomValues[4]; // 4 to hold  random values created from sec, min, hour, +  day, month, year 
  uint8_t *ptr;
  
  rtcTimeTypeDef *nowTime;
  rtcDateTypeDef *nowDate;
   
  nowTime = RtcGetBufferedTime();
  nowDate = RtcGetBufferedDate();
  
  //load random values from time
  randomValues[2] = nowTime->min ^ (nowTime->sec << 4);
  randomValues[1] = ~nowTime->sec ^ (~nowTime->hour << 4);
  randomValues[0] = nowTime->hour ^ (nowTime->min << 4);
  randomValues[3] = nowDate->date ^ (nowDate->month <<3) ^ (nowDate->year << 6);
  
  for (int i = 0; i < 4; i++) // loop for each random value
  {
    ptr = (uint8_t*)&seed;
    *ptr++ ^= randomValues[i];
    *ptr++ ^= randomValues[i];
    *ptr++ ^= randomValues[i];
    *ptr ^= randomValues[i];
    
    seed = (seed << 3 ) | (seed >> 29);
  }
    
    srand(seed); 
  isSeedrandomized = TRUE;
}



int GetRandomValue(int maxValue)
{
  if (!isSeedrandomized) RandomizeSeed();
  return rand() % maxValue; // 
}









