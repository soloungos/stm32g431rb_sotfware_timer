/*
 * timer.c
 *
 *  Created on: Dec 18, 2023
 *      Author: soloungos
 */
#include "main.h"
#include "timer.h"

typedef struct
{
  int8_t   id;
  uint8_t  busy;
  uint32_t tick;
  uint32_t period;
  timer_handler handler;
  timer_mode mode;
}timer_t;

static timer_t timer[TIMER_MAX_COUNT]={0};

/**
  * @brief register_timer
  */
int8_t register_timer(uint32_t period, timer_handler handler, timer_mode mode)
{
  for(int8_t i=0; i<TIMER_MAX_COUNT; i++)
  {
    if(timer[i].handler == NULL)
    {
      timer[i].tick = 0;
      timer[i].id = i;
      timer[i].busy = 0;
      timer[i].period = period;
      timer[i].handler = handler;
      timer[i].mode = mode;
      return i;
    }
  }

  return -1;
}

/**
  * @brief unregister_timer
  */
int8_t unregister_timer(int8_t timer_id)
{
  if((timer_id > 0) && (timer_id < TIMER_MAX_COUNT))
  {
    timer[timer_id].handler = NULL;
    return 0;
  }

  return -1;
}

/**
  * @brief clear_timer
  */
void clear_timer(void)
{
  for(int8_t i=0; i<TIMER_MAX_COUNT; i++)
  {
    timer[i].handler = NULL;
  }
}

/**
  * @brief process_timer
  */
void process_timer(uint32_t param1, uint32_t param2)
{
  for(int8_t i=0; i<TIMER_MAX_COUNT; i++)
  {
    if(timer[i].handler == NULL)
    {
      continue;
    }

    if(timer[i].tick >= timer[i].period)
    {
      timer[i].busy = 1;
      timer[i].tick = 0;
      timer[i].handler(timer[i].id, param1);
      timer[i].busy = 0;

      if(timer[i].mode == ONE_SHOT)
      {
        unregister_timer(i);
      }
    }
  }
}

/**
  * @brief process_timer
  */
void tick_timer(void)
{
  for(int8_t i=0; i<TIMER_MAX_COUNT; i++)
  {
    if(timer[i].handler == NULL)
    {
      continue;
    }

    if(timer[i].busy == 0)
    {
      timer[i].tick++;
    }
  }
}





