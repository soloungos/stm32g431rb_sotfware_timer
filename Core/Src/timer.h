/*
 * timer.h
 *
 *  Created on: Dec 18, 2023
 *      Author: soloungos
 */

#ifndef SRC_TIMER_H_
#define SRC_TIMER_H_

#define TIMER_MAX_COUNT   (10)  /*the value of max*/
#define TIMER_TICK  (10)  /*10ms*/
#define TIMER_10ms  ((uint32_t)(  10/TIMER_TICK))
#define TIMER_50ms  ((uint32_t)(  50/TIMER_TICK))
#define TIMER_100ms ((uint32_t)( 100/TIMER_TICK))
#define TIMER_1s    ((uint32_t)(1000/TIMER_TICK))
#define TIMER_3s    ((uint32_t)(3000/TIMER_TICK))
#define TIMER_5s    ((uint32_t)(5000/TIMER_TICK))

typedef enum
{
  ONE_SHOT,
  PERIODIC,
}timer_mode;
typedef void (*timer_handler)(int8_t timer_id, uint32_t param1);

extern int8_t register_timer(uint32_t period, timer_handler handler, timer_mode mode);
extern int8_t unregister_timer(int8_t timer_id);
extern void   clear_timer(void);
extern void   tick_timer(void);
extern void   process_timer(uint32_t param1, uint32_t param2);


#endif /* SRC_TIMER_H_ */
