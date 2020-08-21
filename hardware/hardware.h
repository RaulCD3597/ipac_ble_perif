/**
 * @file hardware.h
 * @author Raul Camacho
 * @date July 2020
 * @brief Provides hardware control functions
 */

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

/* ---------------- public definitions ----------------*/

#define CONNECTED_LED               17
#define TEST_LED                    19
#define EMERGENCY_BUTTON            13
#define SERVICE_BUTTON              15
/**
 * Value used as error code on stack dump, can be used to identify 
 * stack location on stack unwind.
 */
#define DEAD_BEEF                   0xDEADBEEF

/* ------------------- public enums -------------------*/

/* ----------------- public functions -----------------*/

void hardware_init(void);
void hardware_sleep_mode_enter(void);

#endif /* _HARDWARE_H_ */