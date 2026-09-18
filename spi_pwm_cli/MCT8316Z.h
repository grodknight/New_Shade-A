/*
 * MCT8316Z.h
 *
 *  Created on: Dec 21, 2022
 *      Author: Fabian
 */

#ifndef MCT8316Z_DRIVER_MCT8316Z_H_
#define MCT8316Z_DRIVER_MCT8316Z_H_

// #include "stm32l4xx_hal.h" /* Needed for SPI */
// #include <stdio.h>
#include "stdbool.h"
#include "stdint.h"

#include "MCT18316Z_REGS.h"

#include "spidrv.h"


 void setClockwise(bool clockwise);
 bool isClockwise(void);
 bool motorOn(void);
 bool motorOff(void);
 bool mct8316z_read_reg(uint8_t reg, uint8_t *value);
 
void MCT8316_ReadAllRegs(void);
bool mct8316z_write_reg(uint8_t reg, uint8_t value);
void mct8316z_UnlockRegs(void);


// From MCT18316Z_pwm.h

void MCT8316Z_set_frequency(uint32_t frequency_hz);
void MCT8316Z_set_duty ( uint8_t percent);
uint32_t MCT8316Z_get_frequency(void);
uint8_t  MCT8316Z_get_duty(void);

#endif