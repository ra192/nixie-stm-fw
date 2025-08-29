#ifndef _NIXIE_H
#define _NIXIE_H

#include "main.h"

#define DIGITS_NUM 6
#define EMPTY_DIGIT 10

void nixie_setDigits(uint8_t *digit);

void nixie_refresh(void);

uint8_t nixie_blink(uint8_t digit);

#endif