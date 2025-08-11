#ifndef _NIXIE_H
#define _NIXIE_H

#include "main.h"

#define DIGITS_NUM 6

void nixie_set_digits(uint8_t *digit);

void nixie_refresh(void);

#endif