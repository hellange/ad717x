
#include <stdint.h>
#include <Arduino.h>

#include "libraries/AD7176/AD7176_regs.h"
#include "libraries/AD7176/AD7176_Comm.h"
#include "libraries/AD7176/AD7176.h"

#ifndef _DAC_SETUP_H_
#define _DAC_SETUP_H_

extern st_reg init_state[];

void initADC7176(void);

#endif //_DAC_SETUP_H_
