#include "stm32g4xx_hal.h"

#define APP_START_ADDRESS 0x08004000

#define SRAM_SIZE 128*1024
#define SRAM_END (SRAM_BASE + SRAM_SIZE)

void vStartBootloader(void);
void vJumpToApp(void);