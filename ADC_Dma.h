//#ifndef ADC_DMA
#define ADC_Dma

#include "stm32f4xx.h"

extern uint16_t ADC_VALUE[3];  // Array to store ADC values

// Function declarations
void config_EXTI(void);
void	config_TIM2(void);
void  config_ADC1(void);
void	config_DMA2(void);
//#endif
