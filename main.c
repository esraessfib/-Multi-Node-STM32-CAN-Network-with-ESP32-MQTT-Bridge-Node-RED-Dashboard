#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "ADC_Dma.h"
#include <stdio.h>

// ---- Déclarations externes ----
extern void config_ADC1();
extern void config_DMA2();
extern void config_TIM2();

extern volatile uint16_t ADC_VALUE[3];

// ---- CAN IDs ----
#define FILTRE_ID 0x600
#define RESPONSE_ID 0x100
#define FRAME_DLC 3U

// ---- ADC Channels ----
#define channel_4 0x10
#define channel_5 0x20
#define channel_6 0x30

// ---- LEDs ----
#define LED_GREEN GPIO_ODR_ODR_12  // Reception
#define LED_ORANGE GPIO_ODR_ODR_13 // Transmission OK
#define LED_RED GPIO_ODR_ODR_14    // Transmission KO

// ---- Variables CAN ----
CAN_InitTypeDef CAN_InitStruct;
CAN_FilterInitTypeDef Filter_InitStruct;
CanTxMsg TxMessage;
CanRxMsg RxMessage;

// ---- Prototypes ----
void SystemClock_Config(void);
void GPIO_Init(void);
void CAN_config(void);
void Filter_Config(void);
void NVIC_Config(void);
uint8_t CAN1_Transmit(void);

// ---- Système Clock ----
void SystemClock_Config(void)
{
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

    RCC->PLLCFGR = (4 << RCC_PLLCFGR_PLLM_Pos) | (168 << RCC_PLLCFGR_PLLN_Pos) |
                   (0 << RCC_PLLCFGR_PLLP_Pos) | RCC_PLLCFGR_PLLSRC_HSE | (4 << RCC_PLLCFGR_PLLQ_Pos);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

// ---- GPIO ----
void GPIO_Init(void)
{
    // LEDs
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0;

    // CAN1 PB8 (TX) PB9 (RX)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~((3 << 16) | (3 << 18));
    GPIOB->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1; // AF mode
    GPIOB->AFR[1] &= ~((0xF << 0) | (0xF << 4));
    GPIOB->AFR[1] |= (0x9 << 0) | (0x9 << 4); // AF9 = CAN1
    GPIOB->OSPEEDR |= (0x3 << 16) | (0x3 << 18); // Very High
}

// ---- CAN1 configuration ----
void CAN_config(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    CAN_StructInit(&CAN_InitStruct);
    CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStruct.CAN_Prescaler = 2; // 250 kbit/s
    CAN_InitStruct.CAN_BS1 = CAN_BS1_10tq;
    CAN_InitStruct.CAN_BS2 = CAN_BS2_5tq;
    CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;

    if (CAN_Init(CAN1, &CAN_InitStruct) == CAN_InitStatus_Success)
        GPIOD->ODR |= LED_ORANGE; // Indique CAN ok
    else
        GPIOD->ODR |= LED_RED;
}

// ---- CAN Filter ----
void Filter_Config(void)
{
    Filter_InitStruct.CAN_FilterNumber = 18;
    Filter_InitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
    Filter_InitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
    Filter_InitStruct.CAN_FilterIdHigh = FILTRE_ID << 5;
    Filter_InitStruct.CAN_FilterIdLow = 0x0000;
    Filter_InitStruct.CAN_FilterMaskIdHigh = 0x7FF << 5; // Exact match
    Filter_InitStruct.CAN_FilterMaskIdLow = 0x0000;
    Filter_InitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    Filter_InitStruct.CAN_FilterActivation = ENABLE;

    CAN_SlaveStartBank(20);
    CAN_FilterInit(&Filter_InitStruct);
}

// ---- NVIC ----
void NVIC_Config(void)
{
    NVIC_SetPriority(CAN1_RX0_IRQn, 1);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); // Interrupt FIFO0
}

// ---- CAN Transmit ----
uint8_t CAN1_Transmit(void)
{
    return CAN_Transmit(CAN1, &TxMessage);
}

// ---- CAN RX Interrupt Handler ----
void CAN1_RX0_IRQHandler(void)
{
    if (CAN_MessagePending(CAN1, CAN_FIFO0))
    {
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
        GPIOD->ODR |= LED_GREEN; // Indique réception

        // Préparer la réponse ADC
        if (RxMessage.Data[0] == channel_4)
        {
            TxMessage.StdId = RESPONSE_ID;
            TxMessage.IDE = CAN_Id_Standard;
            TxMessage.RTR = CAN_RTR_Data;
            TxMessage.DLC = FRAME_DLC;
            TxMessage.Data[0] = channel_4;
            TxMessage.Data[1] = ADC_VALUE[0] & 0xFF;
            TxMessage.Data[2] = ADC_VALUE[0] >> 8;

            uint8_t mailbox = CAN1_Transmit();
            if (CAN_TransmitStatus(CAN1, mailbox) == CANTXOK)
                GPIOD->ODR |= LED_RED;
            else
                GPIOD->ODR |= LED_ORANGE;
        }
        // Ajouter ici channel_5 / channel_6 si nécessaire
    }
    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
}

// ---- MAIN ----
int main(void)
{
    SystemClock_Config();
    GPIO_Init();

    config_ADC1();
    config_DMA2();
    config_TIM2();

    CAN_config();
    Filter_Config();
    CAN_OperatingModeRequest(CAN1, CAN_OperatingMode_Normal);
    NVIC_Config();

    // Préparer message initial (optionnel)
    TxMessage.StdId = RESPONSE_ID;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = FRAME_DLC;
    TxMessage.Data[0] = channel_4;
    TxMessage.Data[1] = ADC_VALUE[0] & 0xFF;
    TxMessage.Data[2] = ADC_VALUE[0] >> 8;

    while (1)
    {
        // Tout est géré par interruption
    }
}
