#include "stm32f4xx.h"
#include "stm32f4xx_can.h"


#define FILTER_ID 0x550U
#define FRAME_ID  0x550U
#define FRAME_DLC 6

// LEDs
#define LED_TX     GPIO_ODR_ODR_12
#define LED_RX     GPIO_ODR_ODR_13
#define LED_ERROR  GPIO_ODR_ODR_14
#define LED_OK     GPIO_ODR_ODR_15

// Prototypes
void Clock_Init_HSE(void);
void Ports_Init(void);
void CAN1_Init_Config(void);
void CAN1_Filter_Setup(void);
void CAN1_Send_Frame(void);
void CAN1_Read_Frame(void);

// CAN structures
CAN_InitTypeDef   CAN_InitStruct;
CAN_FilterInitTypeDef CAN_FilterStruct;
CanTxMsg TxMsg;
CanRxMsg RxMsg;

// Test data
uint8_t send_data[6] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
uint8_t received_data[6];

// ===============================
//        HSE CLOCK INIT
// ===============================
void Clock_Init_HSE(void)
{
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    RCC->CFGR = RCC_CFGR_SW_0;
    while (!(RCC->CFGR & RCC_CFGR_SWS_0));
}

// ===============================
//          GPIO INIT
// ===============================
void Ports_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  
    GPIOA->MODER |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER11_1;
    GPIOA->AFR[1] |= (0x9 << 12) | (0x9 << 16);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= GPIO_MODER_MODER12_0 |
                    GPIO_MODER_MODER13_0 |
                    GPIO_MODER_MODER14_0 |
                    GPIO_MODER_MODER15_0;
}

// ===============================
//          CAN CONFIG
// ===============================
void CAN1_Init_Config(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    CAN_StructInit(&CAN_InitStruct);

    CAN_InitStruct.CAN_Mode = CAN_Mode_LoopBack;
    CAN_InitStruct.CAN_Prescaler = 2;
    CAN_InitStruct.CAN_BS1 = CAN_BS1_10tq;
    CAN_InitStruct.CAN_BS2 = CAN_BS2_5tq;
    CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;

    CAN_Init(CAN1, &CAN_InitStruct);
}

// ===============================
//         FILTER CONFIG
// ===============================
void CAN1_Filter_Setup(void)
{
    CAN_FilterStruct.CAN_FilterNumber = 0;
    CAN_FilterStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterStruct.CAN_FilterScale = CAN_FilterScale_32bit;

    CAN_FilterStruct.CAN_FilterIdHigh = (FILTER_ID << 5);
    CAN_FilterStruct.CAN_FilterIdLow = 0x0000;

    CAN_FilterStruct.CAN_FilterMaskIdHigh = (0x7FF << 5);
    CAN_FilterStruct.CAN_FilterMaskIdLow = 0x0000;

    CAN_FilterStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterStruct.CAN_FilterActivation = ENABLE;

    CAN_FilterInit(&CAN_FilterStruct);
}



// ===============================
//         CAN TRANSMIT
// ===============================
void CAN1_Send_Frame(void)
{
    TxMsg.StdId = FRAME_ID;
    TxMsg.IDE = CAN_Id_Standard;
    TxMsg.RTR = CAN_RTR_Data;
    TxMsg.DLC = FRAME_DLC;

    for (int i = 0; i < FRAME_DLC; i++)
        TxMsg.Data[i] = send_data[i];

    uint8_t mailbox = CAN_Transmit(CAN1, &TxMsg);
    uint8_t status = CAN_TransmitStatus(CAN1, mailbox);

    GPIOD->ODR |= LED_TX;

    if (status != 0)
        GPIOD->ODR |= LED_OK;
    else
        GPIOD->ODR |= LED_ERROR;
}

// ===============================
//       CAN RECEIVE FRAME
// ===============================
void CAN1_Read_Frame(void)
{
    CAN_Receive(CAN1, CAN_FIFO0, &RxMsg);

    GPIOD->ODR |= LED_RX;

    for (int i = 0; i < NEW_FRAME_DLC; i++)
        received_data[i] = RxMsg.Data[i];
}

// ===============================
//           MAIN
// ===============================
int main(void)
{
    Clock_Init_HSE();
    Ports_Init();
    CAN1_Init_Config();
    CAN1_Filter_Setup();
    CAN1_NVIC_Setup();

    CAN1_Send_Frame();
    CAN1_Read_Frame();

    while (1)
    {
    }
}
