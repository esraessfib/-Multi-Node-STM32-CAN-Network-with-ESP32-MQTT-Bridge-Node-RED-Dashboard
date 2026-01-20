/*
Quand le nœud central envoie une requête CAN (ID 0x550), le STM32 :
    - Reçoit la requête (interruption CAN)
    - Lit la température du DS1621
    - Envoie la réponse via CAN (ID 0x681)
*/

/*
 ******************************************************************************
  * DS1621 Connection to I2C1:
  * - SDA to PB7
  * - SCL to PB6
  * - VDD to 3.3V
  * - GND to GND
  * - A0, A1, A2 to GND (address 0x48)
  * 
  * CAN1 Connection:
  * - CAN_RX to PB8
  * - CAN_TX to PB9
  * 
  * USART3 (Debug):
  * - PD8 = TX
  * - PD9 = RX
  ******************************************************************************
  */

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include <stdint.h>
#include <stdio.h>

/* ================== DS1621 Definitions ================== */
#define DS1621_WRITE_ADDR          0x90
#define DS1621_READ_ADDR           0x91
#define DS1621_START_CONVERSION    0xEE
#define DS1621_READ_TEMP           0xAA
#define DS1621_ACCESS_CONFIG       0xAC
#define DS1621_ADDR                0x48

/* ================== CAN Definitions ================== */
#define CAN_ID_TEMP_REQUEST   0x550  // ID pour recevoir la requête
#define CAN_ID_TEMP_RESPONSE  0x681  // ID pour envoyer la réponse

/* ================== Global Variables ================== */
float current_temperature = 0.0f;
int bol=0;
/* ================== Function Prototypes ================== */
//void hse_clk(void);
void SystemClock_Config(void);
void GPIO_Config(void);
void config_I2C1(void);
void CAN_Config(void);
void USART3_init(void);
void USART3_send_string(char* str);
void delay_ms(uint32_t ms);

void DS1621_Init(void);
void I2C_COMMAND(int command);
void Start_DS1621_Conv(void);
float I2C_ReadTemperature(void);

void CAN_Send_Temperature(float temp);


/* ================== Function to enable the High-Speed External (HSE) clock ================== */
/*void hse_clk(void) 
{
    // The HSE clock operates at 8MHz, providing lower energy consumption and improved stability compared to the High-Speed Internal (HSI) clock.
    // Note: The HSI clock, which relies on an RC circuit, may generate heat during operation.

    // Step 1: Enable the HSE clock
    RCC->CR |= RCC_CR_HSEON; // Set HSEON bit to turn on HSE

    // Step 2: Wait until the HSE is ready
    while (!(RCC->CR & RCC_CR_HSERDY)); // Polling the HSERDY flag to ensure HSE is stable
    //FLASH->ACR |= FLASH_ACR_LATENCY_0WS;  //  IMPORTANT
    // Step 3: Select HSE as the system clock source
    RCC->CFGR = RCC_CFGR_SW_0; // Set SW bits to select HSE as system clock (multiplexer configuration)

    // Step 4: Wait until the system clock switch is complete
    while (!(RCC->CFGR & RCC_CFGR_SWS_0)); // Polling the SWS bits to confirm the system clock is now HSE
}

{
    // 1. Activer HSE 
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));

    // 2. Configurer PLL 
    
       //HSE = 8 MHz
       //PLLM = 8   => VCO input = 1 MHz
       //PLLN = 336 => VCO output = 336 MHz
       //PLLP = 2   => SYSCLK = 168 MHz
       //PLLQ = 7   => pour USB/SDIO
    
    RCC->PLLCFGR = (8 << 0)   |    // PLLM
                   (336 << 6) |    // PLLN
                   (0 << 16)  |    // PLLP = 2
                   (RCC_PLLCFGR_PLLSRC_HSE) | 
                   (7 << 24);      // PLLQ

    // 3. Activer PLL 
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    // 4. Configurer Flash latency 
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

    // 5. Sélectionner PLL comme SYSCLK 
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // 6. Configurer prescaler APB1 et APB2 
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;  // APB1 = 42 MHz
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;  // APB2 = 84 MHz
		RCC->CFGR |= RCC_CFGR_HPRE_DIV1;   // AHB = 168 MHz
	}*/ 		
void SystemClock_Config(void)
{
    // 1. Enable HSE 
    RCC->CR |= 0x00010000;              // HSEON
    while (!(RCC->CR & 0x00020000));    // HSERDY

    // 2. Enable Power interface clock 
    RCC->APB1ENR |= 0x10000000;          // PWREN

    // 3. Voltage regulator Scale 1 
    PWR->CR |= 0x00004000;               // VOS = Scale 1

    // 4. Flash configuration
     //  - 5 wait states
       //- ICEN | DCEN | PRFTEN
    
    FLASH->ACR = 0x00000705;
    //
      // LATENCY = 5WS  -> bits [2:0] = 101
      // PRFTEN         -> bit 8
      // ICEN           -> bit 9
     //  DCEN           -> bit 10
   

    // 5. PLL configuration
      // PLLM = 4
       //PLLN = 168
       //PLLP = 2
       //PLLSRC = HSE
       //PLLQ = 4
    
    RCC->PLLCFGR = 0x07402A04;
    //
      // PLLM  = 4      -> bits [5:0]
       //PLLN  = 168    -> bits [14:6]
       //PLLP  = 2      -> bits [17:16] = 00
       //PLLSRC = HSE   -> bit 22
       //PLLQ  = 4      -> bits [27:24]
    

    // 6. Enable PLL 
    RCC->CR |= 0x01000000;               // PLLON
    while (!(RCC->CR & 0x02000000));     // PLLRDY

    // 7. Prescalers
      // AHB  = SYSCLK /1
       //APB1 = HCLK /4
       //APB2 = HCLK /2
    
    RCC->CFGR |= 0x00009400;
    
    //   HPRE  = 1   -> 0000
      // PPRE1 = /4  -> 101
      // PPRE2 = /2  -> 100
    

    // 8. Select PLL as system clock 
    RCC->CFGR &= ~0x00000003;            // Clear SW bits
    RCC->CFGR |= 0x00000002;             // SW = PLL

    while ((RCC->CFGR & 0x0000000C) != 0x00000008); // SWS = PLL
}




 /* ================== Configuration des GPIO pour CAN et LED   ================== */

void GPIO_Config(void)
{
    /* Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;
    
    /* CAN1 - PB8(RX), PB9(TX) */
    GPIOB->MODER &= ~((3U << 16) | (3U << 18));
    GPIOB->MODER |=  ((2U << 16) | (2U << 18));  // AF mode
    GPIOB->OSPEEDR |= ((3U << 16) | (3U << 18)); // Very high speed
    GPIOB->OTYPER &= ~((1U << 8) | (1U << 9));   // Push-pull
    GPIOB->PUPDR &= ~((3U << 16) | (3U << 18));  // No pull-up
    GPIOB->AFR[1] &= ~((0xF << 0) | (0xF << 4));
    GPIOB->AFR[1] |=  ((9U << 0) | (9U << 4));   // AF9 = CAN1
    
    /* LEDs - PD12(Green), PD13(Orange), PD14(Red) */
    GPIOD->MODER &= ~((3U << 24) | (3U << 26) | (3U << 28));
    GPIOD->MODER |=  ((1U << 24) | (1U << 26) | (1U << 28));  // Output
    GPIOD->OSPEEDR |= ((2U << 24) | (2U << 26) | (2U << 28)); // Medium speed
    GPIOD->ODR &= ~((1 << 12) | (1 << 13) | (1 << 14));       // All off
}


/* ================== Configuration I2C1 pour DS1621 ================== */
  
void config_I2C1(void)
{
    // Configuration GPIO pour I2C1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  
    GPIOB->OTYPER |= GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7;  
    GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; 
    GPIOB->AFR[0] |= GPIO_AFRL_AFRL6_2 | GPIO_AFRL_AFRL7_2;  

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; 
    // Configure I2C1 for 100 kHz	
    I2C1->CCR &= ~I2C_CCR_FS;  
    I2C1->CR2 = 42;  // APB1 = 42 MHz frequency in MHz
    I2C1->CCR = 210; // 42MHz / (2 * 100kHz) = 210
    I2C1->TRISE = 43;  // (1000ns / 23.8ns) + 1 = 43
    I2C1->CR1 |= I2C_CR1_PE; 
}


/* ================== Configuration de CAN1 ================== */

void CAN_Config(void)
{
    CAN_InitTypeDef CAN_InitStruct; // cet instant, on a creer en memoire la variable CAN_InitStruct contenant des valeurs al?atoiresde de type CAN_InitTypeDef  qui est une  structure (declare ds can.h )  
    CAN_FilterInitTypeDef CAN_FilterStruct;
    
    /* Activation horloge CAN1 */
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
	
	  /* Reset CAN1 */
    RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;
    delay_ms(10);
    
    /* Configuration CAN */
    CAN_InitStruct.CAN_TTCM = DISABLE;
    CAN_InitStruct.CAN_ABOM = ENABLE;   // Automatic Bus-Off Management
    CAN_InitStruct.CAN_AWUM = DISABLE;
    CAN_InitStruct.CAN_NART = DISABLE;  // Automatic Retransmission
    CAN_InitStruct.CAN_RFLM = DISABLE;
    CAN_InitStruct.CAN_TXFP = DISABLE;
    CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
    
    /* Configuration du bit timing pour 500 kbps avec HSE 8MHz */
    /* APB1 = 8MHz (car pas de PLL) */
    /* Baudrate = 500kbps */
    /* Prescaler = 1, BS1 = 13tq, BS2 = 2tq, SJW = 1tq */
    /* Baudrate = 8MHz / (Prescaler * (1 + BS1 + BS2)) = 8MHz / (1 * 16) = 500kbps */
	// changé
    /* Bit timing for 500 kbps @ APB1 = 42 MHz */
    /* Formula: 42MHz / (Prescaler * (1 + BS1 + BS2)) = 500kHz */
    /* 42MHz / (6 * 14) = 500kHz */
    CAN_InitStruct.CAN_SJW = CAN_SJW_1tq; // Synchronization jump width  SJW = 1 TQ
    CAN_InitStruct.CAN_BS1 = CAN_BS1_10tq;
    CAN_InitStruct.CAN_BS2 = CAN_BS2_5tq;
    CAN_InitStruct.CAN_Prescaler = 2;
    
    /* Initialisation CAN */
    if(CAN_Init(CAN1, &CAN_InitStruct) != CAN_InitStatus_Success)
    {
        /* Erreur d'initialisation - allumer LED rouge */
			  
        GPIOD->ODR |= (1 << 14);  // LED rouge PD14
			  USART3_send_string("CAN1 Init Failed!\r\n");
        while(1);
    }
		else
    {
        USART3_send_string("CAN1 Initialized OK\r\n");
    }
    
    /* Configuration du filtre pour recevoir les requêtes */
    CAN_FilterStruct.CAN_FilterNumber = 18; // Filter number
    CAN_FilterStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterStruct.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterStruct.CAN_FilterIdHigh = (CAN_ID_TEMP_REQUEST << 5) ;
    CAN_FilterStruct.CAN_FilterIdLow = 0;
    CAN_FilterStruct.CAN_FilterMaskIdHigh = (0x7FF << 5);  // Masque pour ID exact
    CAN_FilterStruct.CAN_FilterMaskIdLow = 0;
    CAN_FilterStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterStruct.CAN_FilterActivation = ENABLE;
    CAN_SlaveStartBank(20);
    CAN_FilterInit(&CAN_FilterStruct);
    
    /* Activation interruption FIFO0 message pending */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
		USART3_send_string("CAN1 Filter and Interrupts Configured\r\n");
}

/* ================== Configuration USART2 pour debug ================== */
/*void USART3_init(void)
{
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    
    // Configure PD8 (TX) and PD9 (RX) for USART3
    GPIOD->MODER &= ~((3 << (8*2)) | (3 << (9*2)));
    GPIOD->MODER |=  ((2 << (8*2)) | (2 << (9*2)));  // AF

    GPIOD->AFR[1] &= ~((0xF << 0) | (0xF << 4));
    GPIOD->AFR[1] |=  ((7 << 0) | (7 << 4));         // AF7 USART3
    
    // Configure USART2
    USART3->BRR = 115200;//0x1117 ; // 9600 bps @ APB1 = 42 MHz  //0x341 9600 @ 8MHz
    USART3->CR1 = USART_CR1_TE | USART_CR1_UE;
}*/
void USART3_init(void)
{
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    
    // Configure PD8 (TX) and PD9 (RX) for USART3
    GPIOD->MODER &= ~((3 << (8*2)) | (3 << (9*2)));
    GPIOD->MODER |=  ((2 << (8*2)) | (2 << (9*2)));  // AF mode
    
    // Push-pull output type
    GPIOD->OTYPER &= ~((1 << 8) | (1 << 9));
    
    // High speed
    GPIOD->OSPEEDR |= ((3 << (8*2)) | (3 << (9*2)));
    
    // No pull-up/pull-down
    GPIOD->PUPDR &= ~((3 << (8*2)) | (3 << (9*2)));

    // AF7 for USART3
    GPIOD->AFR[1] &= ~((0xF << 0) | (0xF << 4));
    GPIOD->AFR[1] |=  ((7 << 0) | (7 << 4));
    
    // Configure USART3: 115200 bps @ APB1 = 42 MHz
    // BRR = 42000000 / 115200 = 364.58 ˜ 364 (0x16C)
    USART3->BRR = 364;
    
    // Enable TX, RX and USART
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    
    // Petit délai pour stabilisation
    delay_ms(10);
}

/* ================== Envoi d'une chaîne via USART2 ================== */
void USART3_send_string(char* str)
{
    while(*str) {
        while(!(USART3->SR & USART_SR_TXE));
        USART3->DR = *str++;
    }
}

/* ================== Délai en millisecondes ================== */
void delay_ms(uint32_t ms)
{
	  /* Approximation: 168 MHz / 4 cycles per loop = 42M loops/sec */
    volatile uint32_t i;
    for(i = 0; i < ms * 42000; i++) { }
}

/* ========== DS1621 Functions ========== */

/**
  *   Initialisation du DS1621
  */
void DS1621_Init(void) 
{
    // Send Access Config command
    I2C1->CR1 |= I2C_CR1_START; 
    while (!(I2C1->SR1 & I2C_SR1_SB));  
    I2C1->DR = DS1621_WRITE_ADDR;  
    while (!(I2C1->SR1 & I2C_SR1_ADDR)); 
    (void)I2C1->SR1;  
    (void)I2C1->SR2;

    while (!(I2C1->SR1 & I2C_SR1_TXE));  
    I2C1->DR = DS1621_ACCESS_CONFIG; 
    
    while (!(I2C1->SR1 & I2C_SR1_TXE));  
    I2C1->DR = 0x00;  // Set continuous conversion mode

    I2C1->CR1 |= I2C_CR1_STOP;
    while(I2C1->CR1 & I2C_CR1_STOP);
    
    delay_ms(10);
    
    // Start conversion
    Start_DS1621_Conv();
}

/**
  *   Envoi d'une commande I2C au DS1621
  */
void I2C_COMMAND(int command)
{
    I2C1->CR1 |= I2C_CR1_START; 
    while (!(I2C1->SR1 & I2C_SR1_SB));  
    I2C1->DR = DS1621_WRITE_ADDR;  
    while (!(I2C1->SR1 & I2C_SR1_ADDR)); 
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    while (!(I2C1->SR1 & I2C_SR1_TXE));  
    I2C1->DR = command; 

    I2C1->CR1 |= I2C_CR1_STOP;
    while(I2C1->CR1 & I2C_CR1_STOP);
}

/**
  *   Démarrage de la conversion DS1621
  */
void Start_DS1621_Conv(void) 
{
    I2C_COMMAND(DS1621_START_CONVERSION); 
    delay_ms(10);
}

/*
  *  Lecture de la température du DS1621
  *  Température en °C
 */
float I2C_ReadTemperature(void) 
{
    int Msb, Lsb;
    float Temp;

    I2C_COMMAND(DS1621_READ_TEMP);  
    delay_ms(5);
    
    I2C1->CR1 |= I2C_CR1_START;  
    while (!(I2C1->SR1 & I2C_SR1_SB));  

    I2C1->DR = DS1621_READ_ADDR;  
    while (!(I2C1->SR1 & I2C_SR1_ADDR));  
    (void)I2C1->SR1;  
    (void)I2C1->SR2;

    I2C1->CR1 |= I2C_CR1_ACK; 
    while (!(I2C1->SR1 & I2C_SR1_RXNE));  
    Msb = I2C1->DR;  

    I2C1->CR1 &= ~I2C_CR1_ACK; 
    I2C1->CR1 |= I2C_CR1_STOP;
    while (!(I2C1->SR1 & I2C_SR1_RXNE));  
    Lsb = I2C1->DR; 

    if (Msb & 0x80) {  // Negative temperature
        Temp = Msb - 256;
    } else {            // Positive temperature
        Temp = Msb;
    }
    
    // Add 0.5 degrees if bit 7 of LSB is set
    if (Lsb & 0x80) {
        Temp += 0.5;
    }
    
    while(I2C1->CR1 & I2C_CR1_STOP);
    
    return Temp;
}

/* ========== CAN Functions ========== */

/*
  *   Envoi de la température via CAN
*/
void CAN_Send_Temperature(float temp)
{
    CanTxMsg TxMessage;
    uint8_t mailbox;
    int16_t temp_fixed;
    char debug_str[50];
    
    /* Préparation du message CAN */
    TxMessage.StdId = CAN_ID_TEMP_RESPONSE;
    TxMessage.ExtId = 0; // No extended ID
    TxMessage.IDE = CAN_Id_Standard; // Use standard ID
    TxMessage.RTR = CAN_RTR_Data; // Data frame
    TxMessage.DLC = 4;
    
    /* Conversion température en format fixe (x10 pour 1 décimale) */
    temp_fixed = (uint8_t)(temp * 10);
    
    /* Remplissage des données */
    TxMessage.Data[0] = (CAN_ID_TEMP_RESPONSE  & 0xFF); // LSB  0x81
	  TxMessage.Data[1] = (CAN_ID_TEMP_RESPONSE >> 8) & 0xFF; // MSB  0x06
    TxMessage.Data[2] = (temp_fixed >> 8)& 0xFF;   // MSB
	  TxMessage.Data[3] = (temp_fixed & 0xFF); // LSB


    /* Transmission */
    mailbox = CAN_Transmit(CAN1, &TxMessage); // Transmit the CAN message
    
    /* Attendre la fin de transmission */
    uint32_t timeout = 100000;
    while(CAN_TransmitStatus(CAN1, mailbox) == CAN_TxStatus_Pending && timeout > 0)
    {
        timeout--;
    }
    
    if(CAN_TransmitStatus(CAN1, mailbox) == CAN_TxStatus_Ok)
    {
        /* Toggle LED verte = transmission OK */
        GPIOD->ODR ^= (1 << 12);
        
        /* Debug UART */
        sprintf(debug_str, "CAN TX: %.1f C \r\n", temp);
        USART3_send_string(debug_str);
    }
    else
    {
        /* Allumer LED orange = erreur transmission */
        GPIOD->ODR |= (1 << 13);
        USART3_send_string("CAN TX Failed!\r\n");
    }
}


/* ========== Main Function ========== */

/*
  Programme principal
*/
int main(void)
{
    char temp_str[50];
    
    /* Initialisation horloge HSE 8MHz */
    //hse_clk();
    SystemClock_Config();
    /* Configuration GPIO */
    GPIO_Config();
    
    /* Initialisation I2C1 */
    config_I2C1();
    
    /* Initialisation USART2 pour debug */
    USART3_init();
    USART3_send_string("\r\n=== System Starting ===\r\n");
    
    /* Initialisation DS1621 */
    USART3_send_string("Initializing DS1621...\r\n");
    DS1621_Init();
    USART3_send_string("DS1621 OK\r\n");
    
    /* Attendre première conversion (750ms) */
    delay_ms(800);
    
    /* Initialisation CAN1 */
    USART3_send_string("Initializing CAN1...\r\n");
    CAN_Config();
    USART3_send_string("CAN1 OK\r\n");
    
    /* Activation interruptions globales */
    __enable_irq();
    
    USART3_send_string("=== System Ready ===\r\n");
    
    /* Boucle principale */
    while(1) 
    {
     
				/* Lecture manuelle du FIFO0 */
			
  if((CAN1->RF0R & CAN_RF0R_FMP0) != 0)
    {
    uint32_t id = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;
    uint8_t dlc = CAN1->sFIFOMailBox[0].RDTR & 0x0F;
    
    char msg[50];
    sprintf(msg, "Direct read - ID: 0x%03X\r\n", (unsigned int)id);
    USART3_send_string(msg);
    
    /* Libérer le FIFO manuellement */
    CAN1->RF0R |= CAN_RF0R_RFOM0;
    
    /* Traiter si c'est 0x550 */
    if(id == CAN_ID_TEMP_REQUEST) {
        current_temperature = I2C_ReadTemperature();
        CAN_Send_Temperature(current_temperature);
    }
}
        
    }
}