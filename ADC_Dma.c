
//#include "stm32f4xx.h"
#include "ADC_Dma.h"


uint16_t ADC_VALUE[3]; // Stockage des valeurs ADC 


void config_EXTI(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable the SYSCFG clock
		SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; // Select PA0 for EXTI0
    EXTI->IMR |= EXTI_IMR_IM0;            // Enable interrupt mask pour EXTI0	permet au signal d'interruption sur EXTI0 d’atteindre le NVIC											
    EXTI->RTSR |= EXTI_RTSR_TR0;					// rising edge en ligne 0
    NVIC_EnableIRQ(EXTI0_IRQn);  					// Enable EXTI0 interrupt in NVIC
}

void config_TIM2(void)
{

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Activation de l'horloge Timer2

    TIM2->PSC = 15999; //1 ms (16 MHz ÷ 16000 = 1 kHz)
    TIM2->ARR = 29999; // Déclenchement toutes  15 secondes (15000 × 1ms = 15s)
    TIM2->CR2 |= TIM_CR2_MMS_1; //Déclenchement de l'ADC dès que le compteur est activé. toutes les 15s, le timer envoie un signal TRGO. Master mode selection 010
	  TIM2->CR1 |= TIM_CR1_CEN; // Démarrage du Timer
	
}

void config_ADC1 (void)
{
    //  Activer l'horloge pour ADC1 et les GPIO nécessaires
    RCC->APB2ENR |= (1<<8);  // Activer l'horloge ADC1
    RCC->AHB1ENR |= (1<<0);  // Activer l'horloge GPIOA
   


    
     ADC1->CR1 = (1<<8);    // Mode SCAN activé
     ADC1->CR1 &= ~(1<<24); // Résolution 12 bits
	   ADC1->CR1 |= (1<<8)|(1<<5); //|(1<<11);		//configuration de l'ADC en mode scan ( mode multichannel) , et EOC INTERRUPT ENABLE =1 pour activer l'interruptioon a la finn de chaque conversion ( page 419) //discontinuous

   
    ADC1->CR2 |= (1<<10);   // EOC après chaque conversion
    

    //  Définir le temps d'échantillonnage pour chaque canal
    ADC1->SMPR2 &= ~ (7<<12)| (7<<15) | (7<<18);  // Sampling time de 3 cycles pour CH4 CH5 ET CH6 (000)

    //  Définir la longueur de la séquence de conversion (3 conversions)
    ADC1->SQR1 |= (2<<20);  // SQR1_L = 2 pour 3 conversions

    // PA4 PA5 PA6 CONFIGUREES EN MODE ANALOGIQUE moder: |11|11|11|
    GPIOA->MODER |= (3<<8);   // PA4 en mode analogique (CH4)
    GPIOA->MODER |= (3<<12);   // PA6 en mode analogique (CH6)
    GPIOA->MODER |= (3<<10);  // PA5 en mode analogique (CH5)

     // choix de l'ordre de la conversions des channels (mettre 4 dans SQ1 <=> la 1ere channel a convertir c ch4 )
    ADC1->SQR3 |= (4<<0);   // SEQ1 = Channel 4
    ADC1->SQR3 |= (5<<5);  // SEQ2 = Channel 5
    ADC1->SQR3 |= (6<<10); // SEQ3 = Channel 6

    
    ADC1->CR2 |= (1<<8);  // DMA activé
    ADC1->CR2 |= (1<<9);  // permettre au DMA de transférer les données de l'ADC vers la mémoire en continu, sans avoir à réactiver manuellement le DMA après chaque conversion

    // 10. Activer l'ADC
    ADC1->CR2 |= 1<<0; 

    // 11. Configurer le déclenchement par TIMER2
    ADC1->CR2 |= ADC_CR2_EXTEN_0 |  // Déclenchement sur front montant
                 (6 << ADC_CR2_EXTSEL_Pos); // ADC déclenché par le triggeur de TIM2 // 6=0110: Timer 2 TRGO event
	
}

void config_DMA2(void) {
   	// Enable the DMA2 Clock
	RCC->AHB1ENR |= (1<<22);  // DMA2EN = 1
	
	// Select the Data Direction
	DMA2_Stream0->CR &= ~(3<<6);  // Peripheral to memory
	

	DMA2_Stream0->CR |= (1<<8);  // CIRC = 1 
	
	DMA2_Stream0->CR |= (1<<10);  // MINC = 1; //// incrementation de la memire pour stocker la data suivante dans la case memoire suivante 
	DMA2_Stream0->CR |= (1<<11)|(1<<13);  // DR del'adc est de taille 16bit on met 01 ds Psize de mm pour MSIZE
	DMA2_Stream0->CR &= ~(7<<25);  //on lie ch0 ( correspondant au AD1) au stream 0
	DMA2_Stream0->NDTR = 3;   //3datas a envoyer par DMA
	
	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;  // Source address is peripheral address
	
	DMA2_Stream0->M0AR = (uint32_t) ADC_VALUE;  // Destination Address is memory address
		
    // Activation de l'interruption du DMA
    DMA2_Stream0->CR |= DMA_SxCR_TCIE;
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	// Enable the DMA Stream
	DMA2_Stream0->CR |= (1<<0);  // EN =1


		
		
}



int main(void)
{    
	config_EXTI();
	config_TIM2();
  config_ADC1();
	config_DMA2();
	
		
    while(1){
		
		}
			
}

void EXTI0_IRQHandler(void)
{	
    if (EXTI->PR & EXTI_PR_PR0) // Check if the interrupt is triggered by EXTI line 0
			{   	
        TIM2->CR1 |= TIM_CR1_CEN;  // Démarrer TIM2

				EXTI->PR |= EXTI_PR_PR0;		// Clear the interrupt pending bit for EXTI line 0

			}

}

void DMA2_Stream0_IRQHandler(void) 
	{
    if (DMA2->LISR & DMA_LISR_TCIF0) { // Vérifie si l'interruption est due à la fin du transfert
        DMA2->LIFCR = DMA_LIFCR_CTCIF0; // Efface le flag d'interruption


		
    }
		   // ADC1->CR2 |= ADC_CR2_SWSTART; // Redémarrer la conversion ADC
}