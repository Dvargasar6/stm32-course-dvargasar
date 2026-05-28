#include <stdint.h>
#include <stm32f4xx.h>

void TIM3_IRQHandler(void);

int main(void)
{
    // Habilita el reloj del puerto GPIOA en el bus AHB1.
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~(GPIO_MODER_MODE5);
    GPIOA->MODER |= GPIO_MODER_MODE5_0;

    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR5);
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5_1;

    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5);
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD5_0;

    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM3EN);
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->ARR = 249;
    TIM3->PSC = 15999;
    TIM3->CNT = 0;

    TIM3->CR1 &= ~(TIM_CR1_ARPE);
    TIM3->CR1 |= TIM_CR1_ARPE;

    TIM3->CR1 &= ~(TIM_CR1_DIR);


    //Activamos la IRQ del TIM3.
    __NVIC_EnableIRQ(TIM3_IRQn);

    //Bandera de interrupcion del TIM3


    TIM3->SR &= ~(TIM_SR_UIF);
    //TIM3->SR |= TIM_SR_UIF;

    TIM3->DIER &= ~(TIM_DIER_UIE);
    TIM3->DIER |= TIM_DIER_UIE;

    //TIM3->CR1 &= ~(TIM_CR1_CEN);
    TIM3->CR1 |= TIM_CR1_CEN;




    while (1) {
    	         // wait
    }

    return 0;
}

void TIM3_IRQHandler(void){
	if(TIM3->SR && TIM_SR_UIF){

		GPIOA->ODR ^= GPIO_ODR_OD5;

		TIM3->SR &= ~(TIM_SR_UIF);

	}
}
