#include <stdint.h>
#include <stm32f4xx.h>

typedef enum {
    GREEN,
    YELLOW_RED,
    RED,
	YELLOW_GREEN
} TrafficState;

volatile TrafficState current_state = GREEN;
volatile uint32_t timer_count = 0;
uint8_t long_state = 5;
uint8_t short_state = 1;

int main(void)
{
    // Habilita el reloj del puerto GPIOA en el bus AHB1.
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Turn on registers of PA5:
    GPIOA->MODER &= ~(GPIO_MODER_MODE5);
    GPIOA->MODER |= GPIO_MODER_MODE5_0;
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR5);
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5_1;
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5);
    //GPIOA->PUPDR |= GPIO_PUPDR_PUPD5_0;

    // Turn on registers of PA6:
    GPIOA->MODER &= ~(GPIO_MODER_MODE6);
    GPIOA->MODER |= GPIO_MODER_MODE6_0;
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6);
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1;
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD6);

    // Turn on registers of PA7:
    GPIOA->MODER &= ~(GPIO_MODER_MODE7);
    GPIOA->MODER |= GPIO_MODER_MODE7_0;
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR7);
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1;
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD7);



    // Setup TIM3:
    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM3EN);
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->ARR = 999;
    TIM3->PSC = 15999;
    TIM3->CNT = 0;

    TIM3->CR1 &= ~(TIM_CR1_ARPE);
    TIM3->CR1 |= TIM_CR1_ARPE;

    TIM3->CR1 &= ~(TIM_CR1_DIR);

    //Activamos la IRQ del TIM3.
    __NVIC_EnableIRQ(TIM3_IRQn);

    //Bandera de interrupcion del TIM3
    TIM3->SR &= ~(TIM_SR_UIF);

    TIM3->DIER &= ~(TIM_DIER_UIE);
    TIM3->DIER |= TIM_DIER_UIE;

    TIM3->CR1 |= TIM_CR1_CEN;


    while (1) {
    	         // wait
    }

    return 0;
}

void TIM3_IRQHandler(void){
	if(TIM3->SR && TIM_SR_UIF){

		timer_count++;

		switch(current_state){
		// Turn off PA5 y PA6, turn on PA7:
		case GREEN:
			GPIOA->ODR &= ~(GPIO_ODR_OD5 | GPIO_ODR_OD6);
			GPIOA->ODR |= GPIO_ODR_OD7;

			if(timer_count >= long_state){
				timer_count = 0;
				current_state = YELLOW_RED;
			}
			break;

		case YELLOW_RED:
			GPIOA->ODR &= ~(GPIO_ODR_OD5 | GPIO_ODR_OD7);
			GPIOA->ODR |= GPIO_ODR_OD6;

			if(timer_count >= short_state){
				timer_count = 0;
				current_state = RED;
			}
			break;

		case RED:
			GPIOA->ODR &= ~(GPIO_ODR_OD6 | GPIO_ODR_OD7);
			GPIOA->ODR |= GPIO_ODR_OD5;

			if(timer_count >= long_state){
				timer_count = 0;
				current_state = YELLOW_GREEN;
			}
			break;

		case YELLOW_GREEN:
			GPIOA->ODR &= ~(GPIO_ODR_OD5 | GPIO_ODR_OD7);
			GPIOA->ODR |= GPIO_ODR_OD6;

			if(timer_count >= short_state){
				timer_count = 0;
				current_state = GREEN;
			}
			break;

		}

		TIM3->SR &= ~(TIM_SR_UIF);

	}
}
