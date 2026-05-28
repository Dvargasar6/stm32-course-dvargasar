#include <stdint.h>
#include <stm32f4xx.h>

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

    while (1) {
    	GPIOA->ODR |= (1 << 5);                          // LED on
    	for(uint32_t i = 0; i < 2000000; i++){}           // wait
    	GPIOA->ODR &= ~(1 << 5);                         // LED off
    	for(uint32_t i = 0; i < 1000000; i++){}           // wait
    }

    return 0;
}
