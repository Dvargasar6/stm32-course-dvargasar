#include <stdint.h>
#include <stm32f4xx.h>

int main(void)
{
    // Habilita el reloj del puerto GPIOA en el bus AHB1.
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Limpia los bits MODER5 (pone 00 = entrada).
    GPIOA->MODER &= ~GPIO_MODER_MODER5;
    // Configura PA5 como salida de proposito general (01).
    // Faltaba el sufijo _0 y el punto y coma en tu codigo original.
    GPIOA->MODER |= GPIO_MODER_MODER5_0;

    // Salida tipo push-pull (es el valor por defecto, pero lo dejamos explicito).
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;

    // Velocidad media (10).
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED5;
    GPIOA->OSPEEDR |= (0b10 << GPIO_OSPEEDR_OSPEED5_Pos);

    // Enciende el LED poniendo a 1 el bit 5 del registro ODR.
    GPIOA->ODR |= GPIO_ODR_OD5;

    // Bucle infinito vacio: el pin conserva su estado indefinidamente.
    while (1) {
    }

    return 0;
}
