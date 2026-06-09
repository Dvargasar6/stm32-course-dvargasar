#include "stm32f411xe.h" // Device-specific header containing register definitions

// Contador principal:
volatile int32_t contador = 0;

// Tick global incrementado por SysTick cada 1 ms. Sirve como reloj
// de baja resolucion para el debounce por software.
volatile uint32_t tick_ms = 0;

// Marca de tiempo del ultimo flanco aceptado en PC0.
volatile uint32_t ultimo_flanco_pc0_ms = 0;

#define DEBOUNCE_MS  50U

const uint16_t segment_pattern[10] = { 0x00F3, // 0
		0x0012, // 1
		0x0163, // 2
		0x0133, // 3
		0x0192, // 4
		0x01B1, // 5
		0x01F1, // 6
		0x0013, // 7
		0x01F3, // 8
		0x01B3  // 9
		};

const uint8_t segment_shift_map[7] = { 0, 1, 4, 5, 6, 7, 8 };

// Buffer holding the numbers to display.
volatile uint8_t display_buffer[2] = { 4, 4 };

/*
 * Pre-calculated schedules for the hardware state.
 * There are 14 discrete states required to multiplex 2 digits by 7 segments.
 * These arrays will hold the exact bitmasks the ISR needs to apply.
 */
volatile uint16_t portA_mask_buffer[14];
volatile uint16_t portB_mask_buffer[14];
volatile uint8_t isr_state_index = 0;

void init_gpio(void) {
	// Habilitar reloj para el puerto C
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	// Configurar puerto PC0 como entrada
	GPIOC->MODER &= ~(GPIO_MODER_MODE0);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD0);

	// Configurar puerto PC1 como entrada
	GPIOC->MODER &= ~(GPIO_MODER_MODE1);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD1);

	RCC->AHB1ENR |= (1 << 0) | (1 << 1);

	// Configure PA0, PA1, PA4-PA8 as General Purpose Outputs (01)
	GPIOA->MODER &=
			~((3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (4 * 2)) | (3 << (5 * 2))
					| (3 << (6 * 2)) | (3 << (7 * 2)) | (3 << (8 * 2)));
	GPIOA->MODER |=
			((1 << (0 * 2)) | (1 << (1 * 2)) | (1 << (4 * 2)) | (1 << (5 * 2))
					| (1 << (6 * 2)) | (1 << (7 * 2)) | (1 << (8 * 2)));

	// Configure PB0 and PB1 as General Purpose Outputs (01)
	GPIOB->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)));
	GPIOB->MODER |= ((1 << (0 * 2)) | (1 << (1 * 2)));

	// Initialize display to completely blank state
	GPIOA->ODR |= 0x01F3;
	GPIOB->ODR &= ~(0x03);

}

void init_tim(void) {
	// Habilitar reloj del timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;

	// Prescaler: 16 MHz/PSC
	TIM10->PSC = 16 - 1;

	// Autoreload: Reinicio del contador
	TIM10->ARR = 1000 - 1;

	// Enable the Update Interrupt in the Timer 10 DMA/Interrupt Enable Register
	TIM10->DIER |= TIM_DIER_UIE;

	// Enable Timer 10 in the Nested Vectored Interrupt Controller
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	// Start Timer 10 by setting the Counter Enable bit
	TIM10->CR1 |= TIM_CR1_CEN;
}

void init_exti(void) {
	// Encender senal de reloj para SYSCFG
	// No esta en el diagrama
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Configurar el canal del EXTI0, en este caso para el puerto C
	// [0] para canal 1
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0);
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC;

	// Configurar el canal para EXTI1
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI1);
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PC;

	// Detecta flancos de subida
	EXTI->RTSR |= EXTI_RTSR_TR0;

	// Detecta flancos de bajada
	EXTI->FTSR |= EXTI_RTSR_TR1;

	// Se registra la interrupcion en el NVIC
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);

	// Bajamos bandera de interrupcion
	EXTI->PR |= EXTI_PR_PR0;
	EXTI->PR |= EXTI_PR_PR1;

	// Activar interrupcion
	EXTI->IMR |= EXTI_IMR_IM0;
	EXTI->IMR |= EXTI_IMR_IM1;
}

void EXTI0_IRQHandler(void) {
	/*
	 * Verifica que la solicitud pendiente provenga efectivamente de la
	 * linea 0. El registro PR (Pending Register) tiene un bit por linea
	 * que se pone en 1 cuando hay flanco detectado.
	 */
	if (EXTI->PR & (1 << 0)) {
		/*
		 * Limpia el flag escribiendo 1 al bit (semantica "rw1c":
		 * read/write 1 to clear). Si no se limpia, el NVIC vuelve a
		 * entrar a esta ISR indefinidamente.
		 */
		EXTI->PR = (1 << 0);

		/*
		 * Debounce por software: ignora cualquier flanco que ocurra
		 * dentro de los 50 ms posteriores al ultimo aceptado. Esto
		 * evita conteos multiples por el ruido en la transicion del
		 * fototransistor al atravesar el objeto la ranura.
		 */
		if ((tick_ms - ultimo_flanco_pc0_ms) > 50) {
			contador++;
			ultimo_flanco_pc0_ms = tick_ms;
		}
	}
}

void EXTI1_IRQHandler(void) {
	/*
	 * Verifica que la solicitud pendiente provenga efectivamente de la
	 * linea 0. El registro PR (Pending Register) tiene un bit por linea
	 * que se pone en 1 cuando hay flanco detectado.
	 */
	if (EXTI->PR & (1 << 1)) {
		/*
		 * Limpia el flag escribiendo 1 al bit (semantica "rw1c":
		 * read/write 1 to clear). Si no se limpia, el NVIC vuelve a
		 * entrar a esta ISR indefinidamente.
		 */
		EXTI->PR = (1 << 1);

		/*
		 * Debounce por software: ignora cualquier flanco que ocurra
		 * dentro de los 50 ms posteriores al ultimo aceptado. Esto
		 * evita conteos multiples por el ruido en la transicion del
		 * fototransistor al atravesar el objeto la ranura.
		 */
		if ((tick_ms - ultimo_flanco_pc0_ms) > 50) {
			contador--;
			ultimo_flanco_pc0_ms = tick_ms;
		}
	}
}

void TIM1_UP_TIM10_IRQHandler(void) {
	/*
	 * Verifica que la causa de la interrupcion sea efectivamente el
	 * evento de update. El flag UIF (Update Interrupt Flag) esta en
	 * el registro SR (Status Register).
	 */
	if (TIM10->SR & TIM_SR_UIF) {
		/*
		 * Limpia el flag escribiendo 0 en el bit UIF. A diferencia de
		 * EXTI->PR (que se limpia escribiendo 1), los flags de los
		 * timers se limpian escribiendo 0 directamente.
		 */
		TIM10->SR &= ~TIM_SR_UIF;

		// 1. Hardware Blanking
		GPIOB->ODR &= ~(0x03);
		GPIOA->ODR |= 0x01F3;

		// 2. Fetch and apply the pre-calculated mask for the current time slice
		// The mask already contains 0 if the segment is meant to be off.
		GPIOA->ODR &= ~portA_mask_buffer[isr_state_index];
		GPIOB->ODR |= portB_mask_buffer[isr_state_index];

		// 3. Advance the schedule index
		isr_state_index++;
		if (isr_state_index >= 14) {
			isr_state_index = 0;
		}

		/*
		 * Incrementa el contador global de milisegundos. En 32 bits
		 * tarda mas de 49 dias en desbordarse, lo cual es irrelevante
		 * para la logica del debounce (que solo compara diferencias).
		 */
		tick_ms++;
	}
}

int main(void) {

	init_gpio();
	init_tim();
	init_exti();

	while (1) {
		// ... codigo existente del multiplexado del display ...
		int32_t valor = contador;
		if (valor < 0)  valor = 0;     // saturacion inferior (ver nota abajo)
		if (valor > 99) valor = 99;    // saturacion superior: limite fisico del display
		display_buffer[0] = valor / 10; // decenas
		display_buffer[1] = valor % 10; // unidades

		uint8_t state_idx = 0; // Tracks the 0-13 index for the linear buffers

		// Iterate through both digits
		for (uint8_t digit = 0; digit < 2; digit++) {
			uint8_t number = display_buffer[digit];
			uint16_t pattern = segment_pattern[number];

			// Iterate through all 7 segments for the current digit
			for (uint8_t seg = 0; seg < 7; seg++) {
				uint8_t shift = segment_shift_map[seg];

				// Determine if this specific segment requires illumination
				if (pattern & (1 << shift)) {
					// Populate buffer with the exact bit required to pull Port A LOW
					portA_mask_buffer[state_idx] = (1 << shift);
					// Populate buffer with the exact bit required to pull Port B HIGH
					portB_mask_buffer[state_idx] = (1 << digit);
				} else {
					// Populate buffer with 0, meaning no hardware change for this state
					portA_mask_buffer[state_idx] = 0;
					portB_mask_buffer[state_idx] = 0;
				}

				state_idx++; // Advance the buffer index
			}
		}
	}
}
