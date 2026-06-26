#include "stm32f411xe.h"

/*
 * Segment patterns are the masks for turn on each number (0-9) in each port (A,B and C)
 */

// Outputs pins used in PA: PA10:
const uint16_t segment_pattern_a[10] = { 0x0400, // 0
		0x0400, // 1
		0x0400, // 2
		0x0400, // 3
		0x0400, // 4
		0x0000, // 5
		0x0000, // 6
		0x0400, // 7
		0x0400, // 8
		0x0400  // 9
		};

// Outputs pins used in PB: PB7 and PB8:
const uint16_t segment_pattern_b[10] = { 0x0100, // 0
		0x0000, // 1
		0x0080, // 2
		0x0080, // 3
		0x0180, // 4
		0x0180, // 5
		0x0180, // 6
		0x0000, // 7
		0x0180, // 8
		0x0180  // 9
		};

// Outputs pins used in PC: PC2, PC9, PC11 and PC12:
const uint16_t segment_pattern_c[10] = { 0x1A04, // 0
		0x0004, // 1
		0x1A00, // 2
		0x0A04, // 3
		0x0004, // 4
		0x0A04, // 5
		0x1A04, // 6
		0x0204, // 7
		0x1A04, // 8
		0x0A04  // 9
		};

// Arrays for the pins map on each port for segments and PC for digits:
const uint8_t segment_shift_map_a[1] = { 10 };           // Segments in PA: PA10
const uint8_t segment_shift_map_b[2] = { 8, 7 };     // Segments in PB: PB7, PB8
const uint8_t segment_shift_map_c[4] = { 2, 9, 11, 12 }; // Segments in PC: PC2, PC9, PC11, PC12
const uint8_t digit_shift_map_c[4] = { 6, 10, 4, 3 }; // Digits in PC: PC3, PC4, PC6, PC10

// Array that contain the number in each digit. {Thousands, Hundreds, Tens, Units}
volatile uint8_t display_buffer[4] = { 0, 0, 0, 0 };

/*
 * Masks for turn on and turn off each segment and digit in the timer ISR.
 * For 7 segments in 4 digits exists 28 states.
 */

#define MULTIPLEX_STATES 28
volatile uint16_t portA_mask_buffer[MULTIPLEX_STATES];
volatile uint16_t portB_mask_buffer[MULTIPLEX_STATES];
volatile uint16_t portC_mask_buffer[MULTIPLEX_STATES];
volatile uint16_t portC_mask_digits[MULTIPLEX_STATES];

volatile int32_t counter = 0;   // Counter
int32_t value = 0;              // Value in display


// Timestamps to avoid flanks that are too close due to rebounds:
volatile uint32_t global_time = 0;
volatile uint32_t pb1_time = 0;
volatile uint32_t pb12_time = 0;
#define DEBOUNCE_MS  50U

volatile uint8_t isr_state_index = 0;

/*
 * Function to setup the required GPIOs:
 * Digits: PC3, PC4, PC6, PC10
 * Segments: PA10, PB7, PB8, PC2, PC9, PC11, PC12
 * Blinky: PH1
 */
void init_gpio(void) {

	/*
	 * Enable clocks for required ports: A, B, C and H.
	 * Port H is for blinky.
	 */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
			| RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOHEN;

	__asm("nop");

	// Setup PB1 and PB12 as inputs (photointerrupters):
	GPIOB->MODER &= ~(GPIO_MODER_MODE1 | GPIO_MODER_MODE12);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD12);

	// Setup PC3, PC4, PC6 and PC10 as outputs (digits):
	GPIOC->MODER &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE6
			| GPIO_MODER_MODE10);
	GPIOC->MODER |= (GPIO_MODER_MODE3_0 | GPIO_MODER_MODE4_0
			| GPIO_MODER_MODE6_0 | GPIO_MODER_MODE10_0);

	// Configured in high to keep them off:
	GPIOC->ODR |= (GPIO_ODR_OD3 | GPIO_ODR_OD4 | GPIO_ODR_OD6 | GPIO_ODR_OD10);

	// Setup pins for segments:
	GPIOA->MODER &= ~(GPIO_MODER_MODE10);
	GPIOB->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE8);
	GPIOC->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE9 | GPIO_MODER_MODE11
			| GPIO_MODER_MODE12);

	GPIOA->MODER |= (GPIO_MODER_MODE10_0);
	GPIOB->MODER |= (GPIO_MODER_MODE7_0 | GPIO_MODER_MODE8_0);
	GPIOC->MODER |= (GPIO_MODER_MODE2_0 | GPIO_MODER_MODE9_0
			| GPIO_MODER_MODE11_0 | GPIO_MODER_MODE12_0);

	/*
	 *  An 8 is writed to blanked the display for initial state.
	 *  High state is established as turn off.
	 */
	GPIOA->ODR |= segment_pattern_a[8];
	GPIOB->ODR |= segment_pattern_b[8];
	GPIOC->ODR |= segment_pattern_c[8];

	/*
	 * Test start:
	 GPIOC->ODR &= ~(GPIO_ODR_OD3);
	 GPIOA->ODR &= ~segment_pattern_a[9];
	 GPIOB->ODR &= ~segment_pattern_b[9];
	 GPIOC->ODR &= ~segment_pattern_c[9];
	 */

	// Blinky settings (PH1):
	GPIOH->MODER &= ~GPIO_MODER_MODE1;
	GPIOH->MODER |= GPIO_MODER_MODE1_0;
	GPIOH->ODR &= ~GPIO_ODR_OD1;

}


/*
 *  Funtion to setup TIM10
 *  that control the dispñay
 */
void init_tim10(void) {

	//Enable clock for TIM10
	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;

	// Prescaler and Auto-Reload 16MHz/16 = 1 MHz -> 1 \mu s * 100 = 100 \mu s
	TIM10->PSC = 16 - 1;
	TIM10->ARR = 100 - 1;

	// Update Interrupt Enable
	TIM10->DIER |= TIM_DIER_UIE;

	// Enable the interrupt of TIM10 in NVIC (Nested Vectored Interrupt Controller)
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	// Counter Enable: Start counting
	TIM10->CR1 |= TIM_CR1_CEN;
}


/*
 * Function to setup TIM11
 * that control blinking led
 */
void init_tim11(void) {

	RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;

	// Prescaler and Auto-Reload: 16 MHz/16k = 1 kHz -> 1 ms * 500 = 500 ms
	TIM11->PSC = 16000 - 1;
	TIM11->ARR = 500 - 1;

	TIM11->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
	TIM11->CR1 |= TIM_CR1_CEN;
}


/*
 * Function to setup EXTI
 * EXTI1:  Rising flank
 * EXTI12: Falling flank
 */
void init_exti(void) {

	// Enable the SYSCFG peripheral clock:
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Mapping EXTI1 and EXTI12 to port B (PB1 and PB12):
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI1);
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PB;

	SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI12);
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI12_PB;

	// Enable rising flank and disable falling flank for EXTI1:
	EXTI->RTSR |= EXTI_RTSR_TR1;
	EXTI->FTSR &= ~EXTI_FTSR_TR1;

	// Enable falling flank and disable rising flank for EXTI12:
	EXTI->FTSR |= EXTI_FTSR_TR12;
	EXTI->RTSR &= ~EXTI_RTSR_TR12;

	EXTI->PR = (EXTI_PR_PR1 | EXTI_PR_PR12);

	EXTI->IMR |= (EXTI_IMR_IM1 | EXTI_IMR_IM12);

	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/*
 * Interruptions Services Routines
 */

void TIM1_UP_TIM10_IRQHandler(void) {

	/*
	 * Evaluation if UIF it is up.
	 * TIM_SR_UIF = 1<<0
	 */

	if (TIM10->SR & TIM_SR_UIF) {

		// UIF is lowered:
		TIM10->SR &= ~TIM_SR_UIF;

		// Blanking the display:
		GPIOC->ODR |= (0x458);
		GPIOA->ODR |= segment_pattern_a[8];
		GPIOB->ODR |= segment_pattern_b[8];
		GPIOC->ODR |= segment_pattern_c[8];

		/*
		 * In each port register the mask for the current state are writed.
		 * This mask are generate in while cycle.
		 */
		GPIOA->ODR &= ~portA_mask_buffer[isr_state_index];
		GPIOB->ODR &= ~portB_mask_buffer[isr_state_index];
		GPIOC->ODR &= ~portC_mask_buffer[isr_state_index];
		GPIOC->ODR &= ~portC_mask_digits[isr_state_index];

		// It moves to the next state:
		isr_state_index++;

		// The states counter are reinitialized
		if (isr_state_index >= 28) {
			isr_state_index = 0;
		}

		global_time++;
	}
}

void TIM1_TRG_COM_TIM11_IRQHandler(void) {

	if (TIM11->SR & TIM_SR_UIF) {
		TIM11->SR &= ~TIM_SR_UIF;
		GPIOH->ODR ^= GPIO_ODR_OD1;
	}
}

void EXTI1_IRQHandler(void) {

	// It is verified that the interruption flag is up
	if (EXTI->PR & EXTI_PR_PR1) {

		// The interruption flag is lowered:
		EXTI->PR = EXTI_PR_PR1;

		// It is verified that the flank is not due to rebounds:
		if ((global_time - pb1_time) > DEBOUNCE_MS) {
			counter++;             // Counter is increases.
			pb1_time = global_time;
		}
	}
}

void EXTI15_10_IRQHandler(void) {

	if (EXTI->PR & EXTI_PR_PR12) {
		EXTI->PR = EXTI_PR_PR12;
		if ((global_time - pb12_time) > DEBOUNCE_MS) {
			counter--;
			pb12_time = global_time;
		}
	}
}


/*
 *  Main function
 */
int main(void) {

	init_gpio();
	init_tim10();
	init_tim11();
	init_exti();

	while (1) {

		// Value in display get the value of counter.
		value = counter;

		// Limits of display:
		if (value < 0)
			value += 10000;
		if (value > 9999)
			value -= 10000;

		// Decomposition of the number into powers of ten:
		display_buffer[0] = value / 1000;                             // Thousands.
		display_buffer[1] = (value - display_buffer[0] * 1000) / 100; // Hundreds.
		display_buffer[2] = (value - display_buffer[0] * 1000
				- display_buffer[1] * 100) / 10;                       // Tens.
		display_buffer[3] = (value - display_buffer[0] * 1000
				- display_buffer[1] * 100) % 10;                       // Units.

		uint8_t state_idx = 0;

		// Iteration in the four digits:
		for (uint8_t digit = 0; digit < 4; digit++) {

			// Extraction of the corresponding digit:
			uint8_t number = display_buffer[digit];

			// Obtain the pattern for the corresponding number in each port:
			uint16_t pattern_a = segment_pattern_a[number];
			uint16_t pattern_b = segment_pattern_b[number];
			uint16_t pattern_c = segment_pattern_c[number];

			// Obtain position of bit of digit in port C ODR:
			uint8_t shift_digit = digit_shift_map_c[digit];

			// Defined the mask for the digit:
			uint16_t digit_mask = (1 << shift_digit);

			// Iteration in the segments connected in port C:
			for (uint8_t seg = 0; seg < 4; seg++) {

				// Obtain position of bit of segment in port C ODR:
				uint8_t shift = segment_shift_map_c[seg];

				// Boolean to determinate if the segment must be turn on:
				if (pattern_c & (1 << shift)) {

					// 1 << shift is writed on segment masks array:
					portC_mask_buffer[state_idx] = (1 << shift);

					// 1 << shift_digit is writed on digit masks array to turn on the anode:
					portC_mask_digits[state_idx] = digit_mask;

				} else {

					portC_mask_buffer[state_idx] = 0;
					portC_mask_digits[state_idx] = 0;

				}

				// Anothers ports are established in zero:
				portA_mask_buffer[state_idx] = 0;
				portB_mask_buffer[state_idx] = 0;

				state_idx++;
			}

			// Repeat the process for the segment in pin PA10:
			uint8_t shift_a = segment_shift_map_a[0];
			if (pattern_a & (1 << shift_a)) {
				portA_mask_buffer[state_idx] = (1 << shift_a);
				portC_mask_digits[state_idx] = digit_mask;

			} else {
				portA_mask_buffer[state_idx] = 0;
				portC_mask_digits[state_idx] = 0;
			}
			portB_mask_buffer[state_idx] = 0;
			portC_mask_buffer[state_idx] = 0;
			state_idx++;

			// Repeat the process for the segments in PB7 and PB8:
			for (uint8_t seg = 0; seg < 2; seg++) {
				uint8_t shift = segment_shift_map_b[seg];
				if (pattern_b & (1 << shift)) {
					portB_mask_buffer[state_idx] = (1 << shift);
					portC_mask_digits[state_idx] = digit_mask;
				} else {
					portB_mask_buffer[state_idx] = 0;
					portC_mask_digits[state_idx] = 0;
				}
				portA_mask_buffer[state_idx] = 0;
				portC_mask_buffer[state_idx] = 0;
				state_idx++;
			}
		}
	}
}
