#include "stm32f411xe.h" // Device-specific header containing register definitions

/*
 * Constant arrays for segment decoding.
 */
const uint16_t segment_pattern[10] = {
    0x00F3, // 0
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
const uint8_t segment_shift_map[7] = {0, 1, 4, 5, 6, 7, 8};

// Buffer holding the numbers to display.
volatile uint8_t display_buffer[2] = {4, 3};

/*
 * Pre-calculated schedules for the hardware state.
 * There are 14 discrete states required to multiplex 2 digits by 7 segments.
 * These arrays will hold the exact bitmasks the ISR needs to apply.
 */
volatile uint16_t portA_mask_buffer[14];
volatile uint16_t portB_mask_buffer[14];
volatile uint8_t isr_state_index = 0;

void GPIO_Init(void) {
    // Enable peripheral clocks for GPIOA and GPIOB
    RCC->AHB1ENR |= (1 << 0) | (1 << 1);

    // Configure PA0, PA1, PA4-PA8 as General Purpose Outputs (01)
    GPIOA->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (4 * 2)) |
                      (3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2)) | (3 << (8 * 2)));
    GPIOA->MODER |= ((1 << (0 * 2)) | (1 << (1 * 2)) | (1 << (4 * 2)) |
                     (1 << (5 * 2)) | (1 << (6 * 2)) | (1 << (7 * 2)) | (1 << (8 * 2)));

    // Configure PB0 and PB1 as General Purpose Outputs (01)
    GPIOB->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)));
    GPIOB->MODER |= ((1 << (0 * 2)) | (1 << (1 * 2)));

    // Initialize display to completely blank state
    GPIOA->ODR |= 0x01F3;
    GPIOB->ODR &= ~(0x03);
}

void TIM10_Init(void) {
    // Enable the peripheral clock for Timer 10 on the APB2 bus
    RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;

    /* * The default HSI clock is 16 MHz.
     * Set the Prescaler to divide the 16 MHz clock by 16000.
     * The timer will tick at 1000 Hz (1 millisecond per tick).
     */
    TIM10->PSC = 16000 - 1;

    /* * Set the Auto-Reload Register to 500.
     * 1000 Hz / 500 = 2 Hz.
     * The interrupt will now trigger exactly once every 500 milliseconds.
     */
    TIM10->ARR = 3 - 1;

    // Enable the Update Interrupt in the Timer 10 DMA/Interrupt Enable Register
    TIM10->DIER |= TIM_DIER_UIE;

    // Enable Timer 10 in the Nested Vectored Interrupt Controller
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    // Start Timer 10 by setting the Counter Enable bit
    TIM10->CR1 |= TIM_CR1_CEN;
}

/*
 * Refactored Interrupt Service Routine.
 * Processing logic is completely removed. It strictly applies pre-calculated data.
 */
void TIM1_UP_TIM10_IRQHandler(void) {
    if (TIM10->SR & TIM_SR_UIF) {
        TIM10->SR &= ~TIM_SR_UIF; // Clear interrupt flag

        // 1. Hardware Blanking
        GPIOB->ODR &= ~(0x03);
        GPIOA->ODR |= 0x01F3;

        // 2. Fetch and apply the pre-calculated mask for the current time slice
        // The mask already contains 0 if the segment is meant to be off.
        GPIOA->ODR &= ~portA_mask_buffer[isr_state_index];
        GPIOB->ODR |=  portB_mask_buffer[isr_state_index];

        // 3. Advance the schedule index
        isr_state_index++;
        if (isr_state_index >= 14) {
            isr_state_index = 0;
        }
    }
}

int main(void) {
    // Hardware configuration
    GPIO_Init();
    TIM10_Init();

    /*
     * The Main Execution Loop.
     * All decoding, logic extraction, and hardware mapping occurs here.
     * It continuously updates the schedule buffers used by the ISR.
     */
    while (1) {
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
