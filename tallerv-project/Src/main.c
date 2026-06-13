#include "stm32f411xe.h" // Archivo de cabecera con las definiciones de los registros

// Contador principal:
volatile int32_t counter = 0;
int32_t value = 0;

// Tick global incrementado por SysTick cada 1 ms.
volatile uint32_t tick_ms = 0;

// Marcas de tiempo independientes del ultimo flanco aceptado por cada canal.
// Mantenerlas separadas evita que un flanco en una linea inhiba el debounce
// de la otra, lo cual seria erroneo al tratarse de dos fotointerruptores
// fisicamente independientes.
volatile uint32_t ultimo_flanco_pb1_ms  = 0;
volatile uint32_t ultimo_flanco_pb12_ms = 0;

#define DEBOUNCE_MS  50U

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

const uint16_t segment_pattern_b[10] = { 0x0100, // 0
		0x0000, // 1
		0x0000, // 2
		0x0000, // 3
		0x0100, // 4
		0x0100, // 5
		0x0100, // 6
		0x0000, // 7
		0x0100, // 8
		0x0100  // 9
		};

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

const uint16_t segment_pattern_h[10] = { 0x0000, // 0
		0x0000, // 1
		0x0002, // 2
		0x0002, // 3
		0x0002, // 4
		0x0002, // 5
		0x0002, // 6
		0x0000, // 7
		0x0002, // 8
		0x0002  // 9
		};

const uint8_t segment_shift_map_a[1] = { 10 };
const uint8_t segment_shift_map_b[1] = { 8 };
const uint8_t segment_shift_map_c[4] = { 2, 9, 11, 12 };
const uint8_t segment_shift_map_h[1] = { 1 };
const uint8_t digit_shift_map_c[4] = { 6, 10, 4, 3 };

// Bufer para los numeros a mostrar en pantalla.
volatile uint8_t display_buffer[4] = { 1, 1, 4, 4 };

#define MULTIPLEX_STATES 28
volatile uint16_t portA_mask_buffer[MULTIPLEX_STATES];
volatile uint16_t portB_mask_buffer[MULTIPLEX_STATES];
volatile uint16_t portC_mask_buffer[MULTIPLEX_STATES];
volatile uint16_t portH_mask_buffer[MULTIPLEX_STATES];

volatile uint16_t portC_mask_digits[MULTIPLEX_STATES];

volatile uint8_t isr_state_index = 0;

void init_gpio(void) {
	// Habilitacion del reloj para los puertos A, B, C y H:
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
			| RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOHEN;

	// Pausa para estabilizacion de la senal de reloj
	__asm("nop");

	// Configuracion de PB1 y PB12 como entradas (Fotointerruptores):
	GPIOB->MODER &= ~(GPIO_MODER_MODE1 | GPIO_MODER_MODE12);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD12);

	// Configuracion de PC3, PC4, PC6 y PC10 como salidas para los anodos:
	GPIOC->MODER &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE6
			| GPIO_MODER_MODE10);
	GPIOC->MODER |= (GPIO_MODER_MODE3_0 | GPIO_MODER_MODE4_0
			| GPIO_MODER_MODE6_0 | GPIO_MODER_MODE10_0);

	// Se establece ALTO logico por defecto para mantener los digitos apagados
	// en caso de que los catodos esten en bajo (dependiendo de la logica inicial).
	GPIOC->ODR |= (GPIO_ODR_OD3 | GPIO_ODR_OD4 | GPIO_ODR_OD6 | GPIO_ODR_OD10);

	// Limpieza de bits para configuracion de los segmentos como salidas:
	GPIOA->MODER &= ~(GPIO_MODER_MODE10);
	GPIOB->MODER &= ~(GPIO_MODER_MODE8);
	GPIOC->MODER &= ~(GPIO_MODER_MODE2);
	GPIOC->MODER &= ~(GPIO_MODER_MODE9);
	GPIOC->MODER &= ~(GPIO_MODER_MODE11);
	GPIOC->MODER &= ~(GPIO_MODER_MODE12);
	GPIOH->MODER &= ~(GPIO_MODER_MODE1);

	// Aplicacion del modo de salida general (01 en MODER):
	GPIOA->MODER |= (GPIO_MODER_MODE10_0);
	GPIOB->MODER |= (GPIO_MODER_MODE8_0);
	GPIOC->MODER |= (GPIO_MODER_MODE2_0 | GPIO_MODER_MODE9_0
			| GPIO_MODER_MODE11_0 | GPIO_MODER_MODE12_0);
	GPIOH->MODER |= (GPIO_MODER_MODE1_0);

	// Apagado inicial de los segmentos (se asume catodo en ALTO como apagado).
	GPIOA->ODR |= segment_pattern_a[8];
	GPIOB->ODR |= segment_pattern_b[8];
	GPIOC->ODR |= segment_pattern_c[8];
	GPIOH->ODR |= segment_pattern_h[8];

	// Encendido estatico inicial de prueba (se sobreescribe luego por el barrido).
	GPIOC->ODR &= ~(GPIO_ODR_OD3);
	GPIOA->ODR &= ~segment_pattern_a[9];
	GPIOB->ODR &= ~segment_pattern_b[9];
	GPIOC->ODR &= ~segment_pattern_c[9];
	GPIOH->ODR &= ~segment_pattern_h[9];

	// --- Configuracion de PA5 (LED2 de la Nucleo-F411RE) como salida ---
	// Este pin sirve de heartbeat independiente del barrido del display.
	// Limpieza del campo MODER5 y seleccion del modo salida general (01).
	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |=  GPIO_MODER_MODE5_0;

	// Estado inicial apagado. El blink lo conmutara la ISR de TIM11.
	GPIOA->ODR &= ~GPIO_ODR_OD5;
}

void init_tim(void) {
	// Habilitacion del reloj para el temporizador TIM10
	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;

	// Pre-escalador: Reduce la frecuencia base del temporizador
	TIM10->PSC = 16 - 1;

	// Valor de auto-recarga: Determina el tope de conteo y la frecuencia de la interrupcion
	TIM10->ARR = 100 - 1;

	// Se activa la generacion de interrupciones al desbordarse el contador
	TIM10->DIER |= TIM_DIER_UIE;

	// Habilitacion vectorial en el NVIC (Controlador de interrupciones anidadas)
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	// Encendido final del temporizador
	TIM10->CR1 |= TIM_CR1_CEN;
}

void init_tim11(void) {
	// Habilitacion del reloj para TIM11 en el bus APB2.
	// TIM11 se elige por estar libre y poseer su propio vector NVIC, lo que
	// garantiza independencia respecto a la ISR del barrido del display.
	RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;

	// Pre-escalador: con HSI a 16 MHz, dividir entre 16000 produce una base
	// de tiempo de 1 kHz (un tick equivale a 1 ms exacto).
	TIM11->PSC = 16000 - 1;

	// Auto-recarga: 500 ticks de 1 ms = 500 ms por desbordamiento.
	// Conmutar el LED en cada desbordamiento produce un parpadeo a 1 Hz
	// (500 ms encendido, 500 ms apagado).
	TIM11->ARR = 500 - 1;

	// Habilitacion de la interrupcion por evento de actualizacion (overflow).
	TIM11->DIER |= TIM_DIER_UIE;

	// Habilitacion del vector en el NVIC. En STM32F411 la linea de TIM11
	// se comparte con TIM1_TRG_COM, lo cual es irrelevante aqui porque
	// TIM1 no se utiliza en este firmware.
	NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);

	// Arranque del contador.
	TIM11->CR1 |= TIM_CR1_CEN;
}

void init_exti(void) {
	// Habilitacion del reloj para SYSCFG. Este periferico es indispensable
	// para mapear pines fisicos GPIO a las lineas EXTI internas.
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// --- Mapeo de la linea EXTI1 al pin PB1 ---
	// EXTICR[0] agrupa las lineas 0..3. El campo EXTI1 (bits 4..7) selecciona
	// que puerto (A, B, C, ...) actua como fuente para la linea 1.
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI1);    // limpieza del campo
	SYSCFG->EXTICR[0] |=  SYSCFG_EXTICR1_EXTI1_PB;   // seleccion del puerto B

	// --- Mapeo de la linea EXTI12 al pin PB12 ---
	// EXTICR[3] agrupa las lineas 12..15. El campo EXTI12 (bits 0..3)
	// selecciona el puerto fuente para la linea 12.
	SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI12);   // limpieza del campo
	SYSCFG->EXTICR[3] |=  SYSCFG_EXTICR4_EXTI12_PB;  // seleccion del puerto B

	// --- Configuracion de flancos ---
	// Linea 1 (PB1): activa flanco de subida, desactiva flanco de bajada.
	EXTI->RTSR |=  EXTI_RTSR_TR1;
	EXTI->FTSR &= ~EXTI_FTSR_TR1;

	// Linea 12 (PB12): activa flanco de bajada, desactiva flanco de subida.
	EXTI->FTSR |=  EXTI_FTSR_TR12;
	EXTI->RTSR &= ~EXTI_RTSR_TR12;

	// --- Limpieza preventiva de banderas pendientes ---
	// El registro PR sigue la semantica "write-1-to-clear": escribir 1 en
	// un bit lo limpia, escribir 0 no tiene efecto. Por eso se asigna
	// directamente (no se usa |=) para evitar limpiar bits no deseados.
	EXTI->PR = (EXTI_PR_PR1 | EXTI_PR_PR12);

	// --- Desenmascaramiento de las lineas en el controlador EXTI ---
	// IMR habilita que la linea pueda generar la solicitud de interrupcion.
	EXTI->IMR |= (EXTI_IMR_IM1 | EXTI_IMR_IM12);

	// --- Habilitacion vectorial en el NVIC ---
	// Linea 1: vector dedicado EXTI1_IRQn.
	NVIC_EnableIRQ(EXTI1_IRQn);
	// Lineas 10..15: vector compartido EXTI15_10_IRQn. La discriminacion
	// entre lineas debe hacerse leyendo EXTI->PR dentro del handler.
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void EXTI1_IRQHandler(void) {
	// Vector dedicado a la linea EXTI1 (pin PB1). No requiere discriminacion
	// de fuente, pero se valida la bandera por consistencia defensiva.
	if (EXTI->PR & EXTI_PR_PR1) {

		// Limpieza de la bandera pendiente: el bit es "write-1-to-clear".
		// Asignacion directa para no afectar otros bits del registro PR.
		EXTI->PR = EXTI_PR_PR1;

		// Debounce por software: se descartan flancos que ocurran dentro
		// de la ventana DEBOUNCE_MS posterior al ultimo flanco aceptado.
		// ADVERTENCIA: tick_ms se incrementa en TIM10 cada 100 us, no cada
		// 1 ms. La ventana efectiva actual es DEBOUNCE_MS * 100 us = 5 ms.
		// Esto debera corregirse cuando se ajuste la base de tiempo.
		if ((tick_ms - ultimo_flanco_pb1_ms) > DEBOUNCE_MS) {
			counter++;                       // flanco de subida -> incremento
			ultimo_flanco_pb1_ms = tick_ms;  // se actualiza la marca propia
		}
	}
}

void EXTI15_10_IRQHandler(void) {
	// Vector compartido por las lineas EXTI10 a EXTI15. Es obligatorio
	// discriminar la fuente leyendo el registro PR antes de actuar; de lo
	// contrario, un flanco en cualquier otra linea del rango activaria
	// erroneamente la logica del contador.
	if (EXTI->PR & EXTI_PR_PR12) {

		// Limpieza exclusiva de la bandera de la linea 12.
		EXTI->PR = EXTI_PR_PR12;

		// Debounce independiente para esta linea, con su propia marca.
		if ((tick_ms - ultimo_flanco_pb12_ms) > DEBOUNCE_MS) {
			counter--;                        // flanco de bajada -> decremento
			ultimo_flanco_pb12_ms = tick_ms;  // se actualiza la marca propia
		}
	}
}

void TIM1_UP_TIM10_IRQHandler(void) {
	// Evaluacion de procedencia de la interrupcion (Update Event)
	if (TIM10->SR & TIM_SR_UIF) {
		// Limpieza de bandera de interrupcion por escritura de cero logico
		TIM10->SR &= ~TIM_SR_UIF;

		// Paso 1: Rutina de apagado absoluto (Blanking). Evita ghosting entre estados.
		GPIOC->ODR |= (0x458);
		GPIOA->ODR |= segment_pattern_a[8];
		GPIOB->ODR |= segment_pattern_b[8];
		GPIOC->ODR |= segment_pattern_c[8];
		GPIOH->ODR |= segment_pattern_h[8];

		// Paso 2: Volcado al hardware de las mascaras correspondientes al estado en curso.
		GPIOA->ODR &= ~portA_mask_buffer[isr_state_index];
		GPIOB->ODR &= ~portB_mask_buffer[isr_state_index];
		GPIOC->ODR &= ~portC_mask_buffer[isr_state_index];
		GPIOH->ODR &= ~portH_mask_buffer[isr_state_index];
		GPIOC->ODR &= ~portC_mask_digits[isr_state_index];

		isr_state_index++;

		/* * CORRECCION 1:
		 * Dado que su bucle 'for' temporal evalua 4 digitos con 4 segmentos
		 * el arreglo se llena estrictamente con 16 estados validos.
		 * Reiniciar la variable en este limite asegura la sincronia temporal del display.
		 */
		if (isr_state_index >= 28) {
			isr_state_index = 0;
		}

		tick_ms++;
	}
}

void TIM1_TRG_COM_TIM11_IRQHandler(void) {
	// Validacion del origen de la interrupcion: evento de actualizacion.
	// Aunque el vector se comparte con TIM1_TRG_COM, solo TIM11 esta
	// habilitado en este firmware, por lo que la comprobacion alcanza
	// para discriminar la fuente.
	if (TIM11->SR & TIM_SR_UIF) {

		// Limpieza de la bandera escribiendo cero en UIF.
		TIM11->SR &= ~TIM_SR_UIF;

		// Conmutacion atomica de PA5 mediante XOR sobre el registro ODR.
		// La operacion es logicamente independiente del barrido del display,
		// lo que convierte a LED2 en un indicador fiable de firmware activo.
		GPIOA->ODR ^= GPIO_ODR_OD5;
	}
}

int main(void) {

	init_gpio();
	init_tim();
	init_tim11();   // heartbeat independiente en LED2 (PA5)
	init_exti();

	while (1) {
		value = counter;

		// Limites del display e inyeccion al bufer
		if (value < 0)
			value += 10000;
		if (value > 9999)
			value -= 10000;

		display_buffer[0] = value / 1000; // millares
		display_buffer[1] = (value - display_buffer[0] * 1000) / 100; // centenas
		display_buffer[2] = (value - display_buffer[0] * 1000
				- display_buffer[1] * 100) / 10; // decenas
		display_buffer[3] = (value - display_buffer[0] * 1000
				- display_buffer[1] * 100) % 10; // unidades

		uint8_t state_idx = 0;

		// Bucle de iteracion externa sobre los 4 digitos

		for (uint8_t digit = 0; digit < 4; digit++) {
			// Extraccion del digito a representar en esta posicion
			uint8_t number = display_buffer[digit];

			// Patrones precalculados de cada puerto para el numero en cuestion
			uint16_t pattern_a = segment_pattern_a[number];
			uint16_t pattern_b = segment_pattern_b[number];
			uint16_t pattern_c = segment_pattern_c[number];
			uint16_t pattern_h = segment_pattern_h[number];

			// Mascara del anodo: siempre reside en el Puerto C, independientemente
			// de en que puerto este el segmento que se active en cada estado.
			uint8_t shift_digit = digit_shift_map_c[digit];
			uint16_t digit_mask = (1 << shift_digit);

			// --- 4 estados correspondientes a los segmentos del Puerto C ---
			for (uint8_t seg = 0; seg < 4; seg++) {
				// Posicion del bit del segmento dentro del registro ODR de C
				uint8_t shift = segment_shift_map_c[seg];

				// Evaluacion AND para determinar si este segmento se ilumina
				if (pattern_c & (1 << shift)) {
					portC_mask_buffer[state_idx] = (1 << shift); // segmento activo
					portC_mask_digits[state_idx] = digit_mask;   // anodo correspondiente
				} else {
					// Estado "muerto": ni anodo ni segmento se activan,
					// se preserva el slot temporal sin iluminar nada.
					portC_mask_buffer[state_idx] = 0;
					portC_mask_digits[state_idx] = 0;
				}

				// Puertos no involucrados en este estado: se fuerzan a cero
				// para que la ISR no altere bits ajenos al segmento activo.
				portA_mask_buffer[state_idx] = 0;
				portB_mask_buffer[state_idx] = 0;
				portH_mask_buffer[state_idx] = 0;

				state_idx++;
			}

			// --- 1 estado correspondiente al unico segmento del Puerto A ---
			uint8_t shift_a = segment_shift_map_a[0]; // unico pin de A en el display
			if (pattern_a & (1 << shift_a)) {
				portA_mask_buffer[state_idx] = (1 << shift_a);
				portC_mask_digits[state_idx] = digit_mask;
			} else {
				portA_mask_buffer[state_idx] = 0;
				portC_mask_digits[state_idx] = 0;
			}
			// Cero en los puertos restantes para este slot temporal
			portB_mask_buffer[state_idx] = 0;
			portC_mask_buffer[state_idx] = 0;
			portH_mask_buffer[state_idx] = 0;
			state_idx++;

			// --- 1 estado correspondiente al unico segmento del Puerto B ---
			uint8_t shift_b = segment_shift_map_b[0]; // unico pin de B en el display
			if (pattern_b & (1 << shift_b)) {
				portB_mask_buffer[state_idx] = (1 << shift_b);
				portC_mask_digits[state_idx] = digit_mask;
			} else {
				portB_mask_buffer[state_idx] = 0;
				portC_mask_digits[state_idx] = 0;
			}
			portA_mask_buffer[state_idx] = 0;
			portC_mask_buffer[state_idx] = 0;
			portH_mask_buffer[state_idx] = 0;
			state_idx++;

			// --- 1 estado correspondiente al unico segmento del Puerto H ---
			uint8_t shift_h = segment_shift_map_h[0]; // unico pin de H en el display
			if (pattern_h & (1 << shift_h)) {
				portH_mask_buffer[state_idx] = (1 << shift_h);
				portC_mask_digits[state_idx] = digit_mask;
			} else {
				portH_mask_buffer[state_idx] = 0;
				portC_mask_digits[state_idx] = 0;
			}
			portA_mask_buffer[state_idx] = 0;
			portB_mask_buffer[state_idx] = 0;
			portC_mask_buffer[state_idx] = 0;
			state_idx++;
		}
	}
}
