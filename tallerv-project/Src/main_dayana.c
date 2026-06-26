#include "stm32f411xe.h"

/*
 * --------------------------------------------------------------
 *  Visualizador numérico de cuatro cifras con realimentación
 *  bidireccional desde sensores ópticos.
 * --------------------------------------------------------------
 *
 *  Encendido por cifra (no por trazo): en cada ranura del
 *  temporizador se ilumina UNA cifra completa con sus siete
 *  trazos simultáneos. Total: cuatro ranuras.
 *
 *  Mapa físico:
 *
 *    Sensores ópticos (EXTI):
 *      Canal de avance    -> PB1   (flanco ascendente)
 *      Canal de retroceso -> PB14  (flanco descendente)
 *
 *    Selectores de cifra (transistores activos en BAJO):
 *      Cifra extrema izquierda -> PB12
 *      Cifra siguiente         -> PC12
 *      Cifra siguiente         -> PC5
 *      Cifra extrema derecha   -> PC8
 *
 *    Trazos (display de ánodo común, encendido con BAJO):
 *      Trazo A -> PA11    Trazo B -> PC6
 *      Trazo C -> PC13    Trazo D -> PC11
 *      Trazo E -> PC10    Trazo F -> PA12
 *      Trazo G -> PB7
 *
 *    Indicador de actividad: PA5
 */

/* =============================================================
 *  Tablas de codificación de las diez cifras decimales.
 *  Para cada puerto se almacena qué pines deben caer a BAJO
 *  cuando esa cifra se dibuja (ánodo común).
 * ============================================================= */

/* Puerto A:  PA11 = trazo A,  PA12 = trazo F */
static const uint16_t codigos_pa[10] = { (1U << 11) | (1U << 12), /* cifra 0 */
0U, /* cifra 1 */
(1U << 11), /* cifra 2 */
(1U << 11), /* cifra 3 */
(1U << 12), /* cifra 4 */
(1U << 11) | (1U << 12), /* cifra 5 */
(1U << 11) | (1U << 12), /* cifra 6 */
(1U << 11), /* cifra 7 */
(1U << 11) | (1U << 12), /* cifra 8 */
(1U << 11) | (1U << 12) /* cifra 9 */
};

/* Puerto B:  PB7 = trazo G */
static const uint16_t codigos_pb[10] = { 0U, /* 0 */
0U, /* 1 */
(1U << 7), /* 2 */
(1U << 7), /* 3 */
(1U << 7), /* 4 */
(1U << 7), /* 5 */
(1U << 7), /* 6 */
0U, /* 7 */
(1U << 7), /* 8 */
(1U << 7) /* 9 */
};

/* Puerto C:  PC6 = B,  PC10 = E,  PC11 = D,  PC13 = C */
static const uint16_t codigos_pc[10] = { (1U << 6) | (1U << 13) | (1U << 11)
		| (1U << 10), /* 0 */
(1U << 6) | (1U << 13), /* 1 */
(1U << 6) | (1U << 11) | (1U << 10), /* 2 */
(1U << 6) | (1U << 13) | (1U << 11), /* 3 */
(1U << 6) | (1U << 13), /* 4 */
(1U << 13) | (1U << 11), /* 5 */
(1U << 13) | (1U << 11) | (1U << 10), /* 6 */
(1U << 6) | (1U << 13), /* 7 */
(1U << 6) | (1U << 13) | (1U << 11) | (1U << 10), /* 8 */
(1U << 6) | (1U << 13) | (1U << 11) /* 9 */
};

/* =============================================================
 *  Identificadores de los pines selectores de cada cifra,
 *  expresados como bits dentro de su puerto correspondiente.
 * ============================================================= */
#define SEL_CIFRA_A   (1U<<12)    /* PB12 */
#define SEL_CIFRA_B   (1U<<12)    /* PC12 */
#define SEL_CIFRA_C   (1U<<5)     /* PC5  */
#define SEL_CIFRA_D   (1U<<8)     /* PC8  */

/* Conjuntos completos de trazos por puerto, útiles para borrar
 * la imagen anterior antes de pintar la nueva cifra. */
#define LIMPIA_REGISTRO_A   ((1U<<11) | (1U<<12))
#define LIMPIA_REGISTRO_B   ((1U<<7))
#define LIMPIA_REGISTRO_C   ((1U<<6) | (1U<<10) | (1U<<11) | (1U<<13))

/* =============================================================
 *  Estado interno del programa
 * ============================================================= */

/* Cuatro casillas con la cifra que toca a cada posición.
 * Índice 0 = extremo izquierdo, índice 3 = extremo derecho. */
volatile uint8_t digitos_display[4] = { 0U, 0U, 0U, 0U };

/* Acumulador bidireccional manipulado por los sensores ópticos. */
volatile int32_t contador = 0;

/* Imagen estable del acumulador usada por el bucle principal. */
int32_t valor_display = 0;

/* Posición actual del barrido: cifra que se está iluminando. */
volatile uint8_t display_index = 0U;

/* Cuenta de milisegundos transcurridos (incrementada en TIM10). */
volatile uint32_t reloj_ms = 0U;

/* Marcas temporales del último pulso aceptado en cada canal. */
volatile uint32_t tiempo_avance = 0U;
volatile uint32_t tiempo_retroceso = 0U;

/* Ventana mínima entre dos flancos consecutivos para filtrar
 * rebotes mecánicos u ópticos. */
#define VENTANA_FILTRO_MS   50U

/* =============================================================
 *  Configuración de los puertos de entrada/salida
 * ============================================================= */
void init_gpio(void) {
	/* Habilitación de los relojes de los puertos involucrados:
	 * A, B y C.  El indicador PA5 reside en el puerto A, ya
	 * habilitado para los trazos. */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
			| RCC_AHB1ENR_GPIOCEN;

	__asm("nop");
	/* Espera trivial para asegurar el reloj */

	/* ----- Entradas de los sensores ópticos ----- */
	/* PB1 y PB14 sin resistencias internas (acondicionamiento externo) */
	GPIOB->MODER &= ~(GPIO_MODER_MODE1 | GPIO_MODER_MODE14);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD14);

	/* ----- Salidas para los selectores de cifra ----- */
	GPIOB->MODER &= ~GPIO_MODER_MODE12;
	GPIOB->MODER |= GPIO_MODER_MODE12_0; /* PB12 push-pull */

	GPIOC->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE8 | GPIO_MODER_MODE12);
	GPIOC->MODER |= (GPIO_MODER_MODE5_0 | GPIO_MODER_MODE8_0
			| GPIO_MODER_MODE12_0);

	/* Todos los selectores en ALTO al arrancar (cifras apagadas). */
	GPIOB->BSRR = SEL_CIFRA_A;
	GPIOC->BSRR = SEL_CIFRA_B | SEL_CIFRA_C | SEL_CIFRA_D;

	/* ----- Salidas para los trazos del display ----- */
	/* PA11 (A), PA12 (F) y PA5 (indicador de actividad) */
	GPIOA->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE11 | GPIO_MODER_MODE12);
	GPIOA->MODER |= (GPIO_MODER_MODE5_0 | GPIO_MODER_MODE11_0
			| GPIO_MODER_MODE12_0);

	GPIOB->MODER &= ~GPIO_MODER_MODE7;
	GPIOB->MODER |= GPIO_MODER_MODE7_0;

	GPIOC->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE10 | GPIO_MODER_MODE11
			| GPIO_MODER_MODE13);
	GPIOC->MODER |= (GPIO_MODER_MODE6_0 | GPIO_MODER_MODE10_0
			| GPIO_MODER_MODE11_0 | GPIO_MODER_MODE13_0);

	/* Todos los trazos en ALTO al arrancar (display en blanco).
	 * El indicador de actividad PA5 se deja apagado (BAJO). */
	GPIOA->BSRR = LIMPIA_REGISTRO_A | ((uint32_t) (1U << 5) << 16);
	GPIOB->BSRR = LIMPIA_REGISTRO_B;
	GPIOC->BSRR = LIMPIA_REGISTRO_C;
}

/* =============================================================
 *  TIM10: motor del barrido.
 *  16 MHz / 16 = 1 MHz (1 us por tick).  ARR = 1000 -> 1 ms por
 *  ranura.  Cuatro ranuras dan ~250 Hz de refresco completo.
 * ============================================================= */
void init_tim10(void) {
	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;

	TIM10->PSC = 16U - 1U;
	TIM10->ARR = 1000U - 1U;

	TIM10->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	TIM10->CR1 |= TIM_CR1_CEN;
}

/* =============================================================
 *  TIM11: parpadeo del indicador de actividad cada 500 ms.
 * ============================================================= */
void init_tim11(void) {
	RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;

	TIM11->PSC = 16000U - 1U; /* base de 1 kHz */
	TIM11->ARR = 500U - 1U; /* 500 ms */

	TIM11->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
	TIM11->CR1 |= TIM_CR1_CEN;
}

/* =============================================================
 *  EXTI:
 *    Línea 1  -> PB1, flanco ascendente  -> avance
 *    Línea 14 -> PB14, flanco descendente -> retroceso
 * ============================================================= */
void init_exti(void) {
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* Asignar EXTI1 al puerto B (registro EXTICR1: líneas 0..3) */
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PB;

	/* Asignar EXTI14 al puerto B (registro EXTICR4: líneas 12..15) */
	SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI14;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI14_PB;

	/* Selección de polaridades */
	EXTI->RTSR |= EXTI_RTSR_TR1; /* PB1: solo subida */
	EXTI->FTSR &= ~EXTI_FTSR_TR1;

	EXTI->FTSR |= EXTI_FTSR_TR14; /* PB14: solo bajada */
	EXTI->RTSR &= ~EXTI_RTSR_TR14;

	/* Limpieza de banderas previas y desenmascarado */
	EXTI->PR = (EXTI_PR_PR1 | EXTI_PR_PR14);
	EXTI->IMR |= (EXTI_IMR_IM1 | EXTI_IMR_IM14);

	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* =============================================================
 *  Rutinas de atención a interrupciones
 * ============================================================= */

/* Barrido del display: una cifra por interrupción del TIM10 */
void TIM1_UP_TIM10_IRQHandler(void) {
	if (TIM10->SR & TIM_SR_UIF) {

		TIM10->SR &= ~TIM_SR_UIF; /* aceptar el evento */

		/* Apagar todas las cifras (selectores a ALTO). BSRR escribe
		 * de forma atómica sin alterar el resto de bits del puerto. */
		GPIOB->BSRR = SEL_CIFRA_A;
		GPIOC->BSRR = SEL_CIFRA_B | SEL_CIFRA_C | SEL_CIFRA_D;

		/* Apagar todos los trazos (puertos a ALTO en esos pines). */
		GPIOA->BSRR = LIMPIA_REGISTRO_A;
		GPIOB->BSRR = LIMPIA_REGISTRO_B;
		GPIOC->BSRR = LIMPIA_REGISTRO_C;

		/* Pintar los trazos de la cifra actual.  La parte alta de
		 * BSRR (bits 16..31) lleva a BAJO los pines indicados. */
		uint8_t simbolo = digitos_display[display_index];

		GPIOA->BSRR = ((uint32_t) codigos_pa[simbolo]) << 16;
		GPIOB->BSRR = ((uint32_t) codigos_pb[simbolo]) << 16;
		GPIOC->BSRR = ((uint32_t) codigos_pc[simbolo]) << 16;

		/* Activar el selector de la cifra actual (transistor ON). */
		switch (display_index) {
		case 0:
			GPIOB->BSRR = ((uint32_t) SEL_CIFRA_A) << 16;
			break;
		case 1:
			GPIOC->BSRR = ((uint32_t) SEL_CIFRA_B) << 16;
			break;
		case 2:
			GPIOC->BSRR = ((uint32_t) SEL_CIFRA_C) << 16;
			break;
		case 3:
			GPIOC->BSRR = ((uint32_t) SEL_CIFRA_D) << 16;
			break;
		default:
			break;
		}

		/* Avanzar el cursor de forma circular 0->1->2->3->0 */
		display_index++;
		if (display_index >= 4U) {
			display_index = 0U;
		}

		/* Base de tiempo para el filtrado de rebotes en los EXTI */
		reloj_ms++;
	}
}

/* Parpadeo del indicador de actividad */
void TIM1_TRG_COM_TIM11_IRQHandler(void) {
	if (TIM11->SR & TIM_SR_UIF) {
		TIM11->SR &= ~TIM_SR_UIF;
		GPIOA->ODR ^= GPIO_ODR_OD5; /* alternar PA5 */
	}
}

/* Sensor de avance: flanco ascendente en PB1 */
void EXTI1_IRQHandler(void) {
	if (EXTI->PR & EXTI_PR_PR1) {

		EXTI->PR = EXTI_PR_PR1; /* reconocer la línea */

		/* Aceptar solo si pasó más de la ventana mínima desde el
		 * último pulso válido (filtro temporal antirrebote). */
		if ((reloj_ms - tiempo_avance) > VENTANA_FILTRO_MS) {
			contador++;
			tiempo_avance = reloj_ms;
		}
	}
}

/* Sensor de retroceso: flanco descendente en PB14.  El vector
 * EXTI15_10 agrupa las líneas 10..15; aquí solo está activa la 14. */
void EXTI15_10_IRQHandler(void) {
	if (EXTI->PR & EXTI_PR_PR14) {
		EXTI->PR = EXTI_PR_PR14;
		if ((reloj_ms - tiempo_retroceso) > VENTANA_FILTRO_MS) {
			contador--;
			tiempo_retroceso = reloj_ms;
		}
	}
}

/* =============================================================
 *  Programa principal
 * ============================================================= */
int main(void) {
	init_gpio();
	init_tim10();
	init_tim11();
	init_exti();

	while (1) {

		/* Captura puntual del acumulador.  En Cortex-M4 la lectura
		 * de un int32 alineado es atómica, así que no hace falta
		 * deshabilitar interrupciones para esta copia. */
		valor_display = contador;

		/* Mantener la lectura dentro del rango representable
		 * (cuatro cifras decimales, 0000..9999). */
		if (valor_display < 0)
			valor_display += 10000;
		if (valor_display > 9999)
			valor_display -= 10000;

		/* Desarmar la lectura en sus cifras componentes.
		 * Posición 0 = millares, posición 3 = unidades. */
		digitos_display[0] = (uint8_t) (valor_display / 1000);
		digitos_display[1] = (uint8_t) ((valor_display / 100) % 10);
		digitos_display[2] = (uint8_t) ((valor_display / 10) % 10);
		digitos_display[3] = (uint8_t) (valor_display % 10);

		/* El pintado del display ocurre íntegramente dentro de la
		 * ISR del TIM10, por lo que el bucle principal queda libre
		 * para crecer con otras tareas si fuera necesario. */
	}
}
