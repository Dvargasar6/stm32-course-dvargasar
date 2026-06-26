/*
 * main_hal.c
 *
 *  Created on: Jun 11, 2026
 *      Author: Daniel
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>

/* Handle de TIM3 — debe ser global para que stm32f4xx_it.c pueda accederlo */
TIM_HandleTypeDef htim3 = {0};
USART_HandleTypeDef husart2 = {0};
ADC_HandleTypeDef hadc1 = {0};
volatile uint16_t raw_adc = 0;
volatile uint8_t adc_done = 0;
volatile uint8_t ShowMsg = 0;
float adc_mv = 0.0f;
uint8_t msg_buffer[64]={0};

/* Prototipos de funciones privadas */
static void SystemClock_Config(void);
static void gpio_Init(void);
static void tim3_Init(void);
static void usart_Init(void);
static void adc_Init(void);



int main(void)
{
    HAL_Init();           /* inicializa HAL: SysTick, caché, agrupación de prioridades */
    SystemClock_Config(); /* configura el árbol de relojes: HSI a 16 MHz               */
    gpio_Init();          /* configura PA5 como salida push-pull                        */
    tim3_Init();          /* configura TIM3: evento de actualización cada 250 ms        */
    usart_Init();          /* configura USART2: recibo y envío de mensajes */

    adc_Init();

    HAL_ADC_Start_IT(&hadc1);

    while (1)
    {
        /* bucle de aplicación — la conmutación del LED ocurre en el callback */
        if(ShowMsg){
            HAL_USART_Transmit(&husart2, (uint8_t *)"Hola mundo", 15, 100);
            ShowMsg = 0;
            HAL_ADC_Start_IT(&hadc1);
        }
//    if (adc_done == 1){
  //      adc_mv = (float)()
        //}
    }
}


/*
 * SystemClock_Config
 * Usa el oscilador interno HSI a 16 MHz
 * Sin PLL — configuración de reloj más simple posible
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* HSI ya está encendido al resetear — confirmar y usarlo */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* Seleccionar HSI como SYSCLK — todos los divisores de bus en 1 */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK |
            RCC_CLOCKTYPE_HCLK   |
            RCC_CLOCKTYPE_PCLK1  |
            RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK  = 16 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;     /* APB1  = 16 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     /* APB2  = 16 MHz */

    /* FLASH_LATENCY_0 = cero wait states, correcto para 16 MHz */
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

/*
 * gpio_Init
 * Configura PA5 como salida push-pull — LED de la tarjeta Nucleo
 */
static void gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Habilitar reloj de GPIOA en el bus AHB1
       Equivalente bare-metal: RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN */
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* Configurar PH1 */
    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}

static void adc_Init(void){
    /* Configuramos GPIO */
    GPIO_InitTypeDef GPIO_ADC = {0};

    /* Habilitar reloj de GPIOA en el bus AHB1
           Equivalente bare-metal: RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configurar PA4 */
    GPIO_ADC.Pin   = GPIO_PIN_4;
    GPIO_ADC.Mode  = GPIO_MODE_ANALOG;
    GPIO_ADC.Pull  = GPIO_NOPULL;

    /* Cargamos configuración */
    HAL_GPIO_Init(GPIOA, &GPIO_ADC);

    /* Señal de reloj de ADC1 */
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DMAContinuousRequests = DISABLE;

    if(HAL_ADC_Init(&hadc1) != HAL_OK){
        while(1){
            __NOP();
        }
    }

    ADC_ChannelConfTypeDef poten = {0};
    poten.Channel = 4;
    poten.Rank = 1;
    poten.SamplingTime = ADC_SAMPLETIME_56CYCLES;
    poten.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &poten) != HAL_OK)
    {
        __NOP();
    }
    HAL_NVIC_EnableIRQ(ADC_IRQn);
}
static void tim3_Init(void)
{
    /* Habilitar reloj de TIM3 en el bus APB1 */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Configurar la base de TIM3 */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 15999;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 249;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_Base_Init(&htim3);

    /* Habilitar la línea de interrupción de TIM3 en el NVIC */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    /* Arrancar TIM3 en modo interrupción — habilita la interrupción de evento de actualización */
    HAL_TIM_Base_Start_IT(&htim3);
}

static void usart_Init(void){
    GPIO_InitTypeDef GPIO_InitUsart2Tx = {0};

    /* Habilitar reloj de GPIOA en el bus AHB1
       Equivalente bare-metal: RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configurar GPIO_ADC */
    GPIO_InitUsart2Tx.Pin   = GPIO_PIN_2;
    GPIO_InitUsart2Tx.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitUsart2Tx.Pull  = GPIO_NOPULL;
    GPIO_InitUsart2Tx.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitUsart2Tx.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitUsart2Tx);

    /* Habilitar reloj de TIM3 en el bus APB1 */
    __HAL_RCC_USART2_CLK_ENABLE();

    /* Configurar USART 2 */
    husart2.Instance = USART2;
    husart2.Init.BaudRate = 19200;
    husart2.Init.Mode = USART_MODE_TX;
    husart2.Init.Parity = USART_PARITY_NONE;
    husart2.Init.StopBits = USART_STOPBITS_1;
    husart2.Init.WordLength = USART_WORDLENGTH_8B;

    /* Cargar configuración del USART2 en los FSR del MCU */
    HAL_USART_Init(&husart2);

    HAL_USART_Transmit(&husart2, (uint8_t *) "Hola Mundo!!", 13, 100);
}

/*
 * HAL_TIM_PeriodElapsedCallback
 * Llamado automáticamente por HAL_TIM_IRQHandler() cada vez que un evento
 * de actualización del timer se dispara. Es compartido por todos los timers
 * — siempre verifica htim->Instance.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);
    }
}

void HA_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    if(hadc->Instance == ADC1){
        raw_adc = hadc->Instance->DR;
        adc_done = 1;
    }

}
