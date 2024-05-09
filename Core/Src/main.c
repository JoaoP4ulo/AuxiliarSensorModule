/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ANALOG_READS 1
#define VALUE_STAGE_1 1700 // 70 - 1.51V
#define VALUE_STAGE_2 500 // 110 - 0.64V
#define HISTERESYS 150 //
#define BUFFER_SIZE 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*  ADC  */
uint32_t ADCReadings[ANALOG_READS];
uint32_t anBuff[BUFFER_SIZE] = {0};
bool ADC_Complete = false;
uint32_t mediaRead = 0;
uint8_t buf_cnt = 0;


/*  TIMER  */
uint16_t timer_diff;
uint16_t timer_diff2;
uint16_t timer_diff3;

uint16_t timer_bfr = 0;
uint16_t timer_bfr2 = 0;
uint16_t timer_bfr3 = 0;

uint16_t timer_curr = 0;
bool relay_status = false;

uint8_t read_cont = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /*
   * OBS: Uma breve inicialização com LED piscando para verificar que o controlador foi inicializado.
   * */
  for(int i = 0; i < 5; i++) {
	  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	  HAL_Delay(200);
  }

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADCReadings, ANALOG_READS);

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  timer_curr = HAL_GetTick(); // OBS: Essa função retorna o valor em millissegundos.


	  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	  // HAL_Delay(300); // OBS: Esse delay dentro do loop tá travando o seu código por 300ms a cada interação. Isso não se faz!
	  

	  timer_diff = timer_curr - timer_bfr; // OBS: Show! Você inicializou tudo com 0 e como variáveis globais, então vai funcionar.

	  if (timer_diff >= 100) { // OBS: Você executa esse trecho de código a cada aproximadamento 100ms. OK!
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADCReadings, ANALOG_READS); // OBS: Habilita a amostragem do valor das analógicas utilizando DMA. OK!

		  if (ADC_Complete) {	// OBS: Aguarda até as analógicas serem completamente amostradas e convertidas pelo ADC. OK!
			  ADC_Complete = false; // OBS: Atualiza a variável de comunicação do loop com a interrupção. OK!
			  HAL_ADC_Stop_DMA(&hadc1);


			// OBS: Só posiciona o valor lido na analógica na posição da leitura mais antiga, e fica rodando essa variável 'buf_cnt'
			// OBS: Não interessa a posição do valor, no final das contas você vai somar tudo e dividir mesmo.
			anBuff[buf_cnt] = ADCReadings[0];
			buf_cnt++;
			if(buf_cnt == BUFFER_SIZE) {buf_cnt = 0;}

			// OBS: Adicionei a média aqui pois ela somente se altera quando se lê uma nova analógica, logo não faz sentido rodar ela em outro trecho do código
		    // Calcula a média dos valores em anBuff
		    uint32_t sum = 0;

		    for (int i = 0; i < BUFFER_SIZE; i++) {
		      sum += anBuff[i];
		    }
		    mediaRead = sum / BUFFER_SIZE;

      }
      // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADCReadings, ANALOG_READS); // OBS: Você já está habilitando a amostragem do valor das analógicas lá no inicio a cada 100ms.
      	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	   // OBS: Habilitando essa amostragem aqui você tá pedindo para o controlador inicializar a leitura e somente irá ler esse valor 100ms depois.
      	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	   // OBS: Quando você entrar no trecho que roda a cada 100ms o ADC_Complete já vai estar True e com o valor de 100ms atrás.
      timer_bfr = timer_curr; // OBS: Atualiza o novo valor de timer_bfr somente após completar o código completo que roda em 100ms (Amostragem e Leitura da analógica). OK!
    }


	timer_diff2 = timer_curr - timer_bfr2; // OBS: Pelo que entendi você vai usar outro conjunto de variáveis para controlar o tempo da outra função. Então vai precisar desse mesmo processo.
										   // OBS: Daria para fazer usando a mesma variável, mas não importa muito para esse rpograma específico. Podemos fazer assim, sem problemas.

	if (timer_diff2 >= 1000) { // Esse trecho executa a cada 1 segundo. Coloquei 1 segundo e troquei a média movel para 10 valores. BUFFER_SIZE = 10;

	
		// OBS: Observa como funciona a histerese. Depois que o relé liga quando a temperatura passa do VALUE_STAGE_1, ele só desliga depois que desce pelo menos a histerese dese alvo, senão não faz nada e permanece ligado.
		if (mediaRead >= VALUE_STAGE_1){
			relay_status = true; // OBS: Considerei o true como sendo a ventoinha ligada (relé desligado pois estamos usando NC do relé). VERIFICAR A LÓGICA!
		}
		else {
			if(mediaRead <= VALUE_STAGE_1 - HISTERESYS) { // OBS: Coloquei uma observação lá em cima no define da histerese. Colocar um valor fixo e não porcentagem.
				relay_status = false;
			}
		}

		// OBS: Aqui atualiza o output do GPIO para acionamento do relé. OK! (Relé desligado é ventoinha ligada pois estamos usando o NC do relé. VERIFICAR A LÓGICA!
		if (relay_status){
			HAL_GPIO_WritePin(RELAY_SIGNAL_GPIO_Port, RELAY_SIGNAL_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(RELAY_SIGNAL_GPIO_Port, RELAY_SIGNAL_Pin, GPIO_PIN_RESET);
		}

	}


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if (hadc == &hadc1){

		ADC_Complete = true;

	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
