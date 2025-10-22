/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{                  // Khai báo trạng thái LED
  STATE_WAIT_KEY0, // Chờ nhấn thả nút KEY0, 4 LED sáng
  STATE_LED_ASC,   // LED sáng từ 0000 đến 1111
  STATE_LED_DESC,  // LED sáng từ 1111 đến 0000
  STATE_KEY1       // Nhấn KEY1, tắt tất cả LED
} LED_Mode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t key0_pressing = 0;        // Trạng thái nhấn KEY0
static uint32_t key0_press_time = 0;     // Mốc thời gian khi KEY0 được nhấn
static uint32_t step_time_mark = 0;      // Mốc thời gian để thay đổi trạng thái LED
static uint8_t current_count_value = 0;  // Giá trị đếm nhị phân của LED
LED_Mode current_mode = STATE_WAIT_KEY0; // Trạng thái hiện tại của LED (Ban đầu: chờ nhấn thả KEY0 -> 4 LED sáng)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
// Hàm thiết lập trạng thái LED theo giá trị nhị phân
void Set_LEDs_Status(uint8_t value)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (value & 0b1000) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PB0 (MSB) -> LED 0
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (value & 0b0100) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PB1       -> LED 1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (value & 0b0010) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PB2       -> LED 2
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (value & 0b0001) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PB3 (LSB) -> LED 3
}

// Hàm thực thi trạng thái hiện tại
void Execute_Mode(void)
{
  switch (current_mode)
  {
  case STATE_WAIT_KEY0: // Chờ nhấn thả KEY0
    Set_LEDs_Status(0b1111); // Tất cả LED sáng
    break;
  case STATE_LED_ASC: // LED sáng từ 0000 đến 1111
    if (HAL_GetTick() - step_time_mark >= 1000) // Mỗi giây tăng giá trị đếm (delay trạng thái LED 1s)
    {
      step_time_mark = HAL_GetTick();

      // Cập nhật giá trị đếm
      current_count_value++;
      if (current_count_value > 0b1111)
      {
        current_count_value = 0b0000; // Quay lại 0000
      }
      Set_LEDs_Status(current_count_value);
    }
    break;
  case STATE_LED_DESC: // LED sáng từ 1111 đến 0000
    if (HAL_GetTick() - step_time_mark >= 1000)
    {
      step_time_mark = HAL_GetTick();

      // Cập nhật giá trị đếm
      current_count_value--;
      if (current_count_value == 255)
      {
        current_count_value = 0b1111; // Quay lại 1111
      }
      Set_LEDs_Status(current_count_value);
    }
    break;
  case STATE_KEY1: // Nhấn KEY1
    Set_LEDs_Status(0b0000); // Tắt tất cả LED
    break;
  default:
    break;
  }
}
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //     Nếu nhấn KEY1 -> Tắt tất cả LED
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
    {
      current_mode = STATE_KEY1; // Chuyển về trạng thái nhấn KEY1 -> tắt tất cả LED
    }

    //     Nhấn KEY0
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET && key0_pressing == 0)
    {
      key0_pressing = 1; // Đang nhấn KEY0
      key0_press_time = HAL_GetTick(); // Ghi lại mốc thời gian nhấn KEY0
      current_mode = STATE_WAIT_KEY0; // Chuyển về trạng thái chờ nhấn thả KEY0 -> 4 LED sáng
    }

    //     Thả KEY0
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET && key0_pressing == 1)
    {
      key0_pressing = 0; // Đã thả KEY0
      uint32_t hold_duration = HAL_GetTick() - key0_press_time; // Thời gian giữ nút KEY0
      if (hold_duration <= 5000)
      { // Giữ từ 1 đến 5 giây
        // LED sáng từ 0000 đến 1111
        current_count_value = 0b0000;
        current_mode = STATE_LED_ASC;
      }
      else
      { // Giữ lâu hơn 5 giây
        // LED sáng từ 1111 đến 0000
        current_count_value = 0b1111;
        current_mode = STATE_LED_DESC;
      }
    }

    Execute_Mode();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
