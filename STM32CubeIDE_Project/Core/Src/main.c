/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : UART-Controlled LED System for STM32F103C6
  * @description    : Control 2 LEDs using two-digit UART commands
  *                   Command format: "XY" where X controls LED1, Y controls LED2
  *                   0 = LOW (OFF), 1 = HIGH (ON), 2 = TOGGLE
  * @hardware       : LED1 -> PA0, LED2 -> PA1
  *                   UART1 -> PA9 (TX), PA10 (RX)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>  // For memset() and strlen()

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;  // UART1 handle structure

/* USER CODE BEGIN PV */
uint8_t rx_buffer[2];  // Buffer to store 2 received bytes from UART
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void LED_Control(uint8_t code);        // Function to control LED states based on code
void UART_SendString(char *str);       // Helper function to send strings via UART
/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // Variable declarations can go here if needed
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
  MX_GPIO_Init();        // Initialize GPIO pins (PA0 and PA1 as outputs)
  MX_USART1_UART_Init(); // Initialize UART1 (9600 baud, 8N1)

  /* USER CODE BEGIN 2 */

  // Send welcome message to terminal when system starts
  UART_SendString("\r\n");
  UART_SendString("========================================\r\n");
  UART_SendString("  STM32F103C6 UART LED Control System  \r\n");
  UART_SendString("========================================\r\n");
  UART_SendString("\r\n");
  UART_SendString("Command Format: Two digits (e.g., '11')\r\n");
  UART_SendString("  First digit  -> LED1 (PA0)\r\n");
  UART_SendString("  Second digit -> LED2 (PA1)\r\n");
  UART_SendString("\r\n");
  UART_SendString("Commands:\r\n");
  UART_SendString("  0 = Turn LED OFF\r\n");
  UART_SendString("  1 = Turn LED ON\r\n");
  UART_SendString("  2 = TOGGLE LED\r\n");
  UART_SendString("\r\n");
  UART_SendString("Examples:\r\n");
  UART_SendString("  '11' = Both LEDs ON\r\n");
  UART_SendString("  '00' = Both LEDs OFF\r\n");
  UART_SendString("  '10' = LED1 ON, LED2 OFF\r\n");
  UART_SendString("  '21' = LED1 TOGGLE, LED2 ON\r\n");
  UART_SendString("========================================\r\n");
  UART_SendString("System Ready! Waiting for commands...\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Clear the receive buffer to prevent stale data from previous reads
    memset(rx_buffer, 0, sizeof(rx_buffer));

    // Wait to receive exactly 2 bytes from UART
    // HAL_MAX_DELAY means block indefinitely until 2 bytes arrive
    if(HAL_UART_Receive(&huart1, rx_buffer, 2, HAL_MAX_DELAY) == HAL_OK)
    {
      // Validate that both received characters are valid ('0', '1', or '2')
      if(rx_buffer[0] >= '0' && rx_buffer[0] <= '2' &&
         rx_buffer[1] >= '0' && rx_buffer[1] <= '2')
      {
        // Convert ASCII characters to numeric code
        // Example: "21" becomes (2)*10 + (1) = 21
        // '0' has ASCII value 48, so '0'-'0'=0, '1'-'0'=1, '2'-'0'=2
        uint8_t code = (rx_buffer[0] - '0') * 10 + (rx_buffer[1] - '0');

        // Execute LED control based on the received code
        LED_Control(code);

        // Send confirmation message back to terminal
        UART_SendString("[OK] Command: ");
        HAL_UART_Transmit(&huart1, rx_buffer, 2, HAL_MAX_DELAY);  // Echo the command
        UART_SendString(" executed successfully\r\n");
      }
      else
      {
        // Handle invalid input (characters other than 0, 1, or 2)
        UART_SendString("[ERROR] Invalid command: ");
        HAL_UART_Transmit(&huart1, rx_buffer, 2, HAL_MAX_DELAY);
        UART_SendString("\r\n");
        UART_SendString("        Please use only digits 0, 1, or 2\r\n");
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Controls LED states based on received two-digit code
  * @param  code: Numeric code where:
  *               - Tens digit (code/10) controls LED1
  *               - Ones digit (code%10) controls LED2
  *               - 0 = Turn LED OFF
  *               - 1 = Turn LED ON
  *               - 2 = Toggle LED state
  * @retval None
  * @example LED_Control(21) -> LED1 toggles, LED2 turns on
  */
void LED_Control(uint8_t code)
{
  // Extract individual LED actions from the combined code
  uint8_t led1_action = code / 10;  // Tens digit controls LED1
  uint8_t led2_action = code % 10;  // Ones digit controls LED2

  // Control LED1 (connected to GPIOA Pin 0)
  switch (led1_action)
  {
    case 0:
      // Turn LED1 OFF by setting pin to LOW (0V)
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
      break;

    case 1:
      // Turn LED1 ON by setting pin to HIGH (3.3V)
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
      break;

    case 2:
      // Toggle LED1 state (if ON->OFF, if OFF->ON)
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
      break;

    default:
      // Should never reach here due to input validation
      break;
  }

  // Control LED2 (connected to GPIOA Pin 1)
  switch (led2_action)
  {
    case 0:
      // Turn LED2 OFF by setting pin to LOW (0V)
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
      break;

    case 1:
      // Turn LED2 ON by setting pin to HIGH (3.3V)
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
      break;

    case 2:
      // Toggle LED2 state (if ON->OFF, if OFF->ON)
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
      break;

    default:
      // Should never reach here due to input validation
      break;
  }
}

/**
  * @brief  Sends a null-terminated string via UART
  * @param  str: Pointer to the string to transmit
  * @retval None
  * @note   This is a wrapper function to simplify sending strings
  */
void UART_SendString(char *str)
{
  // Transmit the entire string over UART1
  // strlen(str) calculates the number of characters in the string
  // HAL_MAX_DELAY means wait indefinitely until transmission completes
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

/* USER CODE END 4 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;  // Use internal oscillator
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                     // Enable HSI
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;               // PLL not used
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;       // System clock from HSI
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;           // No division
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;            // No division
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;            // No division

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;                      // Communication speed: 9600 bits per second
  huart1.Init.WordLength = UART_WORDLENGTH_8B;      // 8 data bits per frame
  huart1.Init.StopBits = UART_STOPBITS_1;           // 1 stop bit
  huart1.Init.Parity = UART_PARITY_NONE;            // No parity checking
  huart1.Init.Mode = UART_MODE_TX_RX;               // Enable both transmit and receive
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;      // No hardware flow control (no RTS/CTS)
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;  // 16x oversampling for accuracy

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable clock for GPIOA peripheral

  /* Configure GPIO pin Output Level */
  // Initialize both LED pins to LOW (OFF) state at startup
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /* Configure GPIO pins : PA0 PA1 (LED1 and LED2) */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;      // Select pins PA0 and PA1
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;       // Push-pull output mode
  GPIO_InitStruct.Pull = GPIO_NOPULL;               // No pull-up or pull-down resistor
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;      // Low speed is sufficient for LED control
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);           // Apply configuration to GPIOA
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add custom error handling code here */
  __disable_irq();  // Disable all interrupts
  while (1)
  {
    // Stay in infinite loop to indicate error state
    // In production, you might want to blink an LED rapidly here
    // to visually indicate an error has occurred
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
  /* User can add their own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
