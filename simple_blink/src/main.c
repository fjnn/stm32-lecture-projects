#include "stm32f7xx_hal.h"

// Define the LED pin and port (assuming PB0 for the Nucleo-F767ZI)
// On Nucleo-F767ZI, the user LED (LD1) is Green and connected to PB0.
#define LED_PIN             GPIO_PIN_0
#define LED_GPIO_PORT       GPIOB
#define LED_GPIO_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE() // Note the parentheses for the macro call

// Function prototypes
static void MX_GPIO_Init(void);
void Error_Handler(void);

int main(void)
{
  /* HAL library initialization: reset all peripherals, initialize Flash and Systick. */
  HAL_Init();

  /* Initialize GPIO for the LED */
  MX_GPIO_Init();

  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN); // Toggle the LED state
    HAL_Delay(500);                             // Delay for 500 milliseconds (0.5 seconds)
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

  /* Enable the clock for the GPIO Port */
  LED_GPIO_CLK_ENABLE; // Call the clock enable macro

  /* Configure GPIO pin Output Level - initially low */
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);

  /* Configure LED pin as output push pull */
  GPIO_InitStruct.Pin   = LED_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Can be low for an LED

  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
}

// Minimal error handler for a simple blink.
// In a real application, you should have proper error handling.
void Error_Handler(void)
{
  /* User can add their own implementation to report the HAL error return state */
  while(1)
  {
    // You could blink an error code here, or just halt.
  }
}

// The assert_failed function is only needed if USE_FULL_ASSERT is defined.
// For simple examples, it's often not strictly necessary unless debugging.
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     e.g., printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */