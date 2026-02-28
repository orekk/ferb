/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define SAMPLE_RATE     16000
#define FRAME_MS        20
#define FRAME_SAMPLES   (SAMPLE_RATE * FRAME_MS / 1000) // 320
#define FRAME_BYTES     (FRAME_SAMPLES * 2)

#define MAGIC0 0xA5
#define MAGIC1 0x5A
#define PCM_BYTES   FRAME_BYTES     // 640
#define HDR_BYTES   6               // [A5 5A][uint16 fc][uint16 len]
#define PKT_BYTES   (HDR_BYTES + PCM_BYTES)

static uint16_t fc = 0;
static uint8_t tx_pkt[PKT_BYTES];

static int32_t dfsdm_dma[FRAME_SAMPLES * 2];

volatile uint32_t half0_pending = 0;
volatile uint32_t half1_pending = 0;

static int32_t dc = 0;

//// USB audio double-buffer (so USB can transmit while we fill next frame)
//static int16_t pcm_frame_a[FRAME_SAMPLES];
//static int16_t pcm_frame_b[FRAME_SAMPLES];
//extern volatile uint8_t usb_tx_busy;
//static uint8_t pcm_buf_sel = 0; // 0 -> a, 1 -> b

extern volatile uint8_t usb_tx_busy;

/* ---- NEW: small PCM frame queue (ring buffer) ---- */
#define PCM_Q_FRAMES  8   // 8 * 640B = 5120B RAM
static int16_t pcm_q[PCM_Q_FRAMES][FRAME_SAMPLES];
static volatile uint8_t pcm_q_w = 0;
static volatile uint8_t pcm_q_r = 0;
static volatile uint8_t pcm_q_count = 0;

static int16_t scratch0[FRAME_SAMPLES];
static int16_t scratch1[FRAME_SAMPLES];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#include <stdio.h>
#include <string.h>
#include <stdint.h>

uint32_t frame_count = 0;
uint32_t last_tick = 0;

static uint8_t servo_test_done = 0;

typedef enum {
  SERVO_HEAD = 0,
  SERVO_ARM_L = 1,
  SERVO_ARM_R = 2
} servo_id_t;

// ---- Tune these per servo/mechanics ----
// MG995 typical: ~500-2500us, but safe range often ~1000-2000us.
// Start conservative, widen only if you need more travel.
#define SERVO_MIN_US  1000
#define SERVO_MAX_US  2000

static inline void pcm_q_push(const int16_t *frame)
{
  __disable_irq();
  if (pcm_q_count < PCM_Q_FRAMES) {
    memcpy(pcm_q[pcm_q_w], frame, FRAME_BYTES);
    pcm_q_w = (pcm_q_w + 1) % PCM_Q_FRAMES;
    pcm_q_count++;
  }
  // else: overflow -> drop newest frame (you can choose other policy)
  __enable_irq();
}

static inline int pcm_q_pop(int16_t **frame_out)
{
  int ok = 0;
  __disable_irq();
  if (pcm_q_count > 0) {
    *frame_out = pcm_q[pcm_q_r];
    pcm_q_r = (pcm_q_r + 1) % PCM_Q_FRAMES;
    pcm_q_count--;
    ok = 1;
  }
  __enable_irq();
  return ok;
}

static inline void usb_pump(void)
{
  if (usb_tx_busy) return;

  int16_t *frame;
  if (!pcm_q_pop(&frame)) return;

  // Build header
  tx_pkt[0] = MAGIC0;
  tx_pkt[1] = MAGIC1;
  tx_pkt[2] = (uint8_t)(fc & 0xFF);
  tx_pkt[3] = (uint8_t)(fc >> 8);
  tx_pkt[4] = (uint8_t)(PCM_BYTES & 0xFF);
  tx_pkt[5] = (uint8_t)(PCM_BYTES >> 8);

  memcpy(&tx_pkt[HDR_BYTES], frame, PCM_BYTES);

  if (CDC_Transmit_FS(tx_pkt, PKT_BYTES) == USBD_OK) {
    fc++;
  }
}


int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}


// Channel mapping (your pins: PA6=CH1, PA7=CH2, PB0=CH3)
static inline uint32_t servo_channel(servo_id_t id) {
  switch (id) {
    case SERVO_HEAD:  return TIM_CHANNEL_1;
    case SERVO_ARM_L: return TIM_CHANNEL_2;
    case SERVO_ARM_R: return TIM_CHANNEL_3;
    default:          return TIM_CHANNEL_1;
  }
}

static inline uint16_t clamp_u16(uint16_t x, uint16_t lo, uint16_t hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline uint16_t angle_to_us(uint8_t deg_0_180) {
  // Map 0..180 -> SERVO_MIN_US..SERVO_MAX_US
  // integer math, rounded
  uint32_t span = (uint32_t)(SERVO_MAX_US - SERVO_MIN_US);
  uint32_t us = (uint32_t)SERVO_MIN_US + (span * (uint32_t)deg_0_180 + 90u) / 180u;
  return (uint16_t)us;
}

// Low-level: set pulse width in microseconds (works b/c 1 tick = 1us)
static inline void Servo_SetPulseUS(servo_id_t id, uint16_t us) {
  us = clamp_u16(us, SERVO_MIN_US, SERVO_MAX_US);
  __HAL_TIM_SET_COMPARE(&htim3, servo_channel(id), us);
}

// High-level: set angle 0..180 (clamped)
void Servo_SetAngle(servo_id_t id, uint8_t deg) {
  if (deg > 180) deg = 180;
  Servo_SetPulseUS(id, angle_to_us(deg));
}

// Convenience wrappers
static inline void Head_SetAngle(uint8_t deg)   { Servo_SetAngle(SERVO_HEAD, deg); }
static inline void ArmL_SetAngle(uint8_t deg)   { Servo_SetAngle(SERVO_ARM_L, deg); }
static inline void ArmR_SetAngle(uint8_t deg)   { Servo_SetAngle(SERVO_ARM_R, deg); }

// Optional: smooth move (blocking). Call only when you *want* motion.
// step_deg: 1–5 typical, step_ms: 10–30 typical.
void Servo_MoveToBlocking(servo_id_t id, uint8_t from_deg, uint8_t to_deg,
                          uint8_t step_deg, uint16_t step_ms)
{
  if (from_deg > 180) from_deg = 180;
  if (to_deg > 180) to_deg = 180;
  if (step_deg == 0) step_deg = 1;

  if (to_deg >= from_deg) {
    for (uint16_t d = from_deg; d <= to_deg; d += step_deg) {
      Servo_SetAngle(id, (uint8_t)d);
      HAL_Delay(step_ms);
    }
  } else {
    for (int32_t d = (int32_t)from_deg; d >= (int32_t)to_deg; d -= (int32_t)step_deg) {
      Servo_SetAngle(id, (uint8_t)d);
      HAL_Delay(step_ms);
    }
  }
  Servo_SetAngle(id, to_deg);
}

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
  MX_USART2_UART_Init();
  MX_DFSDM1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_DFSDM_FilterRegularStart_DMA(
      &hdfsdm1_filter0,
      dfsdm_dma,
      FRAME_SAMPLES * 2
  );

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Head
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Left arm
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Right arm

  if(!servo_test_done){
  		  // --- SERVO SANITY TEST: run once then hold ---
  		  	  // Start from neutral
  		  	  Head_SetAngle(90);
  		  	  ArmL_SetAngle(90);
  		  	  ArmR_SetAngle(90);
  		  	  HAL_Delay(800);

  		  	  // Head sweep (small, safe)
  		  	  Head_SetAngle(60);
  		  	  HAL_Delay(800);
  		  	  Head_SetAngle(120);
  		  	  HAL_Delay(800);
  		  	  Head_SetAngle(90);
  		  	  HAL_Delay(800);

  		  	  // Arms up/down
  		  	  ArmL_SetAngle(40);
  		  	  ArmR_SetAngle(40);
  		  	  HAL_Delay(800);

  		  	  ArmL_SetAngle(130);
  		  	  ArmR_SetAngle(130);
  		  	  HAL_Delay(800);

  		  	  ArmL_SetAngle(90);
  		  	  ArmR_SetAngle(90);
  		  	  HAL_Delay(800);
  	  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t do0, do1;

	  __disable_irq();
	  do0 = half0_pending; half0_pending = 0;
	  do1 = half1_pending; half1_pending = 0;
	  __enable_irq();

	  if (do0 == 0 && do1 == 0) {
	    usb_pump();
	    continue;
	  }


	  // Drain in correct time order: half0 then half1
	  while (do0 || do1) {

	      if (do0) {
	          int32_t *src = &dfsdm_dma[0];
	          int16_t *out = scratch0;

	          for (int i = 0; i < FRAME_SAMPLES; i++) {

	              int32_t s = src[i] >> 15;        // DFSDM -> approx int16 range

	              // remove DC (high-pass)
	              dc += (s - dc) >> 6;
	              s = s - dc;

	              // OPTIONAL small gain (comment this out if too loud)
	              s = s << 5;   // gain x2

	              // clip safely
	              if (s > 32767) s = 32767;
	              if (s < -32768) s = -32768;

	              out[i] = (int16_t)s;
	          }

//	          HAL_UART_Transmit(&huart2, (uint8_t*)pcm_frame, FRAME_BYTES, HAL_MAX_DELAY);// FOR UART TRANSMISSION

	          // Stream RAW PCM over USB CDC (non-blocking; drops frame if BUSY)
	          // Stream RAW PCM over USB CDC (non-blocking; drops frame if BUSY)
	          pcm_q_push(out);
	          usb_pump();
	          do0--;
	          frame_count++;
	      }

	      if (do1) {
	          int32_t *src = &dfsdm_dma[FRAME_SAMPLES];

	          int16_t *out = scratch1;

	          for (int i = 0; i < FRAME_SAMPLES; i++) {

	              int32_t s = src[i] >> 15;        // DFSDM -> approx int16 range

	              // remove DC (high-pass)
	              dc += (s - dc) >> 6;
	              s = s - dc;

	              // OPTIONAL small gain (comment this out if too loud)
	              s = s << 5;   // gain x2

	              // clip safely
	              if (s > 32767) s = 32767;
	              if (s < -32768) s = -32768;

	              out[i] = (int16_t)s;
	          }

//	          HAL_UART_Transmit(&huart2, (uint8_t*)pcm_frame, FRAME_BYTES, HAL_MAX_DELAY);// FOR UART TRANSMISSION

	          // Stream RAW PCM over USB CDC (non-blocking; drops frame if BUSY)
	          pcm_q_push(out);
	          usb_pump();

	          do1--;
	          frame_count++;
	      }
	  }

	  // 50 frames/sec check (now correct)
	  uint32_t now = HAL_GetTick();
	  if (now - last_tick >= 1000) {
//		  printf("hello my friend \r\n");
	      if (frame_count != 50) HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	      frame_count = 0;
	      last_tick = now;
//
//	      static const char msg[] = "PING\n";
//	      (void)CDC_Transmit_FS((uint8_t*)msg, sizeof(msg)-1);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = DISABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 128;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
  hdfsdm1_channel0.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel0.Init.OutputClock.Divider = 5;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 1;
  hdfsdm1_channel0.Init.Offset = 0;
  hdfsdm1_channel0.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_0, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */


  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hfilter)
{
    if (hfilter->Instance == DFSDM1_Filter0) half0_pending++;
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hfilter)
{
    if (hfilter->Instance == DFSDM1_Filter0) half1_pending++;
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
