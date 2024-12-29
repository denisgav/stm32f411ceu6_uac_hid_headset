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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"

#include "usb_headset.h"
#include "volume_ctrl.h"
#include "ring_buf.h"
#include "usb_headset_settings.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIC_HI2S hi2s1
#define SPK_HI2S hi2s2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s1;
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi2_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
usb_headset_settings_t current_settings;

// Buffer for speaker data
ring_buf_t spk_ring_buffer;
uint32_t *spk_ring_buffer_storage;

// Buffer for microphone data
ring_buf_t mic_ring_buffer;
uint32_t *mic_ring_buffer_storage;

// Buffer for speaker data
uint8_t spk_usb_read_buf[SAMPLE_BUFFER_SIZE * 2 * 4 * 2]; // Max size of audio sample is  2 * 4. 2 Channels, 4 byte width sample
i2s_32b_audio_sample spk_32b_i2s_buffer[SAMPLE_BUFFER_SIZE];
i2s_16b_audio_sample spk_16b_i2s_buffer[SAMPLE_BUFFER_SIZE];
uint32_t spk_i2s_buf[SAMPLE_BUFFER_SIZE * 2];

// Buffer for microphone data
uint32_t mic_usb_24b_buffer[SAMPLE_BUFFER_SIZE];
uint16_t mic_usb_16b_buffer[SAMPLE_BUFFER_SIZE];
uint8_t mic_usb_read_buf[SAMPLE_BUFFER_SIZE * 2 * 4 * 2]; // Max size of audio sample is  2 * 4. 2 Channels, 4 byte width sample
uint32_t mic_i2s_buf[SAMPLE_BUFFER_SIZE * 2];
uint32_t mic_i2s_read_buffer[SAMPLE_BUFFER_SIZE];
uint32_t num_of_mic_samples;

uint32_t get_num_of_mic_samples();

void led_blinking_task(void);

HAL_StatusTypeDef refresh_i2s_spk(void);
HAL_StatusTypeDef refresh_i2s_mic(void);
HAL_StatusTypeDef refresh_clk(void);
void refresh_i2s_connections(void);

int32_t usb_to_i2s_32b_sample_convert(int32_t sample, uint32_t volume_db);
int16_t usb_to_i2s_16b_sample_convert(int16_t sample, uint32_t volume_db);

void usb_headset_mute_handler(int8_t bChannelNumber, int8_t mute_in);
void usb_headset_volume_handler(int8_t bChannelNumber, int16_t volume_in);
void usb_headset_current_sample_rate_handler(uint32_t current_sample_rate_in);
void usb_headset_current_resolution_handler(uint8_t itf,
		uint8_t current_resolution_in);
void usb_headset_current_status_set_handler(uint8_t itf,
		uint32_t blink_interval_ms_in);

void usb_headset_tud_audio_rx_done_pre_read_handler(uint8_t rhport,
		uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out,
		uint8_t cur_alt_setting);
void usb_headset_tud_audio_tx_done_pre_load_handler(uint8_t rhport, uint8_t itf,
		uint8_t ep_in, uint8_t cur_alt_setting);
void usb_headset_tud_audio_tx_done_post_load_handler(uint8_t rhport,
		uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in,
		uint8_t cur_alt_setting);

int spk_machine_i2s_write_stream(uint32_t *buf_in, uint32_t size);
int mic_machine_i2s_read_stream(uint32_t *buf_in, uint32_t size);

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);
//void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

uint32_t feed_spk_dma(uint32_t *dma_buffer_p,
		uint32_t sizeof_half_dma_buffer_in_bytes);

uint32_t empty_mic_dma(uint32_t *dma_buffer_p,
		uint32_t sizeof_half_dma_buffer_in_words);

void status_update_task(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S1_Init(void);
static void MX_I2S2_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S1_Init();
  MX_I2S2_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	current_settings.spk_sample_rate = I2S_SPK_RATE_DEF;
	current_settings.spk_resolution =
	CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX;
	current_settings.spk_blink_interval_ms = BLINK_NOT_MOUNTED;

	current_settings.mic_resolution =
	CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX;
	current_settings.mic_blink_interval_ms = BLINK_NOT_MOUNTED;
	current_settings.usr_mic_mute = false;

	current_settings.status_updated = false;

	usb_headset_set_mute_set_handler(usb_headset_mute_handler);
	usb_headset_set_volume_set_handler(usb_headset_volume_handler);
	usb_headset_set_current_sample_rate_set_handler(
			usb_headset_current_sample_rate_handler);
	usb_headset_set_current_resolution_set_handler(
			usb_headset_current_resolution_handler);
	usb_headset_set_current_status_set_handler(
			usb_headset_current_status_set_handler);

	usb_headset_set_tud_audio_rx_done_pre_read_set_handler(
			usb_headset_tud_audio_rx_done_pre_read_handler);
	usb_headset_set_tud_audio_tx_done_pre_load_set_handler(
			usb_headset_tud_audio_tx_done_pre_load_handler);
	usb_headset_set_tud_audio_tx_done_post_load_set_handler(
			usb_headset_tud_audio_tx_done_post_load_handler);

	for (int i = 0; i < (CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1); i++) {
		current_settings.spk_volume[i] = DEFAULT_VOLUME;
		current_settings.spk_mute[i] = 0;
		current_settings.spk_volume_db[i] = vol_to_db_convert_enc(
				current_settings.spk_mute[i], current_settings.spk_volume[i]);
	}

	//HAL_GPIO_WritePin(LED_MIC_MUTE_GPIO_Port, LED_MIC_MUTE_Pin, GPIO_PIN_SET);

	usb_headset_init();
	refresh_i2s_connections();

	TU_LOG1("Headset running\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		usb_headset_task(); // TinyUSB device task
		led_blinking_task();
		status_update_task();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 258;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 25;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ONBOARD_LED_Pin */
  GPIO_InitStruct.Pin = ONBOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ONBOARD_LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void refresh_i2s_connections(void) {
	HAL_I2S_StateTypeDef dma_state;
	// Stop speaker DMA
	dma_state = HAL_I2S_GetState(&SPK_HI2S);
	if (dma_state != HAL_I2S_STATE_READY) {
		HAL_StatusTypeDef stop_status = HAL_I2S_DMAStop(&SPK_HI2S);
		if (stop_status != HAL_OK) {
			Error_Handler();
		}
	}

	// Stop microphone DMA
	dma_state = HAL_I2S_GetState(&MIC_HI2S);
	if (dma_state != HAL_I2S_STATE_READY) {
		HAL_StatusTypeDef stop_status = HAL_I2S_DMAStop(&MIC_HI2S);
		if (stop_status != HAL_OK) {
			Error_Handler();
		}
	}

	HAL_StatusTypeDef tx_i2s_status = refresh_i2s_spk();
	if (tx_i2s_status != HAL_OK) {
		Error_Handler();
	}
	HAL_StatusTypeDef rx_i2s_status = refresh_i2s_mic();
	if (rx_i2s_status != HAL_OK) {
		Error_Handler();
	}

	HAL_StatusTypeDef clk_reconfig_status = refresh_clk();
	if (clk_reconfig_status != HAL_OK) {
		Error_Handler();
	}

	if (spk_ring_buffer_storage != NULL) {
		free(spk_ring_buffer_storage);
	}
	if (mic_ring_buffer_storage != NULL) {
		free(mic_ring_buffer_storage);
	}

	current_settings.samples_in_i2s_frame_min =
			(current_settings.spk_sample_rate) / 1000;
	current_settings.samples_in_i2s_frame_max =
			(current_settings.spk_sample_rate + 999) / 1000;

	// Ring buffer contains 2 ms data
	spk_ring_buffer_storage = m_new(uint32_t,
			current_settings.samples_in_i2s_frame_max * 2 * 2);
	ringbuf_init(&spk_ring_buffer, spk_ring_buffer_storage,
			current_settings.samples_in_i2s_frame_max * 2 * 2);

	mic_ring_buffer_storage = m_new(uint32_t,
			current_settings.samples_in_i2s_frame_max * 2 * 2);
	ringbuf_init(&mic_ring_buffer, mic_ring_buffer_storage,
			current_settings.samples_in_i2s_frame_max * 2 * 2);

	// Clear memories
	memset(spk_i2s_buf, 0x0, sizeof(spk_i2s_buf));
	memset(mic_i2s_buf, 0x0, sizeof(mic_i2s_buf));
	memset(mic_usb_24b_buffer, 0x0, sizeof(mic_usb_24b_buffer));
	memset(mic_usb_16b_buffer, 0x0, sizeof(mic_usb_16b_buffer));

	uint16_t num_of_samples_in_buffer =
			current_settings.samples_in_i2s_frame_min * 2;

	// Run transmit DMA
	HAL_StatusTypeDef tx_status = HAL_I2S_Transmit_DMA(&SPK_HI2S, spk_i2s_buf,
			num_of_samples_in_buffer);
	if (tx_status != HAL_OK) {
		Error_Handler();
	}

	HAL_StatusTypeDef rx_status = HAL_I2S_Receive_DMA(&MIC_HI2S, mic_i2s_buf,
			num_of_samples_in_buffer);
	if (rx_status != HAL_OK) {
		Error_Handler();
	}
}

HAL_StatusTypeDef refresh_i2s_spk(void) {
	HAL_StatusTypeDef status = HAL_I2S_DeInit(&SPK_HI2S);
	if (status != HAL_OK)
		return status;

	SPK_HI2S.Instance = SPI2;
	SPK_HI2S.Init.Mode = I2S_MODE_MASTER_TX;
	SPK_HI2S.Init.Standard = I2S_STANDARD_PHILIPS;
	SPK_HI2S.Init.DataFormat =
			(current_settings.spk_resolution == 16) ?
			I2S_DATAFORMAT_16B :
														I2S_DATAFORMAT_32B; //I2S_DATAFORMAT_32B;
	SPK_HI2S.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	SPK_HI2S.Init.AudioFreq = current_settings.spk_sample_rate; //I2S_AUDIOFREQ_48K;
	SPK_HI2S.Init.CPOL = I2S_CPOL_LOW;
	SPK_HI2S.Init.ClockSource = I2S_CLOCK_PLL;
	SPK_HI2S.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&SPK_HI2S) != HAL_OK) {
		Error_Handler();
	}

	return HAL_OK;
}

HAL_StatusTypeDef refresh_i2s_mic(void) {
	HAL_StatusTypeDef status = HAL_I2S_DeInit(&MIC_HI2S);
	if (status != HAL_OK)
		return status;

	MIC_HI2S.Instance = SPI1;
	MIC_HI2S.Init.Mode = I2S_MODE_MASTER_RX;
	MIC_HI2S.Init.Standard = I2S_STANDARD_PHILIPS;
	MIC_HI2S.Init.DataFormat = I2S_DATAFORMAT_32B;
	MIC_HI2S.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	MIC_HI2S.Init.AudioFreq = current_settings.spk_sample_rate; //I2S_AUDIOFREQ_48K;
	MIC_HI2S.Init.CPOL = I2S_CPOL_LOW;
	MIC_HI2S.Init.ClockSource = I2S_CLOCK_PLL;
	MIC_HI2S.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&MIC_HI2S) != HAL_OK) {
		Error_Handler();
	}

	return HAL_OK;
}

HAL_StatusTypeDef refresh_clk(void) {
//	typedef struct I2S_CLK_CONFIG_ {
//		uint32_t N;
//		uint32_t R;
//		uint32_t I2SDIV;
//		uint32_t ODD;
//		uint32_t nominal_fdbk;
//	} I2S_CLK_CONFIG;
//	const I2S_CLK_CONFIG I2S_Clk_Config24[3]  = {
//	{429, 4, 19, 0, 0x0B065E56}, // 44.0995
//	{384, 5, 12, 1, 0x0C000000}, // 48.0000
//	{424, 3, 11, 1, 0x1800ED70}  // 96.0144
//	};
//

//	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	if (current_settings.spk_sample_rate == 48000) {
//		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//		RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//		RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//		RCC_OscInitStruct.PLL.PLLM = 25;
//		RCC_OscInitStruct.PLL.PLLN = 336;
//		RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
//		RCC_OscInitStruct.PLL.PLLQ = 7;
//
//		HAL_StatusTypeDef OscConfig_stat = HAL_RCC_OscConfig(&RCC_OscInitStruct);
//		if(OscConfig_stat != HAL_OK){
//			return OscConfig_stat;
//		}

		/** Initializes the peripherals clock
		 */
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
		PeriphClkInitStruct.PLLI2S.PLLI2SN = 258;
		PeriphClkInitStruct.PLLI2S.PLLI2SM = 25;
		PeriphClkInitStruct.PLLI2S.PLLI2SR = 3;
		HAL_StatusTypeDef PeriphCLKConfig_stat = HAL_RCCEx_PeriphCLKConfig(
				&PeriphClkInitStruct);
		if (PeriphCLKConfig_stat != HAL_OK) {
			Error_Handler();
		}

		return HAL_OK;
	}

	if (current_settings.spk_sample_rate == 44100) {
//		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//		RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//		RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//		RCC_OscInitStruct.PLL.PLLM = 25;
//		RCC_OscInitStruct.PLL.PLLN = 336;
//		RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
//		RCC_OscInitStruct.PLL.PLLQ = 7;
//
//		HAL_StatusTypeDef OscConfig_stat = HAL_RCC_OscConfig(&RCC_OscInitStruct);
//		if(OscConfig_stat != HAL_OK){
//			return OscConfig_stat;
//		}

		/** Initializes the peripherals clock
		 */
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
		PeriphClkInitStruct.PLLI2S.PLLI2SN = 271;
		PeriphClkInitStruct.PLLI2S.PLLI2SM = 25;
		PeriphClkInitStruct.PLLI2S.PLLI2SR = 6;
		HAL_StatusTypeDef PeriphCLKConfig_stat = HAL_RCCEx_PeriphCLKConfig(
				&PeriphClkInitStruct);
		if (PeriphCLKConfig_stat != HAL_OK) {
			return PeriphCLKConfig_stat;
		}

		return HAL_OK;
	}

	return HAL_ERROR;
}

void usb_headset_mute_handler(int8_t bChannelNumber, int8_t mute_in) {
	current_settings.spk_mute[bChannelNumber] = mute_in;
	current_settings.spk_volume_db[bChannelNumber] = vol_to_db_convert_enc(
			current_settings.spk_mute[bChannelNumber],
			current_settings.spk_volume[bChannelNumber]);

	current_settings.spk_volume_mul_db[0] = current_settings.spk_volume_db[0]
			* current_settings.spk_volume_db[1]; // Left ch volume
	current_settings.spk_volume_mul_db[1] = current_settings.spk_volume_db[0]
			* current_settings.spk_volume_db[2]; // Right ch volume

	current_settings.status_updated = true;
}

void usb_headset_volume_handler(int8_t bChannelNumber, int16_t volume_in) {
	current_settings.spk_volume[bChannelNumber] = volume_in;
	current_settings.spk_volume_db[bChannelNumber] = vol_to_db_convert_enc(
			current_settings.spk_mute[bChannelNumber],
			current_settings.spk_volume[bChannelNumber]);

	current_settings.spk_volume_mul_db[0] = current_settings.spk_volume_db[0]
			* current_settings.spk_volume_db[1]; // Left ch volume
	current_settings.spk_volume_mul_db[1] = current_settings.spk_volume_db[0]
			* current_settings.spk_volume_db[2]; // Right ch volume

	current_settings.status_updated = true;
}

void usb_headset_current_sample_rate_handler(uint32_t current_sample_rate_in) {
	current_settings.spk_sample_rate = current_sample_rate_in;
	refresh_i2s_connections();
	current_settings.status_updated = true;
}

void usb_headset_current_resolution_handler(uint8_t itf,
		uint8_t current_resolution_in) {
	if ((itf == 0) || (itf == ITF_NUM_AUDIO_STREAMING_SPK))
		current_settings.spk_resolution = current_resolution_in;
	if ((itf == 0) || (itf == ITF_NUM_AUDIO_STREAMING_MIC))
		current_settings.mic_resolution = current_resolution_in;

	refresh_i2s_connections();
	current_settings.status_updated = true;
}

void usb_headset_current_status_set_handler(uint8_t itf,
		uint32_t blink_interval_ms_in) {
	if ((itf == 0) || (itf == ITF_NUM_AUDIO_STREAMING_SPK))
		current_settings.spk_blink_interval_ms = blink_interval_ms_in;
	if ((itf == 0) || (itf == ITF_NUM_AUDIO_STREAMING_MIC))
		current_settings.mic_blink_interval_ms = blink_interval_ms_in;
	current_settings.status_updated = true;
}

void usb_headset_tud_audio_rx_done_pre_read_handler(uint8_t rhport,
		uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out,
		uint8_t cur_alt_setting) {
	uint32_t volume_db_left = current_settings.spk_volume_mul_db[0];
	uint32_t volume_db_right = current_settings.spk_volume_mul_db[1];

	if (current_settings.spk_blink_interval_ms == BLINK_STREAMING) {
		// Speaker data size received in the last frame
		uint16_t usb_spk_data_size = tud_audio_n_read(func_id, spk_usb_read_buf,
				n_bytes_received);
		//uint16_t usb_spk_data_size = tud_audio_read(spk_usb_read_buf,
		//		n_bytes_received);
		uint16_t usb_sample_count = 0;

		if (current_settings.spk_resolution == 16) {
			int16_t *in = (int16_t*) spk_usb_read_buf;
			usb_sample_count = usb_spk_data_size / 4; // 4 bytes per sample 2b left, 2b right

			//if (usb_sample_count >= current_settings.samples_in_i2s_frame_min) {
			for (int i = 0; i < usb_sample_count; i++) {
				int16_t left = in[i * 2 + 0];
				int16_t right = in[i * 2 + 1];

				left = usb_to_i2s_16b_sample_convert(left, volume_db_left);
				right = usb_to_i2s_16b_sample_convert(right, volume_db_right);

				spk_16b_i2s_buffer[i].left = left;
				spk_16b_i2s_buffer[i].right = right;
			}
			spk_machine_i2s_write_stream(&spk_16b_i2s_buffer[0],
					usb_sample_count); // Number of words
			//}
		} else if (current_settings.spk_resolution == 24) {
			int32_t *in = (int32_t*) spk_usb_read_buf;
			usb_sample_count = usb_spk_data_size / 8; // 8 bytes per sample 4b left, 4b right

			//if (usb_sample_count >= current_settings.samples_in_i2s_frame_min) {
			for (int i = 0; i < usb_sample_count; i++) {
				int32_t left = in[i * 2 + 0];
				int32_t right = in[i * 2 + 1];

				left = usb_to_i2s_32b_sample_convert(left, volume_db_left);
				right = usb_to_i2s_32b_sample_convert(right, volume_db_right);

				spk_32b_i2s_buffer[i].left = left;
				spk_32b_i2s_buffer[i].right = right;
			}
			spk_machine_i2s_write_stream(&spk_32b_i2s_buffer[0],
					usb_sample_count * 2); // Number of words
			//}
		}
	}
}

uint32_t get_num_of_mic_samples() {
	static uint32_t format_44100_khz_counter = 0;
	if (current_settings.spk_sample_rate == 44100) {
		format_44100_khz_counter++;
		if (format_44100_khz_counter >= 9) {
			format_44100_khz_counter = 0;
			return 45;
		} else {
			return 44;
		}
	} else {
		return current_settings.samples_in_i2s_frame_min;
	}
}

void usb_headset_tud_audio_tx_done_pre_load_handler(uint8_t rhport, uint8_t itf,
		uint8_t ep_in, uint8_t cur_alt_setting) {
	if (current_settings.mic_blink_interval_ms == BLINK_STREAMING) {
		if (current_settings.mic_resolution == 24) {
			uint32_t buffer_size = num_of_mic_samples
					* CFG_TUD_AUDIO_FUNC_1_FORMAT_2_N_BYTES_PER_SAMPLE_TX
					* CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX;
			uint16_t bytes_writen = tud_audio_write(&(mic_usb_24b_buffer[0]),
					buffer_size);
		} else {
			uint32_t buffer_size = num_of_mic_samples
					* CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX
					* CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX;
			uint16_t bytes_writen = tud_audio_write(&(mic_usb_16b_buffer[0]),
					buffer_size);
		}
	}
}

void usb_headset_tud_audio_tx_done_post_load_handler(uint8_t rhport,
		uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in,
		uint8_t cur_alt_setting) {

	if (current_settings.mic_blink_interval_ms != BLINK_STREAMING) {
		return;
	}

	if (current_settings.usr_mic_mute == true) {
		if (current_settings.mic_resolution == 24) {
			memset(mic_usb_24b_buffer, 0x0, sizeof(mic_usb_24b_buffer));
		} else {
			memset(mic_usb_16b_buffer, 0x0, sizeof(mic_usb_16b_buffer));
		}
		return;
	}

	// Read data from microphone
	num_of_mic_samples = get_num_of_mic_samples();
	uint32_t buffer_size_read = num_of_mic_samples;
	int num_words_read = mic_machine_i2s_read_stream(&mic_i2s_read_buffer[0],
			buffer_size_read);
	int num_of_frames_read = num_words_read;

	if (num_of_frames_read >= current_settings.samples_in_i2s_frame_min) {
		for (uint32_t i = 0; i < num_of_frames_read; i++) {
			if (current_settings.mic_resolution == 24) {
				int32_t mono_24b = (int32_t) mic_i2s_read_buffer[i]
						<< MIC_FORMAT_24B_TO_24B_SHIFT_VAL; // Magic number

				mic_usb_24b_buffer[i] = mono_24b; // TODO: check this value
			} else {
				int32_t mono_24b = (int32_t) mic_i2s_read_buffer[i]
						>> MIC_FORMAT_24B_TO_16B_SHIFT_VAL; // Magic number

				mic_usb_16b_buffer[i] = mono_24b; // TODO: check this value
			}
		}
	} else {
		if (current_settings.mic_resolution == 24) {
			memset(mic_usb_24b_buffer, 0x0, sizeof(mic_usb_24b_buffer));
		} else {
			memset(mic_usb_16b_buffer, 0x0, sizeof(mic_usb_16b_buffer));
		}
	}
}

int32_t usb_to_i2s_32b_sample_convert(int32_t sample, uint32_t volume_db) {
	int64_t sample_tmp = (int64_t) sample * (int64_t) volume_db;
	sample_tmp = sample_tmp >> 30;
	return (int32_t) sample_tmp;
//	return (int32_t)sample;
}

int16_t usb_to_i2s_16b_sample_convert(int16_t sample, uint32_t volume_db) {
	int64_t sample_tmp = (int64_t) sample * (int64_t) volume_db;
	sample_tmp = sample_tmp >> 30;
	return (int16_t) sample_tmp;
//	return (int16_t)sample;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
	static uint32_t start_ms = 0;
	static bool led_state = false;

	uint32_t cur_time_ms = board_millis();

	// Blink every interval ms
	if (cur_time_ms - start_ms < current_settings.spk_blink_interval_ms)
		return;
	start_ms += current_settings.spk_blink_interval_ms;

	board_led_write(led_state);
	led_state = 1 - led_state;
}

int spk_machine_i2s_write_stream(uint32_t *buf_in, uint32_t size) {
	if (size == 0) {
		return 0;
	}

	// copy audio samples from the app buffer to the ring buffer
	// loop, reading samples until the app buffer is emptied
	// for uasyncio mode, the loop will make an early exit if the ring buffer becomes full

	// Not allow buffer overflow
	uint16_t available_space = ringbuf_available_space(&spk_ring_buffer);
	if (available_space <= size) {
		return 0;
	}

	if (current_settings.spk_resolution == 24) {
		for (uint32_t a_index = 0; a_index < size; a_index++) {
			if (ringbuf_push_half_word_swap(&spk_ring_buffer,
					buf_in[a_index]) == false) {
				return a_index;
			}
		}
	} else {
		for (uint32_t a_index = 0; a_index < size; a_index++) {
			if (ringbuf_push(&spk_ring_buffer, buf_in[a_index]) == false) {
				return a_index;
			}
		}
	}

	return size;
}

int mic_machine_i2s_read_stream(uint32_t *buf_in, uint32_t size) {
	if (size == 0) {
		return 0;
	}

	uint32_t available_data_words = ringbuf_available_data(&mic_ring_buffer);

	if (available_data_words >= size) {
		for (uint32_t i = 0; i < size; i++) {
			if (ringbuf_pop_half_word_swap(&mic_ring_buffer,
					&buf_in[i]) == false) {
				return i;
			}
		}
		return size;
	} else {
		// underflow.  clear buffer to transmit "silence" on the I2S bus
		memset(buf_in, 0, size * sizeof(uint32_t));
		return size;
	}
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	int num_of_words_feed =
			(current_settings.spk_resolution == 24) ?
					current_settings.samples_in_i2s_frame_min :
					current_settings.samples_in_i2s_frame_min / 2;
	feed_spk_dma(&spk_i2s_buf[0], num_of_words_feed);
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
	int num_of_words_feed =
			(current_settings.spk_resolution == 24) ?
					current_settings.samples_in_i2s_frame_min :
					current_settings.samples_in_i2s_frame_min / 2;
	feed_spk_dma(&spk_i2s_buf[num_of_words_feed], num_of_words_feed);
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	int num_of_words_empty = current_settings.samples_in_i2s_frame_min;
	empty_mic_dma(&mic_i2s_buf[0], num_of_words_empty);
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
	int num_of_words_empty = current_settings.samples_in_i2s_frame_min;
	empty_mic_dma(&mic_i2s_buf[num_of_words_empty], num_of_words_empty);
}

/**
 * @brief  EXTI line detection callbacks.
 * @param  GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// No interrupts are connected...
	// Nothing to do here

//	static uint32_t prev_btb_press__ms = 0;
//
//	uint32_t cur_time_ms = board_millis();
//
//	if (cur_time_ms - prev_btb_press__ms < 50)
//		return;
//
//	prev_btb_press__ms = cur_time_ms;
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) {
	refresh_i2s_connections();
}

uint32_t feed_spk_dma(uint32_t *dma_buffer_p,
		uint32_t sizeof_half_dma_buffer_in_words) {
	// when data exists, copy samples from ring buffer
	uint32_t available_data_words = ringbuf_available_data(&spk_ring_buffer);
	if (available_data_words > sizeof_half_dma_buffer_in_words) {
		available_data_words = sizeof_half_dma_buffer_in_words;
	}

	if (available_data_words >= sizeof_half_dma_buffer_in_words) {
		for (uint32_t i = 0; i < sizeof_half_dma_buffer_in_words; i++) {
			ringbuf_pop(&spk_ring_buffer, &dma_buffer_p[i]);
		}
		return available_data_words;
	} else {
		// underflow.  clear buffer to transmit "silence" on the I2S bus
		memset(dma_buffer_p, 0,
				sizeof_half_dma_buffer_in_words * sizeof(uint32_t));
		return sizeof_half_dma_buffer_in_words;
	}
}

uint32_t empty_mic_dma(uint32_t *dma_buffer_p,
		uint32_t sizeof_half_dma_buffer_in_words) {
	// when space exists, copy samples into ring buffer
	if (ringbuf_available_space(&mic_ring_buffer)
			>= sizeof_half_dma_buffer_in_words) {
		for (uint32_t i = 0; i < sizeof_half_dma_buffer_in_words; i += 2) {
			if (ringbuf_push(&mic_ring_buffer, dma_buffer_p[i]) == false)
				return i;
		}
	}
	return sizeof_half_dma_buffer_in_words;
}

//--------------------------------------------------------------------+
// STATUS UPDATE TASK
//--------------------------------------------------------------------+
void status_update_task(void) {
	static uint32_t prev_status_update__ms = 0;

	uint32_t cur_time_ms = board_millis();

	// Update status once per second
	if (cur_time_ms - prev_status_update__ms < 100)
		return;

	prev_status_update__ms = cur_time_ms;

	if (current_settings.status_updated == true) {
		current_settings.status_updated = false;
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
	while (1) {
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
