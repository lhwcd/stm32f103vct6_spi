/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "ubx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PRINT_ERROR(fmt, args...)  LOG_ERROR("[MAIN]%s Line %d: "fmt, my_basename(__FILE__), __LINE__, ##args)
#define PRINT_WARN(fmt, args...)   LOG_WARN("[MAIN]%s Line %d: "fmt, my_basename(__FILE__), __LINE__, ##args)
#define PRINT_INFO(fmt, args...)   LOG_INFO("[MAIN]%s Line %d: "fmt, my_basename(__FILE__), __LINE__, ##args)
#define PRINT_DEBUG(fmt, args...)  LOG_DEBUG("[MAIN]%s Line %d: "fmt, my_basename(__FILE__), __LINE__, ##args)
#define PRINT_BIN(fmt, args...)    LOG_BINARY("[MAIN]%s Line %d: "fmt, my_basename(__FILE__), __LINE__, ##args)

// 接收ubx数据的串口buffer
#define UART_RX_MAX_LEN (3000u)
// 时区设置为 +0
#define TIME_ZONE 0
// 震动数据接收缓存，每个点9byte， 收满500个点发送一次
// 200是为添加的包头和校验码留的空间
#define  ARRAYSIZE         (XL355_SAMPLE_COUNT + 200)
typedef enum {
  LED_ON = 1,
  LED_OFF= 2,
  LED_TOGGLE = 3
} led_ctl_t;

typedef enum {
  SYNC_LED = 1,
  WORK_LED = 2
} led_type_t;

typedef enum {
  SYNC_STEP0 = 0,
  SYNC_STEP1 = 1,
  SYNC_STEP2 = 2,
  SYNC_STEP3 = 3,
  SYNC_STEP4 = 4
} sync_step_t;

typedef enum
{
    NMEA_UNKNOWN = -1,
    NMEA_GGA = 0,
    NMEA_GSV,
    NMEA_RMC,
    NMEA_ZDA
}nmea_msg_type_t;

typedef struct {
  uint16_t time_5ms_count;
  uint16_t time_1_sec_count;
  uint16_t pps_count;
  bool pps_flag;
  bool xl355_sync_flag;
  uint8_t start_sync_flag;
  bool nmea_flag;
  uint32_t unix_timestamp_second;
  uint32_t unix_timestamp_micro_second;
} sys_run_mark_t;

typedef struct
{
    uint32_t buffer_len;
    uint8_t  buffer[UART_RX_MAX_LEN];
} uart_rx_t;

typedef struct
{
    uint8_t utc_hh;
    uint8_t utc_mm;
    uint8_t utc_ss;
    uint8_t utc_DD;
    uint8_t utc_MM;
    uint8_t utc_YY;
}gps_time_t;

typedef struct
{
    char    lat_dir;         // 纬度方向(N:北纬,S:南纬)
    char    lon_dir;         // 经度方向(E:东经,W:西经)
    char    var_dir;          // 磁偏角方向,E/W
    char    pos_status;      // 定位状态:A=有效定位, V=无效定位
    char    mode_ind;        // 定位模式指示
    char    a_units;         // 天线高单位,m
    uint8_t gps_qual;        // 解状态, 1:GPS 定位; 4:RTK 固定解; 5:RTK 浮点解
    uint8_t compute_sats;    // 参与计算的卫星数
    uint8_t gp_obs_sats;     // gps 卫星数
    uint8_t bd_obs_sats;     // bd  卫星数
    uint8_t gl_obs_sats;     // glo 卫星数
    uint8_t ga_obs_sats;     // gal 卫星书
    float   alt;             // 天线高度(海平面以上或以下)
    float   speed;           // 地面速率
    float   track_true;      // 地面航向,以真北方向为基准
    float   mag_var;         // 磁偏角(000-180.0°)
    double  lat;             // 纬度
    double  lon;             // 经度
}position_info_t;

typedef struct
{
    gps_time_t utc_time;
    position_info_t pos;
}gnss_info_t;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
uint32_t int_device_serial[3];
uint32_t int_device_memery_info;

// 接收震动数据的buffer
uint8_t spi_send_array[ARRAYSIZE] = {0};
// 发送震动数据的buffer
uint8_t spi_send_array_bak[ARRAYSIZE] = {0};
// 从xl355寄存器读一次温度传感器和 xyz 3轴的震动数据用的buffer
uint8_t spi_read_reg_array[12] = {0}; 

int xl355_fifo_full_flag = 0;

// 用于暂存打印日志的buffer
int msg_len = 0;
uint8_t print_msg[256] = {0};

// 处理ubx的nmea消息的buffer
uint8_t ubx_buff[UART_RX_MAX_LEN] = {0};
uart_rx_t ubx_rx_buffer;
// 运行过程中的参数和标志
sys_run_mark_t sys_run;
// 从ubx解析出来的信息，包括经纬度，时间，卫星数
gnss_info_t gs_gnss_info;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void get_chip_serial_num(void);
void get_chip_memery_info(void);
void adxl355_init(uint8_t range, uint8_t odr, uint8_t fifo_len, uint8_t sync_set);
int adxl355_start_work(void);
float adxl355_conversion_acc_data(uint8_t *data);
float adxl355_conversion_temperature(uint8_t *data);
uint32_t adler32(uint8_t *buf, uint32_t len);
uint8_t spi_read_byte(uint8_t addr);
int spi_write_byte(uint8_t addr, uint8_t data);
int spi_write_multipe_bytes(uint8_t start_addr, uint8_t *txdata, uint8_t len);
int spi_read_multipe_bytes(uint8_t start_addr, uint8_t *rxdata, uint8_t len);
char *my_basename(char *s);
int log_print(void);
void HAL_RTC_GetTime_and_Date(RTC_DateTypeDef *s_date, RTC_TimeTypeDef *s_time);
void HAL_RTC_SetTime_and_Date(RTC_DateTypeDef *s_data, RTC_TimeTypeDef *s_time);
int led_ctl(led_type_t name, led_ctl_t on_off);
int ubx_reset(void);
int ubx_init(void);
int ubx_msg_generate(uint8_t msg_class, uint8_t msg_id, const unsigned char *data, uint16_t data_len, uint8_t *buf, uint16_t buf_len);
/*int xl355_resync(void)*/
int parse_nmea_message(uint8_t *nmea_msg);
int package_xl355_raw_data(uint8_t *in_buf, uint8_t *out_buff, int data_len);
uint32_t get_unix_timestamp(void);
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
  int i;
  int count = 0; // 接收数据计数，满4500清零（10*50*9）
  int package_len = 0;
	HAL_StatusTypeDef ret;
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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  led_ctl(WORK_LED, LED_ON);
  led_ctl(SYNC_LED, LED_ON);
  PRINT_INFO("start stm32f103vct6\r\n");
	
	/* get chip serial number */
	get_chip_serial_num();

	/* printf CPU unique device id */
  PRINT_INFO("The CPU Unique Device ID:[%X-%X-%X]\r\n", int_device_serial[2], int_device_serial[1], int_device_serial[0]);

	/* get chip memery info */
	get_chip_memery_info();

	/* printf sdram and flash size */
	PRINT_INFO("The Flash size:%uKBytes, SRAM size:%uKBytes\r\n", (int_device_memery_info & 0xFFFF), (int_device_memery_info >> 16 & 0xFFFF));

  ubx_reset();
  HAL_Delay(100);  // wait for UBX start work
	ubx_init();

  adxl355_init(XL355_RANGE_2G, XL355_ODR_2000HZ, XL355_FIFO_SAMPLE_90, XL355_EXT_SYNC01);
  adxl355_start_work();

  log_print();

  memset(&sys_run, 0, sizeof(sys_run_mark_t));
  memset(spi_send_array, 0x0, ARRAYSIZE);
  xl355_fifo_full_flag = 0;
  count = 0;

  memset(&ubx_rx_buffer, 0x0, sizeof(uart_rx_t));
  memset(&gs_gnss_info, 0x0, sizeof(gnss_info_t));
  HAL_UART_Receive_DMA(&huart2, ubx_rx_buffer.buffer, UART_RX_MAX_LEN - 1);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	
	while(1)
	{
		if (xl355_fifo_full_flag > 0)
    {
      // memset(spi_read_reg_array, 0x0, 12);
      // ret = spi_read_multipe_bytes(XL355_TEMP2, &spi_read_reg_array[0], 11);

      // PRINT_DEBUG("adxl355: temp:%0.2f\t\taccx:%0.2f\t\taccy:%0.2f\t\taccz:%0.2f\r\n", \
			// 					adxl355_conversion_temperature(&spi_read_reg_array[0]), adxl355_conversion_acc_data(&spi_read_reg_array[2]), \
			// 					adxl355_conversion_acc_data(&spi_read_reg_array[5]), adxl355_conversion_acc_data(&spi_read_reg_array[8]));
     
      ret = spi_read_multipe_bytes(XL355_FIFO_DATA, &spi_send_array[count], XL355_FIFO_SAMPLE_90);

      // for (i = (count+2); i < (count + XL355_FIFO_SAMPLE_90); i+=9)
      // {
      //     if ((spi_send_array[i] & 0x01) == 0x01)
      //     {
      //         PRINT_DEBUG("adxl355: %d\t\taccx%0.2f\t\taccy:%0.2f\t\taccz:%0.2f\r\n", i, \
			// 				    adxl355_conversion_acc_data(&spi_send_array[i-2]), adxl355_conversion_acc_data(&spi_send_array[i+1]), adxl355_conversion_acc_data(&spi_send_array[i+4]));
      //     }
      // } 
			
      // PRINT_DEBUG("xl355_fifo_full_flag: %d\tadxl355_%04d\t\taccx%0.2f\t\taccy:%0.2f\t\taccz:%0.2f\r\n", xl355_fifo_full_flag, count, \
      //             adxl355_conversion_acc_data(&spi_send_array[count]), \
      //             adxl355_conversion_acc_data(&spi_send_array[count+3]), \
      //             adxl355_conversion_acc_data(&spi_send_array[count+6]));

			count += XL355_FIFO_SAMPLE_90;
			if (count >= XL355_SAMPLE_COUNT)
			{
        // send data
				memset(spi_send_array_bak, 0x0, ARRAYSIZE);
        package_len = package_xl355_raw_data(spi_send_array, spi_send_array_bak, count);

        if (package_len > 0)
					HAL_UART_Transmit_DMA(&huart3, spi_send_array_bak, package_len);
        
				memset(spi_send_array, 0x0, ARRAYSIZE);
        count = 0 ;

        sys_run.unix_timestamp_second =  get_unix_timestamp();
        sys_run.unix_timestamp_micro_second =  (sys_run.time_5ms_count % 200) * 5 * 1000;

        // PRINT_DEBUG("time:%u.%06u send %d bytes to uart\r\n", sys_run.unix_timestamp_second, sys_run.unix_timestamp_micro_second, package_len);
        PRINT_DEBUG("xl355_fifo_full_flag: %d\tadxl355_%04d\t\taccx%0.2f\t\taccy:%0.2f\t\taccz:%0.2f\r\n", xl355_fifo_full_flag, count, \
            adxl355_conversion_acc_data(&spi_send_array_bak[15]), \
            adxl355_conversion_acc_data(&spi_send_array_bak[18]), \
            adxl355_conversion_acc_data(&spi_send_array_bak[21]));
			}
			xl355_fifo_full_flag = 0;
    }

    // 秒脉冲处理
    if (sys_run.pps_flag == true)
    {
      sys_run.pps_flag = false;
      led_ctl(WORK_LED, LED_TOGGLE); // 翻转 LED工作灯

      // check if we have done sync
      if (sys_run.xl355_sync_flag == false)
      {
        if (sys_run.pps_count == 4)
        {
          // start do sync
          sys_run.start_sync_flag = SYNC_STEP1; // step 1
          sys_run.time_5ms_count = 0;
          PRINT_DEBUG("SYNC_STEP1......\r\n");
        }
        else if (sys_run.pps_count == 5)
        {
          // syncing
          sys_run.start_sync_flag = SYNC_STEP3; // step 3
          PRINT_DEBUG("SYNC_STEP3......\r\n");
          spi_read_multipe_bytes(XL355_FIFO_DATA, &spi_send_array[0], XL355_FIFO_SAMPLE_90);
          memset(spi_send_array, 0x0, ARRAYSIZE); // data is synced after pps count == 5
          sys_run.time_5ms_count = 0; // time_5ms_count must be 0 after sync
          sys_run.unix_timestamp_second =  get_unix_timestamp();
          sys_run.unix_timestamp_micro_second =  (sys_run.time_5ms_count % 200) * 5 * 1000;
          count = 0;
        }
      }
    }

    if (sys_run.xl355_sync_flag == false)
    {
      if ((sys_run.start_sync_flag == SYNC_STEP0 || sys_run.start_sync_flag == SYNC_STEP2) \
            && sys_run.time_5ms_count % 100 == 1)
      {
        led_ctl(SYNC_LED, LED_TOGGLE); // 翻转 同步LED灯
      }
      else if (sys_run.start_sync_flag == SYNC_STEP1 && sys_run.time_5ms_count > 120) // about 600ms
      {      
        // enable xl355 sync
        HAL_GPIO_WritePin(XL355_SYNC_GPIO_Port, XL355_SYNC_Pin, GPIO_PIN_SET);
        sys_run.start_sync_flag = SYNC_STEP2;     // step 2
        PRINT_DEBUG("SYNC_STEP2......\r\n");
      }
      else if (sys_run.start_sync_flag == SYNC_STEP3 && sys_run.time_5ms_count > 100) // about 500ms
      {
        // disable xl355 sync
        HAL_GPIO_WritePin(XL355_SYNC_GPIO_Port, XL355_SYNC_Pin, GPIO_PIN_RESET);
        led_ctl(SYNC_LED, LED_ON);
        sys_run.xl355_sync_flag = true;
        sys_run.start_sync_flag = SYNC_STEP4;

        PRINT_DEBUG("SYNC_STEP4...... finish sync\r\n");
      }
    }

    if (sys_run.nmea_flag == true)
    {
      sys_run.nmea_flag = false;
      parse_nmea_message(ubx_buff);
    }
    if (sys_run.time_5ms_count >= 60000) // 5 minutes
    {
      sys_run.time_5ms_count = 0;
    }
    log_print();
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == 0x5A5A)
  {
     return;
  }
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x5A5A);
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart1.Init.BaudRate = 2000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 2000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_INT_GPIO_Port, SPI3_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_SYNC_Pin|LED_RUN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, WATCHDOG_Pin|SC200R_RST_Pin|XL355_SYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SC200R_EN_GPIO_Port, SC200R_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(XL355_EN_GPIO_Port, XL355_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SC200R_PW_EN_GPIO_Port, SC200R_PW_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SPI3_INT_Pin */
  GPIO_InitStruct.Pin = SPI3_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI3_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE3 PE6 PE7 PE8
                           PE9 PE10 PE11 PE12
                           PE13 PE14 PE15 PE0
                           PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_SYNC_Pin */
  GPIO_InitStruct.Pin = LED_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_SYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RUN_Pin */
  GPIO_InitStruct.Pin = LED_RUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RUN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC0 PC1 PC3
                           PC8 PC9 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : WATCHDOG_Pin SC200R_EN_Pin SC200R_RST_Pin */
  GPIO_InitStruct.Pin = WATCHDOG_Pin|SC200R_EN_Pin|SC200R_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : XL355_SPI_CS_Pin */
  GPIO_InitStruct.Pin = XL355_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(XL355_SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB14 PB15 PB6
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS_EN_Pin GPS_RST_Pin */
  GPIO_InitStruct.Pin = GPS_EN_Pin|GPS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15
                           PD0 PD1 PD4 PD5
                           PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : XL355_INT1_Pin */
  GPIO_InitStruct.Pin = XL355_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(XL355_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : XL355_SYNC_Pin */
  GPIO_InitStruct.Pin = XL355_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(XL355_SYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_1PPS_Pin */
  GPIO_InitStruct.Pin = GPS_1PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPS_1PPS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : XL355_EN_Pin */
  GPIO_InitStruct.Pin = XL355_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(XL355_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SC200R_PW_EN_Pin */
  GPIO_InitStruct.Pin = SC200R_PW_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SC200R_PW_EN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/*!
    \brief      get chip serial number
    \param[in]  none
    \param[out] none
    \retval     none
*/
void get_chip_serial_num(void)
{
    int_device_serial[0] = *(__IO uint32_t*)(0x1FFFF7E8);
    int_device_serial[1] = *(__IO uint32_t*)(0x1FFFF7EC);
    int_device_serial[2] = *(__IO uint32_t*)(0x1FFFF7F0);
}

/*!
    \brief      get chip sdram and flash info
    \param[in]  none
    \param[out] none
    \retval     none
*/
void get_chip_memery_info(void)
{
    int_device_memery_info = *(__IO uint32_t*)(0x1FFFF7E0);
}

#define BASE 65521
uint32_t adler32(uint8_t *buf, uint32_t len)
{

   uint32_t adler = 1;
   uint32_t s1    = (adler >> 0) & 0xFFFF;
   uint32_t s2    = (adler >> 16) & 0xFFFF;
   uint32_t i;

   for (i = 0; i < len; i++)
   {
      s1 = (s1 + buf[i]) % BASE;
      s2 = (s2 + s1) % BASE;
//      printf("s2:%#x, s1:%#x\n", s2, s1);
   }

   return (s2 << 16) + s1;
}

/* ----------------------xl355 function------------------------------------*/
int spi_write_byte(uint8_t addr, uint8_t data)
{
    uint8_t write_address = addr << 1;

    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi1, &write_address, 1, 0xFF) == HAL_OK)
    {
        HAL_SPI_Transmit(&hspi1, &data, 1, 0xFF);
    }

    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    return 0;
}

uint8_t spi_read_byte(uint8_t addr)             //read 1 Byte data
{
    uint8_t readbuff = 0xFF;
    uint8_t read_address = addr << 1;
    read_address |= 0x01;

    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi1, &read_address, 1, 0xFF) == HAL_OK)
    {
        HAL_SPI_Receive(&hspi1, &readbuff, 1, 0xFFF);
    }

    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);

    return readbuff;
}

int spi_read_multipe_bytes(uint8_t start_addr, uint8_t *rxdata, uint8_t len)
{
    uint8_t read_address = ((start_addr << 1) | 0x01);

    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi1, &read_address, 1, 0xFF) != HAL_OK)
    {
        memset(print_msg, 0x0, 256);
        msg_len = snprintf((char *)print_msg, 256, "\r\nadxl355 WRITE register %u failed\r\n", start_addr);
        HAL_UART_Transmit(&huart1, print_msg, msg_len, 1000);
        HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);
        return -1;
    }
    if (HAL_SPI_Receive(&hspi1, rxdata, len, 1000) != HAL_OK)
    {
        memset(print_msg, 0x0, 256);
        msg_len = snprintf((char *)print_msg, 256, "\r\nadxl355 read %u bytes from register %u failed\r\n", len, start_addr);
        HAL_UART_Transmit(&huart1, print_msg, msg_len, 1000);
        HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);
        return -1;
    }
    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);
    return 0;
}

int spi_write_multipe_bytes(uint8_t start_addr, uint8_t *txdata, uint8_t len)
{
    uint8_t write_address = (start_addr << 1);

    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi1, &write_address, 1, 0xFF) != HAL_OK)
    {
        memset(print_msg, 0x0, 256);
        msg_len = snprintf((char *)print_msg, 256, "\r\nadxl355 WRITE register %u failed\r\n", start_addr);
        HAL_UART_Transmit(&huart1, print_msg, msg_len, 1000);
        HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);
        return -1;
    }
    if (HAL_SPI_Transmit(&hspi1, txdata, len, 0xFFF) != HAL_OK)
    {
        memset(print_msg, 0x0, 256);
        msg_len = snprintf((char *)print_msg, 256, "\r\nadxl355 write %u bytes to register %u failed\r\n", len, start_addr);
        HAL_UART_Transmit(&huart1, print_msg, msg_len, 1000);
        HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);
        return -1;
    }
    HAL_GPIO_WritePin(XL355_SPI_CS_GPIO_Port, XL355_SPI_CS_Pin, GPIO_PIN_SET);
    return 0;
}

void adxl355_init(uint8_t range, uint8_t odr, uint8_t fifo_len, uint8_t sync_set)
{    
    if (spi_read_byte(XL355_PARTID) != 0xED)
    {
        PRINT_ERROR("adxl355 read register failed\r\n");
    }
    else
    {
        uint8_t DEVID_AD, DEVID_MST, REVID;
        DEVID_AD = spi_read_byte(XL355_DEVID_AD);
        DEVID_MST = spi_read_byte(XL355_DEVID_MST);
        REVID = spi_read_byte(XL355_REVID);
        PRINT_INFO("adxl355 ADIID:%u, MEMSID:%u, REVID:%u\r\n", DEVID_AD, DEVID_MST, REVID);

        spi_write_byte(XL355_RESET, 0x52);
        spi_write_byte(XL355_RANGE, range); // 0xC1
        spi_write_byte(XL355_FIFO_SAMPLES, fifo_len); //
        spi_write_byte(XL355_INT_MAP, 0x02); // FULL_EN1
        spi_write_byte(XL355_FILTER, odr); // 2000hz
        spi_write_byte(XL355_SYNC, sync_set);
    //    spi_write_byte(XL355_SELF_TEST, 0x03);

        uint8_t tmp;
        tmp = spi_read_byte(XL355_RANGE);
        if (tmp != range)
        {
            PRINT_WARN("xl355 range set fail(%x:%x)\r\n", range, tmp);
        }
        PRINT_WARN("xl355 range set (%x:%x)\r\n", range, tmp);
        tmp = spi_read_byte(XL355_SYNC);
        if (tmp != sync_set)
        {
            PRINT_WARN("xl355 sync set fail(%x:%x)\r\n", sync_set, tmp);
        }
        PRINT_WARN("xl355 sync set (%x:%x)\r\n", sync_set, tmp);
        tmp = spi_read_byte(XL355_FILTER);
        if (tmp != odr)
        {
            PRINT_WARN("xl355 odr set fail(%x:%x)\r\n", odr, tmp);
        }
        PRINT_WARN("xl355 odr set (%x:%x)\r\n", odr, tmp);
        tmp = spi_read_byte(XL355_FIFO_SAMPLES);
        if (tmp != fifo_len)
        {
            PRINT_WARN("xl355 fifo samples set fail(%x:%x)\r\n", fifo_len, tmp);
        }
        PRINT_WARN("xl355 fifo samples set (%x:%x)\r\n", fifo_len, tmp);
        tmp = spi_read_byte(XL355_INT_MAP);
        if (tmp != 0x02)
        {
            PRINT_WARN("xl355 int_map set fail(%x:%x)\r\n", 0x02, tmp);
        }
        PRINT_WARN("xl355 int_map set (%x:%x)\r\n", 0x02, tmp);
    }
}

int adxl355_start_work(void)
{
    uint8_t pwrctl = spi_read_byte(XL355_POWER_CTL);
    pwrctl &= 0xFE;

    spi_write_byte(XL355_POWER_CTL, pwrctl);
    pwrctl = spi_read_byte(XL355_POWER_CTL);
    if ((pwrctl & 0x1) != 0)
    {
        PRINT_ERROR("xl355 start fail(%x)\r\n", pwrctl);
        return -1;
    }
    return 0;
}

float adxl355_conversion_acc_data(uint8_t *data)
{
  	uint32_t acc_raw;      // register data
    int32_t acc_actual;    // 
    float acc_float;       // 

    acc_raw = (((data[0] << 16) | (data[1] << 8) | data[2]) >> 4);
          
    if(acc_raw >= 0x80000)
    {
        acc_actual = -((~acc_raw & 0x3FFFF) + 1);
    }
    else
    {
        acc_actual = acc_raw;
    }
    
    acc_float = acc_actual * 0.0039; // 2g: scale factor 3.9 ug/LSB
    // acc_float = acc_actual * 0.0039; // 4g: scale factor 7.8 ug/LSB
    // acc_float = acc_actual * 0.0039; // 8g: scale factor 15.6 ug/LSB
    
    return acc_float;
}

float adxl355_conversion_temperature(uint8_t *data)
{
    uint16_t reg;
    float temp;

    reg = (((data[0] & 0x0F) << 8) | data[1]);
    temp = 25 - ((reg - 1852) / (9.05));
 
    return temp;
}

/* -------------gpio interrupt handle -------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    
    if(GPIO_Pin == XL355_INT1_Pin)
    {
        xl355_fifo_full_flag++;
        sys_run.time_5ms_count++;
    }
		else if (GPIO_Pin == GPS_1PPS_Pin)
    {
        sys_run.pps_count++;
        sys_run.pps_flag = true;
    }
}

/* -------------ublox function ----------------*/

/**
 * @brief generate ubx protocol msg, which could send to ubx then.
 * @param msg_class for ubx protocol
 * @param msg_id    for ubx protocol
 * @param data      the content of msg
 * @param data_len
 * @param buf       the buffer to store the generated msg
 * @param buf_len
 *
 * @return actual length of the msg
 */
int ubx_msg_generate(uint8_t msg_class, uint8_t msg_id, const unsigned char *data, uint16_t data_len, uint8_t *buf, uint16_t buf_len)
{
    uint8_t CK_A = 0, CK_B = 0;
    uint16_t count = 0, i = 0;

    if (buf == NULL || buf_len < 6)
        return -1;

    buf[0] = UBX_SYNC_CHAR1;
    buf[1] = UBX_SYNC_CHAR2;
    buf[2] = msg_class;
    buf[3] = msg_id;
    buf[4] = data_len & 0xFF;
    buf[5] = (data_len >> 8) & 0xFF;

    count += 6;

    if (data != NULL && data_len != 0)
    {
        memcpy(&buf[count], data, data_len);
        count += data_len;
    }

    /* calculate CRC */
    for (i = 2; i < count; i++)
    {
        CK_A += buf[i];
        CK_B += CK_A;
    }

    buf[count]   = CK_A;
    buf[count+1] = CK_B;
    count += 2;

    return count;
}

int ubx_reset(void)
{
    // reset ubox
    HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);

    return 0;
}
int ubx_init(void)
{
    uint8_t buf[64] = {0};
    int msg_len = 0;
    int ret;

    msg_len = ubx_msg_generate(UBX_CFG_CLASS_ID, UBX_CFG_TP5_MSG_ID, UBX_MSG_DATA_SET_TIMEPULSE_0, sizeof(UBX_MSG_DATA_SET_TIMEPULSE_0), buf, sizeof(buf));
    if (msg_len > 6)
    {
        ret = HAL_UART_Transmit(&huart2, buf, msg_len, 1000);
        if (ret == HAL_OK)
        {
            PRINT_DEBUG("send cfg0 message to ubx successfully\r\n");
        }
        else
        {
            PRINT_WARN("send cfg0 message to ubx fail:%d\r\n", ret);
            return -1;
        }
    }
    else
    {
        PRINT_WARN("generate ubx cfg0 message fail, returrn len:%d\r\n", msg_len);
        return -1;
    }
    
    msg_len = ubx_msg_generate(UBX_CFG_CLASS_ID, UBX_CFG_TP5_MSG_ID, UBX_MSG_DATA_SET_TIMEPULSE_1, sizeof(UBX_MSG_DATA_SET_TIMEPULSE_1), buf, sizeof(buf));
    if (msg_len > 6)
    {
        ret = HAL_UART_Transmit(&huart2, buf, msg_len, 1000);
        if (ret == HAL_OK)
        {
            PRINT_DEBUG("send cfg1 message to ubx successfully\r\n");
        }
        else
        {
            PRINT_WARN("send cfg1 message to ubx fail:%d\r\n", ret);
            return -1;
        }
    }
    else
    {
        PRINT_WARN("generate ubx cfg1 message fail, returrn len:%d\r\n", msg_len);
        return -1;
    }
    return 0;
}

void ubx_uart_recv_data_callback(UART_HandleTypeDef *huart)
{
    uint32_t temp;

    if((__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        HAL_UART_DMAStop(huart);
        temp = __HAL_DMA_GET_COUNTER(huart->hdmarx);
        ubx_rx_buffer.buffer_len =  UART_RX_MAX_LEN - 1 - temp;
        if (ubx_rx_buffer.buffer_len > 0)
        {
            if (ubx_rx_buffer.buffer[0] == '$')
            {
                // nmea data
                sys_run.nmea_flag = true;
                memset(ubx_buff, 0x0, ubx_rx_buffer.buffer_len);
                memcpy(ubx_buff, ubx_rx_buffer.buffer, ubx_rx_buffer.buffer_len);
            }
            // else
            // {
            //     // ubx data
                
            // }
        }
        memset(&ubx_rx_buffer, 0x0, sizeof(uart_rx_t));
        HAL_UART_Receive_DMA(huart, ubx_rx_buffer.buffer, UART_RX_MAX_LEN - 1);
    }
}

/*int xl355_resync(void)
{
  sys_run.xl355_sync_flag = false;
  sys_run.start_sync_flag = SYNC_STEP0;
  return 0;
}*/

/* ----------------parse nmea ----------------------*/
static int string_to_double(char *str, double *param)
{
    if (str == NULL)
    {
        return -1;
    }
    char *endptr = NULL;
    *param = strtod(str, &endptr);
    return 0;
}
static int string_to_float(char *str, float *param)
{
    if (str == NULL)
    {
        return -1;
    }
    char *endptr = NULL;
    *param = strtof(str, &endptr);
    return 0;
}
static int string_to_uint8(char *str, uint8_t *param)
{
    if (str == NULL)
    {
        return -1;
    }

    while (str != NULL && *str == '0')
    {
        str++;
    }
    char *endptr = NULL;
    *param = (uint8_t)strtoul(str, &endptr, 0);
    return 0;
}

void utc_to_timezone_time(gps_time_t *utc_time, int8_t timezone)
{
    int year, month, day, hour;
    int lastday_of_thismonth = 0;        //last day of this month
    int lastday_of_lastmonth = 0;        //last day of last month

    year  = utc_time->utc_YY; //utc time
    month = utc_time->utc_MM;
    day   = utc_time->utc_DD;
    hour  = utc_time->utc_hh + timezone;

    if (month == 1 || month == 3 || month == 5 || month == 7 || \
        month == 8 || month == 10 || month == 12)
    {
        lastday_of_thismonth = 31;

        if (month == 3)
        {
            if ((year % 400 == 0) || ( year % 4 == 0 && year % 100 != 0)) //if this is lunar year
            {
                lastday_of_lastmonth = 29;
            }
            else
            {
                lastday_of_lastmonth = 28;
            }
        }

        if (month == 8)
        {
            lastday_of_lastmonth = 31;
        }
    }
    else if (month == 4 || month == 6 || month == 9 || month == 11)
    {
        lastday_of_thismonth = 30;
        lastday_of_lastmonth = 31;
    }
    else
    {
        lastday_of_lastmonth = 31;
        if ((year % 400 == 0) || ( year % 4 == 0 && year % 100 != 0))
        {
            lastday_of_thismonth = 29;
        }
        else
        {
            lastday_of_thismonth = 28;
        }
    }

    if (hour >= 24)
    {
        // if >24, day+1
        hour -= 24;
        day += 1;


        if (day > lastday_of_thismonth)
        {
            // next month,  day-lastday of this month
            day -= lastday_of_thismonth;
            month += 1;
            if(month > 12)
            {
                // next year , month-12
                month -= 12;
                year += 1;
            }
        }
    }

    if(hour < 0)
    {
        // if <0, day-1
        hour += 24;
        day -= 1;
        if(day < 1)
        {
            // month-1, day=last day of last month
            day = lastday_of_lastmonth;
            month -= 1;
            if (month < 1)
            {
                // last year , month=12
                month = 12;
                year -= 1;
            }
        }
    }

    utc_time->utc_YY = year;
    utc_time->utc_MM = month;
    utc_time->utc_DD = day;
    utc_time->utc_hh = hour;

    return ;
}

static int parse_gga(uint8_t *data)
{
    // $GPGGA,104227.00,3032.5399904,N,10404.3717714,E,7,21,1.1,546.0337,M,-42.034,M,,*7F

    char *msg_ptr = NULL;
    char *save_ptr = NULL;

    // PRINT_DEBUG("%s", data);
    msg_ptr = strtok_r((char *)data, ",", &save_ptr);   // msg_ptr-> $GPGGA
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 104227.00
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 3032.5399904
    string_to_double(msg_ptr, &(gs_gnss_info.pos.lat));
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> N
    // string_to_uint8(msg_ptr, &(gs_gnss_info.pos.lat_dir));
    gs_gnss_info.pos.lat_dir = *msg_ptr;
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 10404.3717714
    string_to_double(msg_ptr, &(gs_gnss_info.pos.lon));
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> E
    // string_to_uint8(msg_ptr, &(gs_gnss_info.pos.lon_dir));
    gs_gnss_info.pos.lon_dir = *msg_ptr;
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 7
    string_to_uint8(msg_ptr, &(gs_gnss_info.pos.gps_qual));
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 21
    string_to_uint8(msg_ptr, &(gs_gnss_info.pos.compute_sats));
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 1.1
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 546.0337
    string_to_float(msg_ptr, &(gs_gnss_info.pos.alt));
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> M
    // string_to_uint8(msg_ptr, &(gs_gnss_info.pos.a_units));
    gs_gnss_info.pos.a_units = *msg_ptr;
    return 0;
}

static int parse_gsv(uint8_t *data)
{
    /*
    $GPGSV,2,1,08,23,64,098,,10,68,349,45,24,30,044,,18,26,181,*70
    $GPGSV,2,2,08,32,46,298,45,12,23,101,29,25,24,136,,31,15,215,*7C
    $BDGSV,3,1,11,146,70,329,40,156,69,341,41,179,64,006,43,149,63,285,39*6C
    $BDGSV,3,2,11,182,64,022,42,154,53,334,47,145,31,245,35,183,16,241,39*60
    $BDGSV,3,3,11,173,30,311,48,168,12,2947,161,30,092,33,,,,*69
    $GLGSV,1,1,04,42,46,031,43,58,25,294,46,43,25,316,36,51,15,031,45*65
    $GAGSV,1,1,04,89,59,355,40,81,16,315,43,82,43,261,36,74,24,286,41*69
     */
    int flag = 0;
    uint8_t *tmp = NULL;
    char *msg_ptr = NULL;
    char *save_ptr = NULL;

    msg_ptr = strtok_r((char *)data, ",", &save_ptr);   // msg_ptr-> $GPGSV
    if (strcmp(msg_ptr, "GPGSV") == 0)
    {
        flag = 1;
    }
    else if (strcmp(msg_ptr, "BDGSV") == 0)
    {
        flag = 2;
    }
    else if (strcmp(msg_ptr, "GLGSV") == 0)
    {
        flag = 3;
    }
    else if (strcmp(msg_ptr, "GAGSV") == 0)
    {
        flag = 4;
    }
    else
    {
        PRINT_WARN("unkown gsv %s\n", msg_ptr);
        return -1;
    }

    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 2
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 1
    if (*msg_ptr == '1')
    {
        msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 08
        switch (flag)
        {
        case 1:
            tmp = &(gs_gnss_info.pos.gp_obs_sats);
            break;
        case 2:
            tmp = &(gs_gnss_info.pos.bd_obs_sats);
            break;
        case 3:
            tmp = &(gs_gnss_info.pos.gl_obs_sats);
            break;
        case 4:
            tmp = &(gs_gnss_info.pos.ga_obs_sats);
            break;
        default:
            break;
        }
        string_to_uint8(msg_ptr, tmp);
    }
    return 0;
}
/*
static int parse_rmc(uint8_t *data)
{
    // $GPRMC,104307.00,A,3032.5399904,N,10404.3717714,E,000.083,308.4,230821,0.0,W,A*20
    char *msg_ptr = NULL;
    char *save_ptr = NULL;

    msg_ptr = strtok_r((char *)data, ",", &save_ptr);   // msg_ptr-> $GPRMC
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 104307.00
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> A
    // string_to_uint8(msg_ptr, &(gs_gnss_info.pos.pos_status));
    gs_gnss_info.pos.pos_status = *msg_ptr;
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 3032.5399904
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> N
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 10404.3717714
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> E
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 000.083
    string_to_float(msg_ptr, &(gs_gnss_info.pos.speed));
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 308.4
    string_to_float(msg_ptr, &(gs_gnss_info.pos.track_true));
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 230821
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 0
    string_to_float(msg_ptr, &(gs_gnss_info.pos.mag_var));
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> W
    // string_to_uint8(msg_ptr, &(gs_gnss_info.pos.var_dir));
    gs_gnss_info.pos.var_dir = *msg_ptr;
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> A
    // string_to_uint8(msg_ptr, &(gs_gnss_info.pos.mode_ind));
    gs_gnss_info.pos.mode_ind = *msg_ptr;
    return 0;
}*/

static int parse_zda(uint8_t *data)
{
    if (gs_gnss_info.pos.pos_status == 'V')
    {
        return -1;
    }

    RTC_DateTypeDef rtc_date;
    RTC_TimeTypeDef rtc_time;

    // $GPZDA,104317.00,23,08,2021,,*6E
    char *msg_ptr = NULL;
    char *save_ptr = NULL;

    msg_ptr = strtok_r((char *)data, ",", &save_ptr);   // msg_ptr-> $GPZDA
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 104317.00
    gs_gnss_info.utc_time.utc_hh = (msg_ptr[0] - '0') * 10 + msg_ptr[1] - '0';
    gs_gnss_info.utc_time.utc_mm = (msg_ptr[2] - '0') * 10 + msg_ptr[3] - '0';
    gs_gnss_info.utc_time.utc_ss = (msg_ptr[4] - '0') * 10 + msg_ptr[5] - '0';

    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 23
    string_to_uint8(msg_ptr, &(gs_gnss_info.utc_time.utc_DD));
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 08
    string_to_uint8(msg_ptr, &(gs_gnss_info.utc_time.utc_MM));
    msg_ptr = strtok_r(NULL, ",", &save_ptr);   // msg_ptr-> 2021
    msg_ptr += 2;
    string_to_uint8(msg_ptr, &(gs_gnss_info.utc_time.utc_YY));

    // set rtc date and time
    utc_to_timezone_time(&(gs_gnss_info.utc_time), TIME_ZONE);
    rtc_date.Year    = gs_gnss_info.utc_time.utc_YY;
    rtc_date.Month   = gs_gnss_info.utc_time.utc_MM;
    rtc_date.Date    = gs_gnss_info.utc_time.utc_DD;
    rtc_time.Hours   = gs_gnss_info.utc_time.utc_hh;
    rtc_time.Minutes = gs_gnss_info.utc_time.utc_mm;
    rtc_time.Seconds = gs_gnss_info.utc_time.utc_ss;

    HAL_RTC_SetTime_and_Date(&rtc_date, &rtc_time);
    return 0;
}

static nmea_msg_type_t get_nmea_msg_type(char *start_position)
{
    char header[7];
    int header_len = 0;
    nmea_msg_type_t msg_type = NMEA_UNKNOWN;
    char *ptmp = NULL;

    if ((ptmp = strchr(start_position, ',')) != NULL)
    {
        memset(header, 0x0, 7);
        header_len = (int)(ptmp - start_position);
        if (header_len >= 7)
        {
            PRINT_ERROR("nmea header %s len(%d) not right\n", start_position, header_len);
            return -1;
        }
        memcpy(header, start_position, header_len);

        // GNSS_DEBUG("nmea msg header is %s\n", header);
        if ((strcmp(header, "GPGGA") == 0) || (strcmp(header, "GNGGA") == 0))
        {
            msg_type = NMEA_GGA;
        }
        else if ((strcmp(header, "GPGSV") == 0) || (strcmp(header, "BDGSV") == 0) || \
                    (strcmp(header, "GLGSV") == 0) || (strcmp(header, "GAGSV") == 0) || (strcmp(header, "GNGSV") == 0))
        {
            msg_type = NMEA_GSV;
        }
        else if ((strcmp(header, "GPRMC") == 0) || (strcmp(header, "GNRMC") == 0))
        {
            msg_type = NMEA_RMC;
        }
        else if ((strcmp(header, "GPZDA") == 0) || (strcmp(header, "GNZDA") == 0))
        {
            msg_type = NMEA_ZDA;
        }
        else
        {
            // PRINT_WARN("unknown nmea header %s\n", header);
            msg_type = NMEA_UNKNOWN;
        }
    }
    return msg_type;
}
static int parse_one_nmea_message(char *msg)
{
    nmea_msg_type_t msg_type;
//     char *p_start = NULL;
    char *msg_tmp = NULL;
    int ret = -1;

    msg_tmp = msg;

    // if ((p_start = strchr(msg_tmp, '$')) != NULL)
    {
        msg_type = get_nmea_msg_type(msg_tmp);

        switch (msg_type)
        {
        case NMEA_GGA:
            ret = parse_gga((uint8_t *)msg_tmp);
            break;
        case NMEA_GSV:
            ret = parse_gsv((uint8_t *)msg_tmp);
            break;
        case NMEA_RMC:
            // ret = parse_rmc((uint8_t *)msg_tmp);
            ret = 0;
            break;
        case NMEA_ZDA:
            ret = parse_zda((uint8_t *)msg_tmp);
            break;
        default:
            ret = -1;
            break;
        }
    }
    return ret;
}

int parse_nmea_message(uint8_t *nmea_msg)
{
    if (nmea_msg == NULL)
    {
        PRINT_ERROR("strdup fail\n");
        return -1;
    }

    char *msg_ptr = NULL;
    char *save_ptr = NULL;

    msg_ptr = strtok_r((char *)nmea_msg, "$", &save_ptr);
    while(msg_ptr)
    {
        // PRINT_DEBUG("message = %s\n", msg_ptr);
        parse_one_nmea_message(msg_ptr);
        msg_ptr = strtok_r(NULL, "$", &save_ptr);
    }

    return 0;
}

/* ------------get unix timestamp ---------------- */
uint32_t get_unix_timestamp(void)
{
    struct tm tmp;
    uint32_t timestamp;
    RTC_DateTypeDef rtc_date;
    RTC_TimeTypeDef rtc_time;

    HAL_RTC_GetTime_and_Date(&rtc_date, &rtc_time);
    
    tmp.tm_year = rtc_date.Year + 100;
    tmp.tm_mon  = rtc_date.Month - 1;
    tmp.tm_mday = rtc_date.Date;
    tmp.tm_hour = rtc_time.Hours;
    tmp.tm_min  = rtc_time.Minutes;
    tmp.tm_sec  = rtc_time.Seconds;

    timestamp   = mktime(&tmp);

    return timestamp;
}

/* ------------package acceleration data ---------------- */
int package_xl355_raw_data(uint8_t *in_buf, uint8_t *out_buff, int data_len)
{
  	int pack_count = 0;
    int pack_content_len = data_len + 9;
    uint32_t adler32_check_sum = 0;
    uint32_t microseconds = sys_run.unix_timestamp_micro_second;


    out_buff[0] = 0xEF;
    out_buff[1] = 0xEF;
    out_buff[2] = ((pack_content_len & 0xFF000000) >> 24);
    out_buff[3] = ((pack_content_len & 0x00FF0000) >> 16);
    out_buff[4] = ((pack_content_len & 0x0000FF00) >> 8);
    out_buff[5] = ((pack_content_len & 0x000000FF) >> 0);

    out_buff[6] = ((sys_run.xl355_sync_flag == true) ? 0x01 : 0x00);

    out_buff[7] = ((sys_run.unix_timestamp_second & 0xFF000000) >> 24);
    out_buff[8] = ((sys_run.unix_timestamp_second & 0x00FF0000) >> 16);
    out_buff[9] = ((sys_run.unix_timestamp_second & 0x0000FF00) >> 8);
    out_buff[10] =((sys_run.unix_timestamp_second & 0x000000FF) >> 0);

    out_buff[11] =((microseconds & 0xFF000000) >> 24);
    out_buff[12] =((microseconds & 0x00FF0000) >> 16);
    out_buff[13] =((microseconds & 0x0000FF00) >> 8);
    out_buff[14] =((microseconds & 0x000000FF) >> 0);

	  pack_count += 15;

		memcpy(&out_buff[pack_count], in_buf, data_len);
		pack_count += data_len;
				
		adler32_check_sum = adler32(out_buff, pack_count);
    out_buff[pack_count+0] = ((adler32_check_sum & 0xFF000000) >> 24);
    out_buff[pack_count+1] = ((adler32_check_sum & 0x00FF0000) >> 16);
    out_buff[pack_count+2] = ((adler32_check_sum & 0x0000FF00) >> 8);
    out_buff[pack_count+3] = ((adler32_check_sum & 0x000000FF) >> 0);
		pack_count += 4;

    return pack_count;
}
/* ------------my log function---------------- */
#define MAX_ONE_LOG_SIZE   (250)									// length of one log message
#define MAX_LOG_BUFF_SIZE  (MAX_ONE_LOG_SIZE * 30) // total buffer length of log

char log_msg_buf[MAX_LOG_BUFF_SIZE];
char log_msg_send[MAX_LOG_BUFF_SIZE];
uint16_t log_msg_len = 0;  //needn`t mutex

char *my_basename(char *s)
{
    char *p_ret;
    p_ret = strrchr(s, '/');
    if (p_ret)
    {
        return (p_ret + 1);
    }
    return s;
}

int log_message(log_level_e level, const char *level_str, const char *fmt, ...)
{
    char msg_buf[MAX_ONE_LOG_SIZE] = {0};
    int len = 0;
    
    va_list ap;
    RTC_DateTypeDef rtc_date;
    RTC_TimeTypeDef rtc_time;

    if (level < LOG_LEVEL_DEBUG)
    {
        return 0;
    }

    if (log_msg_len+100 >= MAX_LOG_BUFF_SIZE)
    {
      return 1;
    }
    HAL_RTC_GetTime_and_Date(&rtc_date, &rtc_time);

    len = snprintf(msg_buf, MAX_ONE_LOG_SIZE, "%4u/%02u/%02u %02u:%02u:%02u.%03u[%5.5s] ",
                                    (2000 + rtc_date.Year), rtc_date.Month, rtc_date.Date, \
		rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds, ((sys_run.time_5ms_count%200) * 5), level_str); // todo : how to get ms
    va_start(ap, fmt);
    len += vsnprintf(&msg_buf[len], (MAX_ONE_LOG_SIZE - len), fmt, ap);
    va_end(ap);

    if (len > 0)
    {
      if ((log_msg_len + len) < MAX_LOG_BUFF_SIZE)
      {
        memcpy(&log_msg_buf[log_msg_len], msg_buf, len);
        log_msg_len += len;
        return 0;
      }
      else
      {
        log_msg_buf[log_msg_len] = 0;
        log_msg_len = MAX_LOG_BUFF_SIZE;
      }
    }

    return -1;
}

int log_binary(log_level_e level, const char *fmt, ...)
{
    va_list ap;
    char msg_buf[MAX_ONE_LOG_SIZE] = {0};
    int len = 0;

    if (level < LOG_LEVEL_DEBUG) 
    {
        return 0;
    }

    if (log_msg_len+100 >= MAX_LOG_BUFF_SIZE)
    {
      return 1;
    }

    va_start(ap, fmt);
    len = vsnprintf((char *)msg_buf, MAX_ONE_LOG_SIZE, fmt, ap);
    va_end(ap);

    if (len > 0)
    {
      if ((log_msg_len + len) < MAX_LOG_BUFF_SIZE)
      {
        memcpy(&log_msg_buf[log_msg_len], msg_buf, len);
        log_msg_len += len;
        return 0;
      }
      else
      {
        log_msg_buf[log_msg_len] = 0;
        log_msg_len = MAX_LOG_BUFF_SIZE;
      }
    }

    return -1;
}


int log_print(void)
{
    if (log_msg_len > 0) 
    {
        memcpy(log_msg_send, log_msg_buf, MAX_LOG_BUFF_SIZE);
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)log_msg_send, strlen(log_msg_send));
        memset(log_msg_buf, 0x0, MAX_LOG_BUFF_SIZE);
        log_msg_len = 0;
        return 0;
    }
    return -1;
}

/*-----------------rtc function ------------------------*/
void HAL_RTC_GetTime_and_Date(RTC_DateTypeDef *s_date, RTC_TimeTypeDef *s_time)
{
  if (s_time != NULL)
    HAL_RTC_GetTime(&hrtc, s_time, RTC_FORMAT_BIN);

  if (s_date != NULL)
    HAL_RTC_GetDate(&hrtc, s_date, RTC_FORMAT_BIN);
}

void HAL_RTC_SetTime_and_Date(RTC_DateTypeDef *s_data, RTC_TimeTypeDef *s_time)
{
  if (s_time != NULL)
    HAL_RTC_SetTime(&hrtc, s_time, RTC_FORMAT_BIN);
  if (s_data != NULL)
    HAL_RTC_SetDate(&hrtc, s_data, RTC_FORMAT_BIN);
}

/*-----------------led control function ------------------------*/
int led_ctl(led_type_t name, led_ctl_t on_off)
{
  GPIO_TypeDef *GPIOx;
  uint16_t pin;

  if (name == SYNC_LED)
  {
    GPIOx = LED_SYNC_GPIO_Port;
    pin = LED_SYNC_Pin;
  }
  else if (name == WORK_LED)
  {
    GPIOx = LED_RUN_GPIO_Port;
    pin = LED_RUN_Pin;
  }
  else
  {
    return -1;
  }

  if (on_off == LED_ON)
  {
    HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_RESET);
  }
  else if (on_off == LED_OFF)
  {
    HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_SET);
  }
  else if (on_off == LED_TOGGLE)
  {
    HAL_GPIO_TogglePin(GPIOx, pin);
  }
  else
  {
    return -1;
  }

  return 0;
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
