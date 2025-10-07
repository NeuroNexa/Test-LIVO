
# Mid-360 + Seeker Omni Depth + Jetson Orin NX + **STM32F103C8T6（Blue Pill）**
**硬件时间硬同步（双路独立 PPS + 串口时标 + `$GPRMC`）完整实操手册（含完整 STM32 源码）**  
> 适配你的板卡与需求：Seeker 走 GH1.25（GND | RX | PPS | VCC），Mid-360 走功能线（Pin8=PPS，Pin10=GPS 串口输入）；STM32 用 **TIM3 的 CH1/CH2** 同步输出两路独立 PPS，**USART1→Seeker** 发送**时标帧**，**USART2→Mid-360** 发送 **`$GPRMC`**。

---

## 目录
- [Mid-360 + Seeker Omni Depth + Jetson Orin NX + **STM32F103C8T6（Blue Pill）**](#mid-360--seeker-omni-depth--jetson-orin-nx--stm32f103c8t6blue-pill)
  - [目录](#目录)
  - [确定协议与格式（来自你提供的源码/资料）](#确定协议与格式来自你提供的源码资料)
    - [1）Seeker 相机串口**时标帧**（来自你给的 `pps` 目录用户态代码 `pps_s_main.c`）](#1seeker-相机串口时标帧来自你给的-pps-目录用户态代码-pps_s_mainc)
    - [2）Mid-360 的 `$GPRMC`（参考你提供的 avia+mvs+stm32 示例代码）](#2mid-360-的-gprmc参考你提供的-aviamvsstm32-示例代码)
  - [硬件连接（不并联：双路 PPS 独立引脚）](#硬件连接不并联双路-pps-独立引脚)
    - [你的供电约定](#你的供电约定)
    - [STM32F103C8T6（Blue Pill）→ 外设连线](#stm32f103c8t6blue-pill-外设连线)
  - [STM32CubeIDE 工程创建与外设配置](#stm32cubeide-工程创建与外设配置)
  - [1. 新建工程（选择芯片、生成骨架）](#1-新建工程选择芯片生成骨架)
  - [2. System Core 设置（RCC、调试口）](#2-system-core-设置rcc调试口)
  - [3. Clock Configuration（72 MHz 系统时钟）](#3-clock-configuration72mhz-系统时钟)
  - [4. 配置 TIM3：两路 PWM 同步输出（1 Hz，20 ms 高电平）](#4-配置-tim3两路-pwm-同步输出1hz20ms-高电平)
  - [5. 配置串口：USART1（给 Seeker）与 USART2（给 Mid‑360）](#5-配置串口usart1给-seeker与-usart2给-mid360)
    - [5.1 USART1 → Seeker（发送时标帧；并接收 SETTIME 命令）](#51-usart1--seeker发送时标帧并接收-settime-命令)
    - [5.2 USART2 → Mid‑360（发送 `$GPRMC`）](#52-usart2--mid360发送-gprmc)
  - [6. 生成代码](#6-生成代码)
  - [7. 构建与烧录](#7-构建与烧录)
  - [完整 STM32 源码（可直接粘贴编译）](#完整-stm32-源码可直接粘贴编译)
  - [上电烧录与自检](#上电烧录与自检)
  - [Jetson 侧运行与 SLAM](#jetson-侧运行与-slam)
  - [联调与常见问题](#联调与常见问题)
    - [结语](#结语)

---

## 确定协议与格式（来自你提供的源码/资料）

### 1）Seeker 相机串口**时标帧**（来自你给的 `pps` 目录用户态代码 `pps_s_main.c`）
你上传的代码定义了：
```c
struct pps_packet {
    uint32_t header;
    uint64_t sec;
    uint64_t nsec;
    uint32_t footer;
} __attribute__ ((packed));

// 构造：
packet.header = 0XAA;
packet.sec    = ts->tv_sec;
packet.nsec   = ts->tv_nsec;
packet.footer = crc32(&packet->sec, sizeof(packet->sec) + sizeof(packet->nsec));
```
**要点**：
- `header` 是 **4 字节**的 `0x000000AA`（小端发送时序列化为 `AA 00 00 00`）。  
- CRC32 只覆盖 `sec` 与 `nsec` 两段（共 16 字节），**不含 header**。  
- 全部字段按 **小端**发送（与 STM32 小端一致）。

> 这就是我们 STM32 要发送给 **Seeker.RX** 的格式；下面的代码完全按此实现，避免与你仓库解析逻辑不一致。

### 2）Mid-360 的 `$GPRMC`（参考你提供的 avia+mvs+stm32 示例代码）
你给的 TIM3 中断里用 `sprintf` 构造 `$GPRMC,hhmmss.00,A,……,DDMMYY,…,*CS\r\n`，并通过函数 `checkNum()` 计算 NMEA 校验和（`*` 之前异或）。  
本手册的 STM32 代码：
- 也在**每个新秒**生成 `$GPRMC`；
- 采用**与你示例一致**的 NMEA 校验算法；
- 经纬度/航速/航向等字段**允许为 0**（仅用于授时，Mid-360 不强制要求真实定位数据）。

---

## 硬件连接（不并联：双路 PPS 独立引脚）

### 你的供电约定
- **STM32**：板载 **Micro-USB** 供电；  
- **Seeker**：通过 **USB 连接 Jetson** 供电与数据；  
- **Mid-360**：使用原厂电源线独立供电；网口连 Jetson。

### STM32F103C8T6（Blue Pill）→ 外设连线
**STM32 引脚分配**
- **PA6 (TIM3_CH1)** → Seeker.PPS（1 Hz，推荐高电平 20 ms）  
- **PA7 (TIM3_CH2)** → Mid-360.PPS Pin8（与 PA6 严格同步）  
- **PA9 (USART1_TX)** → Seeker.RX（115200/8N1，发送时标帧）  
- **PA2 (USART2_TX)** → Mid-360 Pin10（GPS 串口输入，9600/8N1，发送 `$GPRMC`）  
- **GND 共地**：STM32 ↔ Seeker ↔ Mid-360

**Seeker GH1.25（从左到右：GND | RX | PPS | VCC）**
- `STM32 GND` → **GND**  
- `PA9 (USART1_TX)` → **RX**  
- `PA6 (TIM3_CH1)` → **PPS**  
- **VCC 不接**（已从 USB 取电）

**Mid-360 功能线**
- `STM32 GND` → **GND**（功能线 Pin2/3 任一地脚）  
- `PA7 (TIM3_CH2)` → **Pin 8（PPS，LVTTL_IN 3.3V）**  
- `PA2 (USART2_TX)` → **Pin 10（GPS 串口输入，9600/8N1）**

**建议**
- 两路 PPS 各串 **47–100 Ω 小电阻** 抑制反射；  
- 所有 IO 均为 **3.3 V** 电平；**不要**给 STM32 IO 口输入 5 V。

---

## STM32CubeIDE 工程创建与外设配置

## 1. 新建工程（选择芯片、生成骨架）
1. 打开 **STM32CubeIDE** → `File` → `New` → **STM32 Project**。  
2. 选择 **MCU/MPU Selector**，在 **Part Number** 输入：`STM32F103C8T6`（或 `STM32F103C8Tx`）→ `Next`。  
3. 工程名随意（例：`pps_sync_bluepill`）→ `Finish`。  
4. 弹窗询问是否生成初始化代码 → 选 **Yes**。

---

## 2. System Core 设置（RCC、调试口）
1. 左侧 `Pinout & Configuration` 栏，展开 **System Core**：
   - **RCC**：把 **HSE** 设为 **Crystal/Ceramic Resonator**（Blue Pill 板载 8 MHz 晶振）。  
   - **SYS**：`Debug` 选择 **Serial Wire**（启用 SWD，自动关闭 JTAG，避免占用 PA13/PA14）。

> 这样做的目的是：后面我们用 PLL 把系统时钟升到 **72 MHz**；SWD 便于烧录与调试。

---

## 3. Clock Configuration（72 MHz 系统时钟）
1. 切换到上方的 **Clock Configuration** 选项卡。  
2. 按下列目标设置（常见 Blue Pill 时钟树）：
   - **HSE = 8 MHz**
   - **PLL Source = HSE**, **PLL Multiplier = x9** → **SYSCLK = 72 MHz**
   - **AHB (HCLK) = 72 MHz**
   - **APB1 (PCLK1) = 36 MHz**（Cube 会自动设成 /2）  
   - **APB2 (PCLK2) = 72 MHz**
3. **关键提示**：虽然 **PCLK1=36 MHz**，**TIM3 属于 APB1**，当 APB1 分频不为 1 时，**定时器时钟会 ×2**，因此 **TIM3CLK=72 MHz**。这正好和我们在代码里按 72 MHz 计算的预分频一致。

---

## 4. 配置 TIM3：两路 PWM 同步输出（1 Hz，20 ms 高电平）
1. 回到 **Pinout & Configuration**。  
2. 展开 **Timers → TIM3**：
   - `Channel 1` 选择 **PWM Generation CH1**（Cube 自动把 **PA6** 设为 `TIM3_CH1`）。  
   - `Channel 2` 选择 **PWM Generation CH2**（Cube 自动把 **PA7** 设为 `TIM3_CH2`）。
3. 在右侧 **Parameter Settings**，填入：
   - **Prescaler = 7199**（72 MHz ÷ (7199+1) = **10 kHz**）  
   - **Counter Mode = Up**  
   - **Period = 9999**（10 kHz ÷ (9999+1) = **1 Hz**）  
   - **Pulse CH1 = 200**（**20 ms** 高电平）  
   - **Pulse CH2 = 200**（与 CH1 一样，保证两路一致）  
   - **Internal Clock Division (CKD) = No Division**，**Auto‑Reload Preload = Disable**
4. 在 **NVIC Settings** 勾选 **TIM3 global interrupt**（用于 `HAL_TIM_PeriodElapsedCallback()` 每秒置位 `pps_tick_flag`）。

> 以上设置会让 **PA6、PA7** 由同一个定时器驱动，周期与相位严格一致，满足“PPS 双路独立、严格同步”。

---

## 5. 配置串口：USART1（给 Seeker）与 USART2（给 Mid‑360）
### 5.1 USART1 → Seeker（发送时标帧；并接收 SETTIME 命令）
1. 打开 **Connectivity → USART1**，勾选 **Asynchronous**。  
2. 右侧参数：
   - **Baud Rate = 115200**
   - **Word Length = 8 Bits**
   - **Parity = None**
   - **Stop Bits = 1**
   - **Data Direction = Receive and Transmit**（TX 用于发时标帧；RX 用于收 `SETTIME`）
3. **GPIO**：Cube 会把 **PA9=TX，PA10=RX** 自动设为复用。  
4. **NVIC**：建议勾选 **USART1 global interrupt**（用于 `HAL_UART_RxCpltCallback` 解析命令）。

### 5.2 USART2 → Mid‑360（发送 `$GPRMC`）
1. 打开 **Connectivity → USART2**，勾选 **Asynchronous**。  
2. 参数：
   - **Baud Rate = 9600**
   - **Word Length = 8 Bits**
   - **Parity = None**
   - **Stop Bits = 1**
   - **Data Direction = Receive and Transmit**（只发送；如果 Cube 不允许只选 TX，选 TX&RX 也没关系，代码里只用 TX）
3. **GPIO**：Cube 会把 **PA2=TX** 自动设为复用（PA3=RX 可不使用）。  
4. NVIC 对 USART2 不强制开启（本项目不从它接收）。

> **Remap 提醒**：保持 **TIM3/USART1/USART2** 全部为 **“No Remap”**（默认），确保使用 **PA6/PA7/PA9/PA10/PA2** 这些你已经接线的引脚。

---

## 6. 生成代码
1. 点 **Save** 或 `Project → Generate Code`。  
2. 可到 **Project Manager → Code Generator**：勾选 *Generate peripheral initialization as a pair of .c/.h files per peripheral*（可选），不影响功能。  
3. 工程生成后，`Core/Src/main.c` 会包含 `SystemClock_Config / MX_GPIO_Init / MX_TIM3_Init / MX_USART1_UART_Init / MX_USART2_UART_Init` 等函数。

> 你可以把我给你的 **完整 `main.c`** 直接覆盖（或把“用户代码”部分粘贴进去），编译即可。

---

## 7. 构建与烧录
1. 右键工程 → **Build Project**（或锤子图标）。  
2. **ST‑Link 连线**：SWCLK、SWDIO、GND、3V3（3V3 只作目标电压检测）。`BOOT0=0`。  
3. 点 **Debug** 或 **Run** 下载到板子。若连接异常：
   - 先 **电源循环**（重新插拔 Micro‑USB）
   - 在 **Debug Configurations… → ST‑Link GDB Server** 里勾选 **Connect Under Reset** 再试
   - 确认 `SYS → Debug = Serial Wire`（JTAG 已关闭）

---

> 下面给出**可直接粘贴**的完整 `main.c` 源码，已包含：双路 PWM 启动、`$GPRMC` 生成与校验、Seeker 时标帧构造与 CRC32、`SETTIME` 命令解析、所有 HAL 初始化函数（`SystemClock_Config` / `MX_TIM3_Init` / `MX_USART1_UART_Init` / `MX_USART2_UART_Init` / `MX_GPIO_Init`）。

---

## 完整 STM32 源码（可直接粘贴编译）

> **说明**  
> - 工程基于 HAL 库；把这份 `main.c` 覆盖到 `Core/Src/main.c` 即可（保持 `main.h` 为 Cube 生成版本）。  
> - 如果编译器报某些警告/细节差异，请以你本地 HAL 版本为准微调。

```c
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
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* --- 运行时状态 --- */
volatile bool     pps_tick_flag = false;     /* TIM3 每秒触发一次 */
volatile uint64_t utc_seconds   = 1730803200;/* 上电默认UTC秒(示例)，可被 SETTIME 更新 */
volatile uint32_t last_crc32    = 0;

/* --- 串口1命令接收（SETTIME） --- */
static volatile uint8_t uart1_rx_byte = 0;
static char cmd_buf[64];
static int  cmd_len = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* 额外添加：NVIC 统一使能（若你在 Cube 里没勾选，也能保证中断可用） */
static void MX_NVIC_Init(void);

/* 工具函数 */
static bool is_leap(int y);
static void unix_to_utc(uint64_t sec, int *Y,int *M,int *D,int *h,int *m,int *s);
static void send_mid360_gprmc(uint64_t sec);
static uint32_t crc32_le_uint64_pair(uint64_t a, uint64_t b);
static void send_seeker_timestamp(uint64_t sec, uint64_t nsec);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 每秒一次回调：TIM3 更新中断 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    utc_seconds++;
    pps_tick_flag = true;
  }
}

/* 串口1接收回调：解析 SETTIME 命令 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint8_t c = uart1_rx_byte;

    if (c == '\r' || c == '\n')
    {
      if (cmd_len > 0)
      {
        cmd_buf[cmd_len] = 0;
        if (strncmp(cmd_buf, "SETTIME ", 8) == 0)
        {
          uint64_t t = 0;
          /* 注意：Keil/ARMCC/GCC 均支持 %llu / SCNu64，这里用标准写法 */
          sscanf(cmd_buf + 8, "%" SCNu64, &t);
          utc_seconds = t;
          const char ok[] = "[OK] UTC updated\r\n";
          HAL_UART_Transmit(&huart1, (uint8_t*)ok, sizeof(ok) - 1, 50);
        }
        else
        {
          const char er[] = "[ERR] use: SETTIME <unix>\r\n";
          HAL_UART_Transmit(&huart1, (uint8_t*)er, sizeof(er) - 1, 50);
        }
        cmd_len = 0;
      }
    }
    else
    {
      if (cmd_len < (int)sizeof(cmd_buf) - 1)
      {
        cmd_buf[cmd_len++] = (char)c;
      }
      else
      {
        cmd_len = 0; /* 溢出则清空 */
      }
    }

    /* 继续接收下一个字节 */
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart1_rx_byte, 1);
  }
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* 如果 Cube 没有生成 NVIC 使能（你截图里没看到），这里手动打开 */
  MX_NVIC_Init();

  /* 启动 TIM3 双路 PWM：PA6 -> Seeker.PPS，PA7 -> Mid-360.PPS */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* 开启 TIM3 “更新中断”，每秒触发一次 HAL_TIM_PeriodElapsedCallback */
  HAL_TIM_Base_Start_IT(&htim3);

  /* 开启 USART1 中断接收（用于 SETTIME 命令） */
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart1_rx_byte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 每秒在 PPS 上升沿之后：先发 Mid-360 的 $GPRMC，再发 Seeker 的时标帧 */
    if (pps_tick_flag)
    {
      pps_tick_flag = false;

      /* 1) Mid-360: $GPRMC（USART2，9600/8N1）*/
      send_mid360_gprmc(utc_seconds);

      /* 2) Seeker: 时标帧（USART1，115200/8N1），nsec=0 表示秒沿对齐 */
      send_seeker_timestamp(utc_seconds, 0);
    }
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; /* PCLK1=36MHz, TIM3CLK=72MHz(×2 规则) */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; /* PCLK2=72MHz */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Prescaler = 7199;                       /* 72MHz / (7199+1) = 10kHz */
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;                          /* 10kHz / (9999+1) = 1Hz */
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; /* = No Division */
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
  sConfigOC.Pulse = 200;                             /* 高电平 ~20ms */
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

  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;        /* TX 发给 Seeker, RX 用于 SETTIME */
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
  huart2.Init.Mode = UART_MODE_TX_RX;        /* 只用 TX 给 Mid-360 发送 $GPRMC，留 RX 不影响 */
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* TIM3/USART 的具体引脚复用配置由 HAL_TIM_MspPostInit() / HAL_UART_MspInit() 完成 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* --- NVIC 统一初始化：使能 TIM3 和 USART1 中断 --- */
static void MX_NVIC_Init(void)
{
  /* 如果你已经在 Cube 里勾了 NVIC，这里重复使能也无妨 */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* --- 工具函数实现 --- */
static bool is_leap(int y){ return (y%4==0 && y%100!=0) || (y%400==0); }

/* 简单 UNIX 秒到 UTC（不考虑闰秒） */
static void unix_to_utc(uint64_t sec, int *Y,int *M,int *D,int *h,int *m,int *s)
{
  uint64_t z=sec;
  *s = z%60; z/=60;
  *m = z%60; z/=60;
  *h = z%24; z/=24;
  int y=1970;
  while (1) {
    int days = is_leap(y) ? 366 : 365;
    if (z >= (uint64_t)days) z -= days, y++;
    else break;
  }
  *Y = y;
  static const int mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  int mi=0;
  while (1) {
    int d = mdays[mi] + (mi==1 && is_leap(y) ? 1 : 0);
    if (z >= (uint64_t)d) z -= d, mi++;
    else break;
  }
  *M = mi+1;
  *D = (int)z + 1;
}

/* 发送 $GPRMC（USART2, 9600/8N1）——仅授时，经纬度置零也可 */
static void send_mid360_gprmc(uint64_t sec)
{
  int Y,M,D,h,m,s;
  unix_to_utc(sec, &Y,&M,&D,&h,&m,&s);

  char payload[96];
  /* 形如：GPRMC,hhmmss.000,A,0000.0000,N,00000.0000,E,0.0,0.0,DDMMYY,,,A */
  snprintf(payload, sizeof(payload),
    "GPRMC,%02d%02d%02d.000,A,0000.0000,N,00000.0000,E,0.0,0.0,%02d%02d%02d,,,A",
    h,m,s, D,M,(Y%100));

  /* NMEA 校验：payload（不含$与*）逐字节异或 */
  unsigned char cs = 0;
  for (size_t i=0; i<strlen(payload); ++i) cs ^= (unsigned char)payload[i];

  char sentence[128];
  snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", payload, cs);

  HAL_UART_Transmit(&huart2, (uint8_t*)sentence, (uint16_t)strlen(sentence), 100);
}

/* CRC32(LE, poly=0xEDB88320)，覆盖 sec 与 nsec（各8字节） */
static uint32_t crc32_le_uint64_pair(uint64_t a, uint64_t b)
{
  uint32_t crc = 0xFFFFFFFFu;

  for (int k=0; k<8; ++k) {
    uint8_t x = (uint8_t)((a >> (8*k)) & 0xFF);
    crc ^= x;
    for (int i=0; i<8; ++i) {
      if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320u;
      else         crc = (crc >> 1);
    }
  }
  for (int k=0; k<8; ++k) {
    uint8_t x = (uint8_t)((b >> (8*k)) & 0xFF);
    crc ^= x;
    for (int i=0; i<8; ++i) {
      if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320u;
      else         crc = (crc >> 1);
    }
  }
  return ~crc;
}

/* 发送 Seeker 时标帧（与 Seeker 仓库 pps 代码一致）：小端序
   struct pps_packet {
       uint32_t header; // 0x000000AA
       uint64_t sec;
       uint64_t nsec;
       uint32_t footer; // CRC32(sec+nsec)
   } __attribute__((packed));
*/
static void send_seeker_timestamp(uint64_t sec, uint64_t nsec)
{
  uint8_t  buf[4+8+8+4];
  uint32_t header = 0xAA;
  uint32_t crc    = crc32_le_uint64_pair(sec, nsec);

  /* header(LE) */
  for (int i=0;i<4;i++)  buf[i]     = (uint8_t)((header >> (8*i)) & 0xFF);
  /* sec(LE) */
  for (int i=0;i<8;i++)  buf[4+i]   = (uint8_t)((sec    >> (8*i)) & 0xFF);
  /* nsec(LE) */
  for (int i=0;i<8;i++)  buf[12+i]  = (uint8_t)((nsec   >> (8*i)) & 0xFF);
  /* crc(LE) */
  for (int i=0;i<4;i++)  buf[20+i]  = (uint8_t)((crc    >> (8*i)) & 0xFF);

  HAL_UART_Transmit(&huart1, buf, sizeof(buf), 100);
  last_crc32 = crc;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

```

---

## 上电烧录与自检

1. **烧录**：用 ST-Link 连接 SWD（SWCLK/SWDIO/GND/3V3），CubeIDE `Run/Debug` 下载。  
2. **PPS 检测**：示波器测 **PA6/PA7** 均为 **1 Hz**、**高电平约 20 ms**，两路上升沿同步。  
3. **Mid-360**：Pin8 收到 PPS，Pin10 每秒收到一条 `$GPRMC`。  
4. **Seeker**：GH1.25 PPS 收到脉冲；RX 收到 24 字节时标帧（4+8+8+4）。  
5. **对时（可选）**：用 USB-TTL 连接 `PA10`，向 `USART1` 发送 `SETTIME <UNIX秒>`（例如 `SETTIME 1730803200`）。

---

## Jetson 侧运行与 SLAM

- **LiDAR**：编译并运行 `livox_ros_driver2`（Mid-360 网口连通即可）。硬件吃到 **PPS + `$GPRMC`** 后，雷达内部时间戳自动锁定。  
- **相机**：按你的相机驱动/ROS 包启动；**设置 `time_sync:=false`**（因为已经硬同步）。  
- **SLAM**：运行 FAST-LIVO2，确保订阅的点云与图像话题名正确。

---

## 联调与常见问题

- **没共地 → 不同步**：务必 STM32、Seeker、Mid-360 共地。  
- **电平错误**：STM32 IO = 3.3 V，禁止 5V 直连。  
- **Mid-360 不认同步**：检查 Pin8/Pin10 接线；`$GPRMC` 是否在该秒 PPS 后紧随发出（9600bps 一帧约 70 ms）；日期/UTC 是否正确。  
- **Seeker 不认时标**：确保**24 字节格式无误**（header=0x000000AA 小端；CRC32 覆盖 sec+nsec）；波特率/8N1 一致。  
- **时间漂移**：无 GNSS 时可定期 `SETTIME` 校正；需要长期稳定可外接 GNSS（PPS+NMEA）替换本地时钟。

---

### 结语
至此，**Mid-360 + Seeker + Jetson Orin NX + STM32F103C8T6** 的硬件时间同步方案完整落地：**双路独立 PPS 输出**保障两端引脚互不影响、严格同相；**USART1 时标帧**与**USART2 `$GPRMC`** 对齐同秒发出，满足 FAST-LIVO2 对“硬同步”的要求。
