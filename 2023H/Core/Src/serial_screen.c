#include "serial_screen.h"

#include "usart.h"

#include <stdio.h>
#include <string.h>

#define SERIAL_SCREEN_RX_BUF_LEN       64U
#define SERIAL_SCREEN_CMD_BUF_LEN      64U
#define SERIAL_SCREEN_UART_TIMEOUT_MS  100U
#define SERIAL_SCREEN_PHASE_HEADER_LEN 6U

static uint8_t rx_buf[SERIAL_SCREEN_RX_BUF_LEN];
static uint8_t rx_pending_buf[SERIAL_SCREEN_RX_BUF_LEN];
static volatile uint16_t rx_pending_len = 0U;
static volatile uint8_t rx_pending = 0U;
static volatile uint32_t rx_overrun = 0U;
static volatile uint8_t separate_requested = 0U;
static volatile uint8_t phase_set_requested = 0U;
static volatile int32_t phase_set_deg = 0;

static char cmd_buf[SERIAL_SCREEN_CMD_BUF_LEN];
static uint16_t cmd_len = 0U;
static uint8_t phase_header_index = 0U;
static uint8_t phase_value_pending = 0U;

static void SerialScreen_StartReceive(void);
static void SerialScreen_ProcessRx(const uint8_t *data, uint16_t len);
static void SerialScreen_ProcessPhaseByte(uint8_t ch);
static void SerialScreen_SendTextValue(const char *obj, uint32_t freq_hz);

void SerialScreen_Init(void)
{
  rx_pending_len = 0U;
  rx_pending = 0U;
  rx_overrun = 0U;
  separate_requested = 0U;
  phase_set_requested = 0U;
  phase_set_deg = 0;
  cmd_len = 0U;
  cmd_buf[0] = '\0';
  phase_header_index = 0U;
  phase_value_pending = 0U;

  HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  SerialScreen_StartReceive();
}

void SerialScreen_Task(void)
{
  uint8_t local_buf[SERIAL_SCREEN_RX_BUF_LEN];
  uint16_t local_len = 0U;
  uint16_t i;

  __disable_irq();
  if (rx_pending != 0U)
  {
    local_len = rx_pending_len;
    for (i = 0U; i < local_len; i++)
    {
      local_buf[i] = rx_pending_buf[i];
    }
    rx_pending_len = 0U;
    rx_pending = 0U;
  }
  __enable_irq();

  if (local_len == 0U)
  {
    return;
  }

  (void)HAL_UART_Transmit(&huart1, local_buf, local_len, SERIAL_SCREEN_UART_TIMEOUT_MS);
  SerialScreen_ProcessRx(local_buf, local_len);
}

uint8_t SerialScreen_TakeSeparateRequest(void)
{
  uint8_t requested;

  __disable_irq();
  requested = separate_requested;
  separate_requested = 0U;
  __enable_irq();

  return requested;
}

uint8_t SerialScreen_TakePhaseSetRequest(int32_t *phase_deg)
{
  uint8_t requested;

  __disable_irq();
  requested = phase_set_requested;
  phase_set_requested = 0U;
  if ((requested != 0U) && (phase_deg != NULL))
  {
    *phase_deg = phase_set_deg;
  }
  __enable_irq();

  return requested;
}

void SerialScreen_SendFrequencies(uint32_t freq0_hz, uint32_t freq1_hz)
{
  SerialScreen_SendTextValue("t3", freq0_hz);
  SerialScreen_SendTextValue("t4", freq1_hz);
}

static void SerialScreen_StartReceive(void)
{
  if (HAL_UARTEx_ReceiveToIdle_IT(&huart3, rx_buf, SERIAL_SCREEN_RX_BUF_LEN) != HAL_OK)
  {
    Error_Handler();
  }
}

static void SerialScreen_ProcessRx(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  for (i = 0U; i < len; i++)
  {
    uint8_t ch = data[i];

    SerialScreen_ProcessPhaseByte(ch);

    if ((ch >= 0x20U) && (ch <= 0x7EU))
    {
      if (cmd_len >= (SERIAL_SCREEN_CMD_BUF_LEN - 1U))
      {
        memmove(&cmd_buf[0], &cmd_buf[1], SERIAL_SCREEN_CMD_BUF_LEN - 2U);
        cmd_len = SERIAL_SCREEN_CMD_BUF_LEN - 2U;
      }
      cmd_buf[cmd_len++] = (char)ch;
      cmd_buf[cmd_len] = '\0';

      if (strstr(cmd_buf, "Key:Seperate") != NULL)
      {
        separate_requested = 1U;
        cmd_len = 0U;
        cmd_buf[0] = '\0';
      }
    }
    else if ((ch == '\r') || (ch == '\n') || (ch == 0x00U) || (ch == 0xFFU))
    {
      cmd_len = 0U;
      cmd_buf[0] = '\0';
    }
  }
}

static void SerialScreen_ProcessPhaseByte(uint8_t ch)
{
  static const uint8_t header[SERIAL_SCREEN_PHASE_HEADER_LEN] =
  {
    'P', 'h', 'a', 's', 'e', ':'
  };

  if (phase_value_pending != 0U)
  {
    phase_set_deg = (int32_t)ch;
    phase_set_requested = 1U;
    phase_value_pending = 0U;
    phase_header_index = 0U;
    return;
  }

  if (ch == header[phase_header_index])
  {
    phase_header_index++;
    if (phase_header_index >= SERIAL_SCREEN_PHASE_HEADER_LEN)
    {
      phase_header_index = 0U;
      phase_value_pending = 1U;
    }
  }
  else if (ch == header[0])
  {
    phase_header_index = 1U;
  }
  else
  {
    phase_header_index = 0U;
  }
}

static void SerialScreen_SendTextValue(const char *obj, uint32_t freq_hz)
{
  static const uint8_t end_cmd[3] = {0xFFU, 0xFFU, 0xFFU};
  char cmd[32];
  int len;

  len = snprintf(cmd, sizeof(cmd), "%s.txt=\"%luKHz\"", obj, (unsigned long)(freq_hz / 1000U));
  if (len < 0)
  {
    return;
  }
  if ((uint32_t)len >= sizeof(cmd))
  {
    len = (int)sizeof(cmd) - 1;
  }

  (void)HAL_UART_Transmit(&huart3, (uint8_t *)cmd, (uint16_t)len, SERIAL_SCREEN_UART_TIMEOUT_MS);
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)end_cmd, (uint16_t)sizeof(end_cmd), SERIAL_SCREEN_UART_TIMEOUT_MS);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t i;
  uint16_t copy_len;

  if (huart->Instance != USART3)
  {
    return;
  }

  if (Size > 0U)
  {
    copy_len = Size;
    if (copy_len > SERIAL_SCREEN_RX_BUF_LEN)
    {
      copy_len = SERIAL_SCREEN_RX_BUF_LEN;
    }

    if (rx_pending == 0U)
    {
      for (i = 0U; i < copy_len; i++)
      {
        rx_pending_buf[i] = rx_buf[i];
      }
      rx_pending_len = copy_len;
      rx_pending = 1U;
    }
    else
    {
      rx_overrun++;
    }
  }

  SerialScreen_StartReceive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    SerialScreen_StartReceive();
  }
}
