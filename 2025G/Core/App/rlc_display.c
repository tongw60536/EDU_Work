#include "rlc_display.h"
#include "usart.h"

#include <math.h>

#define UART1_CMD_START2           "Start_2"
#define UART1_CMD_START2_LEN       (7U)
#define SCREEN_CMD_LEARN           "start_learn"
#define SCREEN_CMD_LEARN_LEN       (11U)
#define SCREEN_CMD_FILTER          "start_filter"
#define SCREEN_CMD_FILTER_LEN      (12U)
#define SCREEN_MAG_DB_MIN          (-40.0f)
#define SCREEN_MAG_DB_MAX          (0.0f)
#define SCREEN_CURVE_MAX_VALUE     (127U)
#define SCREEN_CURVE_MAX_POINTS    (320U)

static uint8_t uart1_rx_byte;
static uint8_t screen_rx_byte;
static uint8_t uart1_start2_match;
static uint8_t screen_learn_match;
static uint8_t screen_filter_match;
static volatile uint8_t start_learn_requested;
static volatile uint8_t start_filter_requested;
static volatile uint8_t debug_enabled = 1U;
static uint32_t curve_sent_count;
static uint32_t curve_total_points;

/* 计算字符串长度：逐字节查找 '\0'，这样发送串口时不需要依赖标准库 strlen。 */
static uint32_t text_len(const char *text);

/* 向屏幕串口发送原始命令片段：只发文本本体，不自动补 Nextion 结束符。 */
static void screen_send_raw(const char *text);

/* 发送 Nextion 命令结束符：每条屏幕命令后必须补三个 0xFF 字节。 */
static void screen_send_end(void);

/* 向屏幕串口发送无符号十进制数字：手动拆分数字，避免使用 printf。 */
static void screen_send_uint(uint32_t value);

/* 向屏幕曲线控件发送一个点：先限制到 0..127，再拼出 add 命令并补结束符。 */
static void screen_send_curve(uint32_t channel, uint32_t value);

/* 幅值缩放函数：先把线性幅值转为 dB，再裁剪到显示范围并映射到 0..127。 */
static uint32_t screen_scale_mag(float mag);

/* 相位缩放函数：先把角度规整到 -180..180 度，再映射到屏幕曲线的 0..127。 */
static uint32_t screen_scale_phase(float phase_deg);

/* 曲线抽点判断：根据当前频点序号和总点数，决定是否发送到屏幕，避免超过 320 点上限。 */
static uint8_t screen_curve_point_allowed(uint32_t point_index, uint32_t total_points);

/* 屏幕命令匹配：逐字节匹配 start_learn 和 start_filter，匹配成功后置位对应请求标志。 */
static void screen_match_byte(uint8_t value);

/* 初始化显示通信：清空命令匹配状态和请求标志，分别复位 USART1/USART3 接收，再开启单字节中断接收。 */
void rlc_display_init(void)
{
  uart1_start2_match = 0U;
  screen_learn_match = 0U;
  screen_filter_match = 0U;
  start_learn_requested = 0U;
  start_filter_requested = 0U;
  debug_enabled = 1U;

  (void)HAL_UART_AbortReceive(&huart1);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
  __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  (void)HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1U);

  (void)HAL_UART_AbortReceive(&huart3);
  __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
  __HAL_UART_SEND_REQ(&huart3, UART_RXDATA_FLUSH_REQUEST);
  HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  (void)HAL_UART_Receive_IT(&huart3, &screen_rx_byte, 1U);
}

/* 读取“开始学习”请求：关中断保护共享标志，取出后立即清零，防止同一条命令被重复执行。 */
uint8_t rlc_display_take_start_learn(void)
{
  uint8_t requested;

  __disable_irq();
  requested = start_learn_requested;
  start_learn_requested = 0U;
  __enable_irq();

  return requested;
}

/* 读取“开始滤波”请求：用临界区保护标志位，返回本次是否收到启动实时滤波命令。 */
uint8_t rlc_display_take_start_filter(void)
{
  uint8_t requested;

  __disable_irq();
  requested = start_filter_requested;
  start_filter_requested = 0U;
  __enable_irq();

  return requested;
}

/* 设置调试输出开关：enabled 非零时允许发送调试信息，为 0 时所有调试打印函数直接返回。 */
void rlc_display_set_debug_enabled(uint8_t enabled)
{
  debug_enabled = enabled;
}

/* 开始一轮频谱显示：清空已发送点计数，并保存总频点数供后续抽点计算使用。 */
void rlc_display_spectrum_begin(uint32_t total_points)
{
  curve_sent_count = 0U;
  curve_total_points = total_points;
}

/* 发送频谱点到屏幕：若该点通过抽点判断，就分别把幅值和相位缩放后送入两个曲线通道。 */
void rlc_display_send_spectrum_point(uint32_t point_index, uint32_t total_points,
                                     float mag, float phase_deg)
{
  if (total_points == 0U)
  {
    total_points = curve_total_points;
  }
  if (screen_curve_point_allowed(point_index, total_points) == 0U)
  {
    return;
  }

  screen_send_curve(0U, screen_scale_mag(mag));
  screen_send_curve(1U, screen_scale_phase(phase_deg));
}

/* 更新滤波器类型显示：拼接 Nextion 文本赋值命令 t0.txt="..."，最后发送结束符。 */
void rlc_display_send_filter_type(const char *type_text)
{
  screen_send_raw("t0.txt=\"");
  screen_send_raw(type_text);
  screen_send_raw("\"");
  screen_send_end();
}

/* 调试字符串输出：先检查 debug_enabled，允许输出时按字符串长度阻塞发送到 USART1。 */
void rlc_display_debug_puts(const char *text)
{
  if (debug_enabled == 0U)
  {
    return;
  }
  HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)text_len(text), HAL_MAX_DELAY);
}

/* 调试整数输出：把整数反向拆成十进制字符，再倒序逐字节发出；0 单独处理。 */
void rlc_display_debug_print_uint(uint32_t value)
{
  char text[11];
  uint32_t index = 0U;

  if (debug_enabled == 0U)
  {
    return;
  }
  if (value == 0U)
  {
    rlc_display_debug_puts("0");
    return;
  }

  while ((value > 0U) && (index < sizeof(text)))
  {
    text[index++] = (char)('0' + (value % 10U));
    value /= 10U;
  }

  while (index > 0U)
  {
    index--;
    HAL_UART_Transmit(&huart1, (uint8_t *)&text[index], 1U, HAL_MAX_DELAY);
  }
}

/* 调试定点数输出：先按 scale 放大并四舍五入，再分别打印整数部分和补零后的小数部分。 */
void rlc_display_debug_print_fixed(float value, uint32_t scale)
{
  int32_t scaled;
  uint32_t integer;
  uint32_t fraction;
  uint32_t divisor;

  if (debug_enabled == 0U)
  {
    return;
  }
  if (value < 0.0f)
  {
    rlc_display_debug_puts("-");
    value = -value;
  }

  scaled = (int32_t)((value * (float)scale) + 0.5f);
  integer = (uint32_t)scaled / scale;
  fraction = (uint32_t)scaled % scale;

  rlc_display_debug_print_uint(integer);
  if (scale > 1U)
  {
    rlc_display_debug_puts(".");
    for (divisor = scale / 10U; divisor > 0U; divisor /= 10U)
    {
      char digit = (char)('0' + ((fraction / divisor) % 10U));
      HAL_UART_Transmit(&huart1, (uint8_t *)&digit, 1U, HAL_MAX_DELAY);
    }
  }
}

/* 调试科学计数法输出：把数值归一化到 1..10，记录指数，再输出尾数和 e+/- 指数。 */
void rlc_display_debug_print_sci(float value)
{
  int32_t exp = 0;

  if (debug_enabled == 0U)
  {
    return;
  }
  if (value < 0.0f)
  {
    rlc_display_debug_puts("-");
    value = -value;
  }
  if (value == 0.0f)
  {
    rlc_display_debug_puts("0.000000e+0");
    return;
  }

  while (value >= 10.0f)
  {
    value *= 0.1f;
    exp++;
  }
  while (value < 1.0f)
  {
    value *= 10.0f;
    exp--;
  }

  rlc_display_debug_print_fixed(value, 1000000U);
  if (exp >= 0)
  {
    rlc_display_debug_puts("e+");
    rlc_display_debug_print_uint((uint32_t)exp);
  }
  else
  {
    rlc_display_debug_puts("e-");
    rlc_display_debug_print_uint((uint32_t)(-exp));
  }
}

/* UART 接收完成处理：USART1 匹配 Start_2 启动滤波；USART3 回显字节并匹配屏幕命令，然后继续接收下一字节。 */
void rlc_display_uart_rx_cplt_callback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    const char *cmd = UART1_CMD_START2;

    if (uart1_rx_byte == (uint8_t)cmd[uart1_start2_match])
    {
      uart1_start2_match++;
      if (uart1_start2_match >= UART1_CMD_START2_LEN)
      {
        start_filter_requested = 1U;
        uart1_start2_match = 0U;
      }
    }
    else
    {
      uart1_start2_match = (uart1_rx_byte == (uint8_t)cmd[0]) ? 1U : 0U;
    }

    (void)HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1U);
  }
  else if (huart->Instance == USART3)
  {
    (void)HAL_UART_Transmit(&huart1, &screen_rx_byte, 1U, 1U);
    screen_match_byte(screen_rx_byte);
    (void)HAL_UART_Receive_IT(&huart3, &screen_rx_byte, 1U);
  }
}

/* UART 错误恢复：按实际出错串口清错误标志、刷新接收数据，并重新挂起单字节接收中断。 */
void rlc_display_uart_error_callback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    (void)HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1U);
  }
  else if (huart->Instance == USART3)
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    (void)HAL_UART_Receive_IT(&huart3, &screen_rx_byte, 1U);
  }
}

/* 计算字符串长度：从第 0 个字符开始遍历，直到遇到字符串结束符 '\0'。 */
static uint32_t text_len(const char *text)
{
  uint32_t len = 0U;

  while (text[len] != '\0')
  {
    len++;
  }

  return len;
}

/* 发送屏幕原始文本：把字符串转换为字节流，通过 USART3 阻塞发送给屏幕。 */
static void screen_send_raw(const char *text)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)text, (uint16_t)text_len(text), HAL_MAX_DELAY);
}

/* 发送屏幕命令结束符：Nextion 协议要求命令尾部固定追加 FF FF FF。 */
static void screen_send_end(void)
{
  static const uint8_t end_bytes[3] = {0xFFU, 0xFFU, 0xFFU};

  HAL_UART_Transmit(&huart3, (uint8_t *)end_bytes, sizeof(end_bytes), HAL_MAX_DELAY);
}

/* 发送屏幕数字参数：将 uint32_t 拆成十进制 ASCII 字符，按正常顺序发送。 */
static void screen_send_uint(uint32_t value)
{
  char text[11];
  uint32_t index = 0U;

  if (value == 0U)
  {
    screen_send_raw("0");
    return;
  }

  while ((value > 0U) && (index < sizeof(text)))
  {
    text[index++] = (char)('0' + (value % 10U));
    value /= 10U;
  }

  while (index > 0U)
  {
    index--;
    HAL_UART_Transmit(&huart3, (uint8_t *)&text[index], 1U, HAL_MAX_DELAY);
  }
}

/* 发送曲线点：限制曲线值上限，拼出 add 3,channel,value 命令并发给 Nextion。 */
static void screen_send_curve(uint32_t channel, uint32_t value)
{
  if (value > SCREEN_CURVE_MAX_VALUE)
  {
    value = SCREEN_CURVE_MAX_VALUE;
  }

  screen_send_raw("add 3,");
  screen_send_uint(channel);
  screen_send_raw(",");
  screen_send_uint(value);
  screen_send_end();
}

/* 判断是否发送当前点：把原始频点均匀压缩到屏幕 320 点容量内，达到目标序号时才放行。 */
static uint8_t screen_curve_point_allowed(uint32_t point_index, uint32_t total_points)
{
  uint32_t target_sent;

  if (curve_sent_count >= SCREEN_CURVE_MAX_POINTS)
  {
    return 0U;
  }
  if (total_points == 0U)
  {
    total_points = curve_total_points;
  }
  if (total_points == 0U)
  {
    return 0U;
  }

  target_sent = ((point_index + 1U) * SCREEN_CURVE_MAX_POINTS) / total_points;
  if (target_sent >= SCREEN_CURVE_MAX_POINTS)
  {
    target_sent = SCREEN_CURVE_MAX_POINTS - 1U;
  }
  if (curve_sent_count <= target_sent)
  {
    curve_sent_count++;
    return 1U;
  }

  return 0U;
}

/* 幅值映射：对幅值取 20log10，限制在 -40dB 到 0dB，再线性映射为屏幕曲线数值。 */
static uint32_t screen_scale_mag(float mag)
{
  float db;
  float scaled;

  if (mag <= 1.0e-6f)
  {
    db = SCREEN_MAG_DB_MIN;
  }
  else
  {
    db = 20.0f * log10f(mag);
  }

  if (db < SCREEN_MAG_DB_MIN)
  {
    db = SCREEN_MAG_DB_MIN;
  }
  if (db > SCREEN_MAG_DB_MAX)
  {
    db = SCREEN_MAG_DB_MAX;
  }

  scaled = ((db - SCREEN_MAG_DB_MIN) * (float)SCREEN_CURVE_MAX_VALUE) /
           (SCREEN_MAG_DB_MAX - SCREEN_MAG_DB_MIN);
  if (scaled < 0.0f)
  {
    scaled = 0.0f;
  }
  if (scaled > (float)SCREEN_CURVE_MAX_VALUE)
  {
    scaled = (float)SCREEN_CURVE_MAX_VALUE;
  }

  return (uint32_t)(scaled + 0.5f);
}

/* 相位映射：通过加减 360 度消除越界，再把 -180..180 度线性变为 0..127。 */
static uint32_t screen_scale_phase(float phase_deg)
{
  float scaled;

  while (phase_deg > 180.0f)
  {
    phase_deg -= 360.0f;
  }
  while (phase_deg < -180.0f)
  {
    phase_deg += 360.0f;
  }

  scaled = ((phase_deg + 180.0f) * (float)SCREEN_CURVE_MAX_VALUE) / 360.0f;
  if (scaled < 0.0f)
  {
    scaled = 0.0f;
  }
  if (scaled > (float)SCREEN_CURVE_MAX_VALUE)
  {
    scaled = (float)SCREEN_CURVE_MAX_VALUE;
  }

  return (uint32_t)(scaled + 0.5f);
}

/* 命令字节匹配：分别维护学习和滤波命令的匹配位置，连续匹配完整字符串后置位请求。 */
static void screen_match_byte(uint8_t value)
{
  const char *learn = SCREEN_CMD_LEARN;
  const char *filter = SCREEN_CMD_FILTER;

  if (value == (uint8_t)learn[screen_learn_match])
  {
    screen_learn_match++;
    if (screen_learn_match >= SCREEN_CMD_LEARN_LEN)
    {
      start_learn_requested = 1U;
      screen_learn_match = 0U;
    }
  }
  else
  {
    screen_learn_match = (value == (uint8_t)learn[0]) ? 1U : 0U;
  }

  if (value == (uint8_t)filter[screen_filter_match])
  {
    screen_filter_match++;
    if (screen_filter_match >= SCREEN_CMD_FILTER_LEN)
    {
      start_filter_requested = 1U;
      screen_filter_match = 0U;
    }
  }
  else
  {
    screen_filter_match = (value == (uint8_t)filter[0]) ? 1U : 0U;
  }
}
