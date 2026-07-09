#include "rlc_app.h"
#include "rlc_display.h"
#include "rlc_filter.h"

typedef enum
{
  RLC_APP_STATE_IDLE = 0U,
  RLC_APP_STATE_LEARNING,
  RLC_APP_STATE_FILTERING
} rlc_app_state_t;

static rlc_app_state_t app_state = RLC_APP_STATE_IDLE;

/* 初始化应用层：先初始化滤波和显示模块，再把状态机置为空闲并通过调试串口提示可接收命令。 */
void rlc_app_init(void)
{
  rlc_filter_init();
  rlc_display_init();
  app_state = RLC_APP_STATE_IDLE;
  rlc_display_debug_puts("CMD_READY,start_learn,start_filter\r\n");
}

/* 应用调度函数：先读取屏幕命令标志，收到学习命令就停止实时滤波并执行扫频学习；收到滤波命令就启动实时 IIR；最后处理滤波后台任务。 */
void rlc_app_task(void)
{
  if (rlc_display_take_start_learn() != 0U)
  {
    if (app_state == RLC_APP_STATE_FILTERING)
    {
      rlc_filter_stop_realtime();
    }

    app_state = RLC_APP_STATE_LEARNING;
    rlc_filter_start_learning();
    app_state = RLC_APP_STATE_IDLE;
    return;
  }

  if ((rlc_display_take_start_filter() != 0U) && (app_state != RLC_APP_STATE_FILTERING))
  {
    rlc_filter_start_realtime();
    if (rlc_filter_is_running() != 0U)
    {
      app_state = RLC_APP_STATE_FILTERING;
    }
    else
    {
      app_state = RLC_APP_STATE_IDLE;
    }
  }

  rlc_filter_task();
}

/* UART 接收完成回调：HAL 中断进入这里后，直接转交给显示模块解析 USART1/USART3 收到的命令字节。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  rlc_display_uart_rx_cplt_callback(huart);
}

/* UART 错误回调：发生溢出、噪声、帧错误等异常时，交给显示模块清标志并重新开启接收中断。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  rlc_display_uart_error_callback(huart);
}

/* ADC DMA 全缓冲完成回调：扫频模式下标记采样完成，实时模式下通知滤波模块处理后半段数据。 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  rlc_filter_adc_conv_cplt_callback(hadc);
}

/* ADC DMA 半缓冲完成回调：实时滤波时通知滤波模块处理前半段数据，形成双缓冲流水。 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  rlc_filter_adc_conv_half_cplt_callback(hadc);
}

/* ADC 错误回调：把 ADC 异常状态交给滤波模块记录，扫频等待循环会据此中止当前频点。 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  rlc_filter_adc_error_callback(hadc);
}

/* DAC DMA 全缓冲完成回调：转交滤波模块统计输出完成次数，用于扫频和实时输出诊断。 */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  rlc_filter_dac_conv_cplt_ch1_callback(hdac);
}

/* DAC DMA 半缓冲完成回调：实时滤波时转交滤波模块统计半缓冲输出节奏。 */
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  rlc_filter_dac_conv_half_cplt_ch1_callback(hdac);
}

/* DAC 错误回调：出现 DMA 欠载等错误时，交给滤波模块计数并清除 DAC 错误标志。 */
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac)
{
  rlc_filter_dac_error_ch1_callback(hdac);
}
