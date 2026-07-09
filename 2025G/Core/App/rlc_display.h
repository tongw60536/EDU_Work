#ifndef RLC_DISPLAY_H
#define RLC_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 初始化显示通信：配置调试串口和屏幕串口的中断接收，准备接收控制命令。 */
void rlc_display_init(void);

/* 读取并清除“开始学习”请求：主循环调用一次后该请求会被消费掉。 */
uint8_t rlc_display_take_start_learn(void);

/* 读取并清除“开始实时滤波”请求：主循环调用一次后该请求会被消费掉。 */
uint8_t rlc_display_take_start_filter(void);

/* 设置调试串口输出开关：实时滤波期间可关闭打印以减少阻塞和抖动。 */
void rlc_display_set_debug_enabled(uint8_t enabled);

/* 开始发送一轮频谱曲线前调用：清零曲线计数，并记录本轮总频点数。 */
void rlc_display_spectrum_begin(uint32_t total_points);

/* 发送一个频谱点：把幅值和相位缩放到屏幕曲线范围，并按屏幕最大点数做抽点。 */
void rlc_display_send_spectrum_point(uint32_t point_index, uint32_t total_points,
                                     float mag, float phase_deg);

/* 更新屏幕上的滤波器类型文本，例如 LOWPASS、BANDPASS、HIGHPASS 或 BANDSTOP。 */
void rlc_display_send_filter_type(const char *type_text);

/* 通过调试串口输出字符串：用于把学习结果、错误信息和系数打印到上位机。 */
void rlc_display_debug_puts(const char *text);

/* 通过调试串口输出无符号整数：不依赖 printf，避免嵌入式工程额外开销。 */
void rlc_display_debug_print_uint(uint32_t value);

/* 通过调试串口输出定点小数：scale 表示小数放大倍数，例如 1000 表示保留三位。 */
void rlc_display_debug_print_fixed(float value, uint32_t scale);

/* 通过调试串口输出科学计数法浮点数：用于打印较大或较小的拟合系数。 */
void rlc_display_debug_print_sci(float value);

/* 处理 UART 接收完成：匹配上位机和屏幕发来的命令，并重新启动下一字节接收。 */
void rlc_display_uart_rx_cplt_callback(UART_HandleTypeDef *huart);

/* 处理 UART 错误：清除错误标志、刷新接收数据寄存器，并恢复中断接收。 */
void rlc_display_uart_error_callback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* RLC_DISPLAY_H */
