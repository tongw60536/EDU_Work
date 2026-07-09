#ifndef RLC_FILTER_H
#define RLC_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 初始化滤波硬件链路：主要完成 ADC 校准，为后续扫频采样和实时滤波做准备。 */
void rlc_filter_init(void);

/* 启动学习流程：停止可能正在运行的实时滤波，然后执行频率扫描并拟合 RLC 模型。 */
void rlc_filter_start_learning(void);

/* 启动实时滤波：使用学习阶段得到的模型和 IIR 系数，配置 ADC/DAC 双缓冲流水。 */
void rlc_filter_start_realtime(void);

/* 停止实时滤波：关闭定时器、ADC DMA 和 DAC DMA，并恢复调试输出。 */
void rlc_filter_stop_realtime(void);

/* 滤波后台任务：主循环周期调用，用来处理 DMA 中断留下的半缓冲数据块。 */
void rlc_filter_task(void);

/* 查询实时滤波状态：返回非零表示 ADC/DAC/IIR 实时处理链路正在运行。 */
uint8_t rlc_filter_is_running(void);

/* ADC 全缓冲完成处理：扫频时标记采样完成，实时滤波时调度后半缓冲处理。 */
void rlc_filter_adc_conv_cplt_callback(ADC_HandleTypeDef *hadc);

/* ADC 半缓冲完成处理：实时滤波时调度前半缓冲处理，配合双缓冲降低延迟。 */
void rlc_filter_adc_conv_half_cplt_callback(ADC_HandleTypeDef *hadc);

/* ADC 错误处理：记录错误状态，让扫频或实时流程可以感知并采取恢复动作。 */
void rlc_filter_adc_error_callback(ADC_HandleTypeDef *hadc);

/* DAC 全缓冲完成处理：统计 DAC 输出轮次，用于扫频等待和实时诊断。 */
void rlc_filter_dac_conv_cplt_ch1_callback(DAC_HandleTypeDef *hdac);

/* DAC 半缓冲完成处理：实时滤波时统计输出半缓冲节奏，辅助判断是否跟上采样。 */
void rlc_filter_dac_conv_half_cplt_ch1_callback(DAC_HandleTypeDef *hdac);

/* DAC 错误处理：统计 DMA 欠载/错误次数，并清除 DAC 通道 1 的错误标志。 */
void rlc_filter_dac_error_ch1_callback(DAC_HandleTypeDef *hdac);

#ifdef __cplusplus
}
#endif

#endif /* RLC_FILTER_H */
