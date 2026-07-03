#ifndef SIGNAL_SEPARATION_H
#define SIGNAL_SEPARATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 启动信号分离引擎。
 *
 * 该函数会配置 TIM2 采样时钟，准备 CORDIC 查表数据，启动 ADC DMA 采样，
 * 启动两个 DAC 通道的 DMA 输出，最后启动 TIM2，使 ADC 和 DAC 共用同一
 * 个时间基准。
 */
void SignalSeparation_Start(void);

/*
 * 主循环中的非中断处理函数。
 *
 * 需要在主循环中反复调用。它会处理已经采集完成的 ADC 数据帧，
 * 完成两个分量的识别或跟踪，并在 DAC DMA 回调确认半缓冲区可写后
 * 重新填充对应的 DAC 数据。
 */
void SignalSeparation_Task(void);

/*
 * 强制模块回到频率识别状态。
 *
 * 已有的分离结果会被清除，两个 DAC 缓冲区会回到中点电平，直到新的
 * ADC 数据帧足够完成下一次识别。
 */
void SignalSeparation_RestartIdentify(void);

/*
 * 设置额外施加到 DAC 通道 2 的输出相位偏移。
 *
 * 输入角度会被限制在允许范围内，并按配置步进取整。正值表示通道 2
 * 相对于锁定到的输入分量提前。
 */
void SignalSeparation_SetPhaseOffsetDeg(int32_t deg);

/*
 * 获取当前通道 2 输出相位偏移，单位为度。
 */
int32_t SignalSeparation_GetPhaseOffsetDeg(void);

/*
 * 读取已经识别出的两个分量频率。
 *
 * 如果频率有效，返回 1，并把结果写入非 NULL 的输出指针；如果识别尚未
 * 完成，则返回 0。
 */
uint8_t SignalSeparation_GetFrequencies(uint32_t *freq0_hz, uint32_t *freq1_hz);

#ifdef __cplusplus
}
#endif

#endif
