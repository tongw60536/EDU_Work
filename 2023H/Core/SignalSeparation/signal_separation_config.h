#ifndef SIGNAL_SEPARATION_CONFIG_H
#define SIGNAL_SEPARATION_CONFIG_H

/*
 * 信号分离模块配置。
 *
 * 模块先通过 ADC 采样混合输入信号，识别其中两个主要频率分量，再通过
 * 两个 DAC 通道重新生成分离后的信号。下面所有频率和相位计算都以
 * SIGSEP_SAMPLE_RATE_HZ 为基准。
 */

/* TIM2 驱动的 ADC/DAC 统一采样率。 */
#define SIGSEP_SAMPLE_RATE_HZ          2500000U

/* 每个处理帧分析的 ADC 采样点数。 */
#define SIGSEP_FRAME_LEN               500U

/* ADC 环形 DMA 缓冲区长度：两帧，对应半传输和全传输回调。 */
#define SIGSEP_ADC_DMA_LEN             (SIGSEP_FRAME_LEN * 2U)

/* 每个 DAC 通道的环形 DMA 缓冲区长度。 */
#define SIGSEP_DAC_FRAME_LEN           2000U

/* DAC 半缓冲区长度；DMA 释放某半区后由任务重新填充。 */
#define SIGSEP_DAC_HALF_LEN            (SIGSEP_DAC_FRAME_LEN / 2U)

/* 正弦查找表地址位数。 */
#define SIGSEP_SINE_LUT_BITS           10U

/* 重新生成正弦波时使用的查找表点数。 */
#define SIGSEP_SINE_LUT_SIZE           (1U << SIGSEP_SINE_LUT_BITS)

/* 将 Q32 相位累加器值转换为正弦表索引的右移位数。 */
#define SIGSEP_SINE_LUT_SHIFT          (32U - SIGSEP_SINE_LUT_BITS)

/* 分量搜索时的最低候选频率。 */
#define SIGSEP_FREQ_MIN_HZ             10000U

/* 相邻候选频点之间的频率间隔。 */
#define SIGSEP_FREQ_STEP_HZ            5000U

/* 从最低频率开始向上检查的候选频点数量。 */
#define SIGSEP_FREQ_COUNT              19U

/* 正弦/余弦步进表支持的最高频点索引。 */
#define SIGSEP_MAX_BIN                 100U

/* 初始识别时用于平均的帧数，平均后再确定最强的两个分量。 */
#define SIGSEP_IDENTIFY_FRAMES         4U

/* 12 位 DAC 输出的中点码值。 */
#define SIGSEP_DAC_MID                 2048U

/* 12 位 DAC 输出的最大码值。 */
#define SIGSEP_DAC_MAX                 4095U

/* 将测得的 16 位 ADC 幅度换算为对应 DAC 幅度。 */
#define SIGSEP_ADC_TO_DAC_SCALE        (4095.0f / 65535.0f)

/* 测得分量较弱时使用的最小再生输出幅度。 */
#define SIGSEP_DEFAULT_DAC_AMP         700.0f

/* DAC 再生波形的幅度上限，防止输出越界。 */
#define SIGSEP_MAX_DAC_AMP             1850.0f

/* 正值表示该 DAC 通道相对于锁定到的输入分量提前。 */
#define SIGSEP_DAC1_PHASE_OFFSET_DEG   0
#define SIGSEP_DAC2_PHASE_OFFSET_DEG   0

/*
 * 当两个分离分量来自同一个相干信号源时，只锁定一个主相位环路，
 * 另一个通道根据同一个时间误差推导修正量。
 * 通道 0 为 A'，通道 1 为 B'。
 */
#define SIGSEP_COMMON_SOURCE_LOCK      1U
#define SIGSEP_PHASE_MASTER_CH         0U

/* 低于该 ADC 幅度的分量认为太弱，不再进行可靠的波形分类。 */
#define SIGSEP_MIN_VALID_ADC_AMP       120.0f

/* 用于判定三角波的三次谐波比例阈值。 */
#define SIGSEP_TRI_H3_RATIO            0.060f

/* 当三次谐波与另一个分量重叠时，改用五次谐波比例阈值。 */
#define SIGSEP_TRI_H5_RATIO            0.025f

/* 相位参考的比例修正强度；右移越大，相位修正越慢。 */
#define SIGSEP_PLL_PHASE_KP_SHIFT      1U

/* NCO 频率步进比例修正的分频系数。 */
#define SIGSEP_PLL_STEP_KP_DIV         20U

/* NCO 频率步进长期积分修正的分频系数。 */
#define SIGSEP_PLL_STEP_KI_DIV         200U

/* PLL 步进修正上限为 nominal_step / 该值。 */
#define SIGSEP_PLL_MAX_CORR_DIV        2000U

/* PLL 积分器累加值的绝对上限。 */
#define SIGSEP_PLL_INTEGRATOR_LIMIT    8589934592LL

/* 积分泄漏系数分子，分母为 65536，用于防止历史误差无限累积。 */
#define SIGSEP_PLL_INTEGRATOR_LEAK_NUM 65535U

/* 幅度平滑系数：每次更新向新值移动 1 / (2^shift)。 */
#define SIGSEP_AMP_SMOOTH_SHIFT        3U

/* 模块调试/状态打印使用的 UART 发送超时时间。 */
#define SIGSEP_UART_TIMEOUT_MS         100U

#endif
