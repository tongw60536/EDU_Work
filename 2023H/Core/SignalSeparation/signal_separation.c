#include "signal_separation.h"
#include "signal_separation_config.h"

#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"

#ifndef ARM_MATH_CM7
#define ARM_MATH_CM7
#endif
#include "arm_math.h"
#include "stm32h7xx_hal_dac_ex.h"

#include <stdarg.h>
#include <stdio.h>

/*
 * 信号分离运行时模块。
 *
 * 数据流：
 * 1. ADC1 通过双缓冲 DMA 采样混合输入信号。
 * 2. 任务函数分析已经释放的 ADC 数据帧，找出最强的两个频率分量，
 *    并判断每个分量是正弦波还是三角波。
 * 3. 两个 NCO 将分离后的分量重新生成到 DAC1 CH1/CH2 DMA 缓冲区。
 *    轻量级 PLL 用于保持再生输出与输入测量相位对齐。
 */

/* 运行时可调的 DAC 通道 2 相位偏移。 */
#define SIGSEP_PHASE_OFFSET_DEFAULT_DEG 150
#define SIGSEP_PHASE_OFFSET_MIN_DEG     0
#define SIGSEP_PHASE_OFFSET_MAX_DEG     180
#define SIGSEP_PHASE_OFFSET_STEP_DEG    5

/* 根据谐波含量选择的再生波形类型。 */
typedef enum
{
  WAVE_SINE = 0,
  WAVE_TRIANGLE = 1
} WaveType;

/* 当前缓存的 CORDIC 模式，用于避免每次调用都重新配置外设。 */
typedef enum
{
  CORDIC_MODE_NONE = 0,
  CORDIC_MODE_SINE,
  CORDIC_MODE_PHASE
} CordicMode;

/* 一个已识别信号分量，以及由它推导出的输出参数。 */
typedef struct
{
  /* 该分量的频率，单位 Hz。 */
  uint32_t freq_hz;

  /* 该分量在候选频率表中的索引。 */
  uint8_t freq_index;

  /* DAC 输出路径需要再生的波形类型。 */
  WaveType wave;

  /* 去直流并完成频率相关检测后的 ADC 测量幅度。 */
  float32_t amp_adc;

  /* ADC 幅度换算并限幅后的 DAC 输出幅度。 */
  float32_t amp_dac;

  /* Q32 格式的测量相位，2^32 表示一个完整周期。 */
  uint32_t phase_q32;
} SignalComponent;

/* 单个 DAC 输出通道的 NCO 和 PLL 状态。 */
typedef struct
{
  /* 对应已识别频率的理想 Q32 每采样点相位步进。 */
  uint32_t nominal_step;

  /* PLL 叠加到 nominal_step 上的频率步进修正量。 */
  int32_t step_corr;

  /* 相位误差的泄漏积分项，用于长期频率跟踪。 */
  int64_t integrator;

  /* sample_ref 对应时刻的相位锚点。 */
  uint32_t phase_ref;

  /* phase_ref 对应的绝对采样点编号。 */
  uint64_t sample_ref;

  /* 最近一次有符号相位误差，用于诊断和同源跟随。 */
  int32_t last_error;
} NcoState;

/* DMA 缓冲区保持 32 字节对齐，兼顾 STM32H7 DMA 和缓存一致性安全。 */
__ALIGNED(32) static uint16_t adc_dma_buf[SIGSEP_ADC_DMA_LEN];
__ALIGNED(32) static uint16_t dac1_dma_buf[SIGSEP_DAC_FRAME_LEN];
__ALIGNED(32) static uint16_t dac2_dma_buf[SIGSEP_DAC_FRAME_LEN];

/* 查找表和初始识别阶段使用的工作缓冲区。 */
static int16_t sine_lut[SIGSEP_SINE_LUT_SIZE];
static float32_t step_sin[SIGSEP_MAX_BIN + 1U];
static float32_t step_cos[SIGSEP_MAX_BIN + 1U];
static float32_t identify_amp_acc[SIGSEP_FREQ_COUNT];

/* 当前生效的两个分离分量及其输出振荡器状态。 */
static SignalComponent active_comp[2];
static NcoState nco_state[2];
static CordicMode cordic_mode = CORDIC_MODE_NONE;

/* ADC/DAC DMA 回调用于向任务函数传递状态的变量。 */
static volatile int32_t adc_ready_offset = -1;
static volatile uint32_t adc_frame_count = 0;
static volatile uint32_t adc_frame_overrun = 0;
static volatile uint64_t adc_sample_count = 0;
static volatile uint64_t adc_ready_sample_start = 0;
static volatile uint64_t dac_sample_count = 0;
static volatile uint64_t dac_ready_play_sample[2];
static volatile uint32_t dac_ready_mask = 0;
static volatile uint32_t dac_half_overrun = 0;

/* 主任务拥有的识别状态和输出相位设置。 */
static volatile uint8_t separation_identified = 0;
static volatile int32_t output_phase_offset_deg = SIGSEP_PHASE_OFFSET_DEFAULT_DEG;
static uint32_t identify_frame_count = 0;

static void Debug_Printf(const char *fmt, ...);
static void Startup_Print(void);
static void Print_SeparationResult(const SignalComponent comp[2]);
static const char *WaveName(WaveType wave);
static void Timer2_SetSampleRate(uint32_t sample_rate_hz);
static uint32_t Timer2_GetClockHz(void);

static void CORDIC_PrepareTables(void);
static void CORDIC_SelectSine(void);
static void CORDIC_SelectPhase(void);
static float32_t CORDIC_SinFromPhase(uint32_t phase);
static uint32_t CORDIC_PhaseFromIQ(float32_t i_part, float32_t q_part);

static void Fill_DacMidscale(void);
static void Fill_DacHalf(uint32_t half_index, uint64_t play_sample);
static void Build_DacSamples(uint16_t *dst, const SignalComponent *comp, uint32_t start_phase,
                             uint32_t phase_step, uint32_t len);

static uint8_t Analyze_Frame(const uint16_t *samples, SignalComponent out[2]);
static float32_t Frame_Mean(const uint16_t *samples);
static void Measure_Component(const uint16_t *samples, float32_t mean, uint32_t freq_hz,
                              float32_t *amp, uint32_t *phase_q32);
static float32_t Measure_AmplitudeOnly(const uint16_t *samples, float32_t mean, uint32_t freq_hz);
static float32_t Correlate_Amplitude(const uint16_t *samples, float32_t mean, uint32_t freq_hz);
static WaveType Detect_WaveType(const uint16_t *samples, float32_t mean, uint32_t freq_hz, uint32_t other_hz,
                                float32_t fundamental_amp);

static void Nco_Init(uint32_t ch, const SignalComponent *comp, uint64_t sample_start);
static uint32_t Nco_Step(uint32_t ch);
static uint32_t Nco_PhaseAtSample(uint32_t ch, uint64_t sample);
static int32_t Nco_UpdateLock(uint32_t ch, uint32_t measured_phase, uint64_t frame_start_sample);
static void Nco_FollowCommonSource(uint32_t follower_ch, uint32_t master_ch,
                                   int32_t master_phase_error, uint64_t frame_start_sample);
static int32_t Scale_PhaseErrorByFreq(int32_t phase_error, uint32_t dst_hz, uint32_t src_hz);
static int32_t Scale_StepCorrection(uint32_t follower_ch, uint32_t master_ch);
static int32_t Normalize_PhaseOffsetDeg(int32_t deg);
static uint32_t PhaseOffsetDeg_To_Q32(int32_t deg);
static uint32_t PhaseStep_Q32(uint32_t freq_hz);

/* 通过 USART1 输出格式化调试信息。 */
static void Debug_Printf(const char *fmt, ...)
{
  char buf[128];
  va_list args;
  int len;

  va_start(args, fmt);
  len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (len < 0)
  {
    return;
  }
  if ((uint32_t)len >= sizeof(buf))
  {
    len = (int)sizeof(buf) - 1;
  }

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, SIGSEP_UART_TIMEOUT_MS);
}

/* 信号路径启动后打印模块启动提示。 */
static void Startup_Print(void)
{
  Debug_Printf("\r\nsignal separation start\r\n");
}

/* 将内部波形枚举转换为简短的调试打印名称。 */
static const char *WaveName(WaveType wave)
{
  return (wave == WAVE_TRIANGLE) ? "tri" : "sin";
}

/* 通过调试串口输出两个已经识别出的分离分量。 */
static void Print_SeparationResult(const SignalComponent comp[2])
{
  Debug_Printf("A: %luHz %s | B: %luHz %s\r\n",
               (unsigned long)comp[0].freq_hz,
               WaveName(comp[0].wave),
               (unsigned long)comp[1].freq_hz,
               WaveName(comp[1].wave));
}

/* 获取 TIM2 实际输入时钟，包含 APB 定时器时钟倍频规则。 */
static uint32_t Timer2_GetClockHz(void)
{
  uint32_t clk = HAL_RCC_GetPCLK1Freq();

  if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != RCC_D2CFGR_D2PPRE1_DIV1)
  {
    clk *= 2U;
  }

  return clk;
}

/* 配置 TIM2，使 ADC 和 DAC 触发频率等于指定采样率。 */
static void Timer2_SetSampleRate(uint32_t sample_rate_hz)
{
  uint32_t tim_clk = Timer2_GetClockHz();
  uint32_t arr;

  if ((sample_rate_hz == 0U) || (tim_clk < sample_rate_hz))
  {
    Error_Handler();
  }

  arr = (tim_clk / sample_rate_hz) - 1U;
  __HAL_TIM_DISABLE(&htim2);
  __HAL_TIM_SET_PRESCALER(&htim2, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  htim2.Instance->EGR = TIM_EVENTSOURCE_UPDATE;
}

/* 选择 CORDIC 正弦模式，用于相位到正弦值的转换。 */
static void CORDIC_SelectSine(void)
{
  CORDIC_ConfigTypeDef cfg;

  if (cordic_mode == CORDIC_MODE_SINE)
  {
    return;
  }

  cfg.Function = CORDIC_FUNCTION_SINE;
  cfg.Precision = CORDIC_PRECISION_6CYCLES;
  cfg.Scale = CORDIC_SCALE_0;
  cfg.NbWrite = CORDIC_NBWRITE_1;
  cfg.NbRead = CORDIC_NBREAD_1;
  cfg.InSize = CORDIC_INSIZE_32BITS;
  cfg.OutSize = CORDIC_OUTSIZE_32BITS;
  if (HAL_CORDIC_Configure(&hcordic, &cfg) != HAL_OK)
  {
    Error_Handler();
  }
  cordic_mode = CORDIC_MODE_SINE;
}

/* 选择 CORDIC 相位模式，用于 I/Q 向量到相位的转换。 */
static void CORDIC_SelectPhase(void)
{
  CORDIC_ConfigTypeDef cfg;

  if (cordic_mode == CORDIC_MODE_PHASE)
  {
    return;
  }

  cfg.Function = CORDIC_FUNCTION_PHASE;
  cfg.Precision = CORDIC_PRECISION_6CYCLES;
  cfg.Scale = CORDIC_SCALE_0;
  cfg.NbWrite = CORDIC_NBWRITE_2;
  cfg.NbRead = CORDIC_NBREAD_1;
  cfg.InSize = CORDIC_INSIZE_32BITS;
  cfg.OutSize = CORDIC_OUTSIZE_32BITS;
  if (HAL_CORDIC_Configure(&hcordic, &cfg) != HAL_OK)
  {
    Error_Handler();
  }
  cordic_mode = CORDIC_MODE_PHASE;
}

/* 使用 STM32 CORDIC 外设根据 Q32 相位计算正弦值。 */
static float32_t CORDIC_SinFromPhase(uint32_t phase)
{
  int32_t in = (int32_t)phase;
  int32_t out = 0;

  CORDIC_SelectSine();
  if (HAL_CORDIC_Calculate(&hcordic, &in, &out, 1U, 10U) != HAL_OK)
  {
    Error_Handler();
  }

  return ((float32_t)out) / 2147483648.0f;
}

/*
 * 将相关检测得到的 I/Q 向量转换为 Q32 相位。
 *
 * 输入 CORDIC 前先对 I/Q 做归一化，避免较大的 ADC 累加值超过 CORDIC
 * 定点输入范围。
 */
static uint32_t CORDIC_PhaseFromIQ(float32_t i_part, float32_t q_part)
{
  float32_t abs_i = (i_part >= 0.0f) ? i_part : -i_part;
  float32_t abs_q = (q_part >= 0.0f) ? q_part : -q_part;
  float32_t scale = (abs_i > abs_q) ? abs_i : abs_q;
  int32_t in[2];
  int32_t out = 0;

  if (scale < 1.0f)
  {
    return 0U;
  }

  in[0] = (int32_t)((i_part / scale) * 1073741824.0f);
  in[1] = (int32_t)((q_part / scale) * 1073741824.0f);

  CORDIC_SelectPhase();
  if (HAL_CORDIC_Calculate(&hcordic, in, &out, 1U, 10U) != HAL_OK)
  {
    Error_Handler();
  }

  return (uint32_t)out;
}

/*
 * 预计算实时路径需要的查找表。
 *
 * step_sin/step_cos 用于在各个支持的频点上旋转相关检测向量；
 * sine_lut 用于后续再生正弦输出采样点。
 */
static void CORDIC_PrepareTables(void)
{
  uint32_t freq_hz;
  uint32_t phase_step;
  uint32_t i;

  step_sin[0] = 0.0f;
  step_cos[0] = 1.0f;
  for (i = 1U; i <= SIGSEP_MAX_BIN; i++)
  {
    freq_hz = i * SIGSEP_FREQ_STEP_HZ;
    phase_step = (uint32_t)(((uint64_t)freq_hz * 4294967296ULL) / SIGSEP_SAMPLE_RATE_HZ);
    step_sin[i] = CORDIC_SinFromPhase(phase_step);
    step_cos[i] = CORDIC_SinFromPhase(phase_step + 0x40000000UL);
  }

  for (i = 0U; i < SIGSEP_SINE_LUT_SIZE; i++)
  {
    uint32_t phase = i << SIGSEP_SINE_LUT_SHIFT;
    float32_t s = CORDIC_SinFromPhase(phase);
    sine_lut[i] = (int16_t)(s * 32767.0f);
  }
}

/* 将两个 DAC DMA 缓冲区置为中点电平，使输出保持静默并以中点为基准。 */
static void Fill_DacMidscale(void)
{
  uint32_t i;

  for (i = 0U; i < SIGSEP_DAC_FRAME_LEN; i++)
  {
    dac1_dma_buf[i] = SIGSEP_DAC_MID;
    dac2_dma_buf[i] = SIGSEP_DAC_MID;
  }
}

/* 计算一个 ADC 分析帧的直流均值。 */
static float32_t Frame_Mean(const uint16_t *samples)
{
  float32_t mean = 0.0f;
  uint32_t i;

  for (i = 0U; i < SIGSEP_FRAME_LEN; i++)
  {
    mean += (float32_t)samples[i];
  }

  return mean / (float32_t)SIGSEP_FRAME_LEN;
}

/*
 * 测量 ADC 帧中某个指定频率分量。
 *
 * 该函数把输入帧与 freq_hz 对应的正弦/余弦参考信号做相关检测，
 * 相关向量的模长得到幅度，I/Q 角度得到相位。
 */
static void Measure_Component(const uint16_t *samples, float32_t mean, uint32_t freq_hz,
                              float32_t *amp, uint32_t *phase_q32)
{
  uint32_t bin = freq_hz / SIGSEP_FREQ_STEP_HZ;
  float32_t s = 0.0f;
  float32_t c = 1.0f;
  float32_t sum_s = 0.0f;
  float32_t sum_c = 0.0f;
  float32_t mag = 0.0f;
  float32_t x;
  float32_t next_c;
  uint32_t i;

  if ((bin == 0U) || (bin > SIGSEP_MAX_BIN) || ((freq_hz % SIGSEP_FREQ_STEP_HZ) != 0U))
  {
    *amp = 0.0f;
    *phase_q32 = 0U;
    return;
  }

  for (i = 0U; i < SIGSEP_FRAME_LEN; i++)
  {
    x = ((float32_t)samples[i]) - mean;
    sum_s += x * s;
    sum_c += x * c;

    next_c = (c * step_cos[bin]) - (s * step_sin[bin]);
    s = (s * step_cos[bin]) + (c * step_sin[bin]);
    c = next_c;
  }

  (void)arm_sqrt_f32((sum_s * sum_s) + (sum_c * sum_c), &mag);
  *amp = (2.0f * mag) / (float32_t)SIGSEP_FRAME_LEN;
  *phase_q32 = CORDIC_PhaseFromIQ(sum_s, sum_c);
}

/* 幅度相关检测的兼容包装函数。 */
static float32_t Correlate_Amplitude(const uint16_t *samples, float32_t mean, uint32_t freq_hz)
{
  return Measure_AmplitudeOnly(samples, mean, freq_hz);
}

/*
 * 只测量某个频率分量的幅度。
 *
 * 当不需要相位信息时使用，例如波形分类或同源跟随通道的幅度跟踪。
 */
static float32_t Measure_AmplitudeOnly(const uint16_t *samples, float32_t mean, uint32_t freq_hz)
{
  uint32_t bin = freq_hz / SIGSEP_FREQ_STEP_HZ;
  float32_t s = 0.0f;
  float32_t c = 1.0f;
  float32_t sum_s = 0.0f;
  float32_t sum_c = 0.0f;
  float32_t mag = 0.0f;
  float32_t x;
  float32_t next_c;
  uint32_t i;

  if ((bin == 0U) || (bin > SIGSEP_MAX_BIN) || ((freq_hz % SIGSEP_FREQ_STEP_HZ) != 0U))
  {
    return 0.0f;
  }

  for (i = 0U; i < SIGSEP_FRAME_LEN; i++)
  {
    x = ((float32_t)samples[i]) - mean;
    sum_s += x * s;
    sum_c += x * c;

    next_c = (c * step_cos[bin]) - (s * step_sin[bin]);
    s = (s * step_cos[bin]) + (c * step_sin[bin]);
    c = next_c;
  }

  (void)arm_sqrt_f32((sum_s * sum_s) + (sum_c * sum_c), &mag);
  return (2.0f * mag) / (float32_t)SIGSEP_FRAME_LEN;
}

/*
 * 根据谐波含量判断该分量是正弦波还是三角波。
 *
 * 三角波具有更明显的奇次谐波。如果三次谐波刚好与另一个分离分量重叠，
 * 则改用五次谐波作为辅助判据。
 */
static WaveType Detect_WaveType(const uint16_t *samples, float32_t mean, uint32_t freq_hz, uint32_t other_hz,
                                float32_t fundamental_amp)
{
  float32_t h3_amp = 0.0f;
  float32_t h5_amp = 0.0f;

  if (fundamental_amp < SIGSEP_MIN_VALID_ADC_AMP)
  {
    return WAVE_SINE;
  }

  if ((freq_hz * 3U) <= (SIGSEP_SAMPLE_RATE_HZ / 2U))
  {
    h3_amp = Correlate_Amplitude(samples, mean, freq_hz * 3U);
  }
  if ((freq_hz * 5U) <= (SIGSEP_SAMPLE_RATE_HZ / 2U))
  {
    h5_amp = Correlate_Amplitude(samples, mean, freq_hz * 5U);
  }

  if ((other_hz == (freq_hz * 3U)) && (h5_amp > (fundamental_amp * SIGSEP_TRI_H5_RATIO)))
  {
    return WAVE_TRIANGLE;
  }
  if ((other_hz != (freq_hz * 3U)) && (h3_amp > (fundamental_amp * SIGSEP_TRI_H3_RATIO)))
  {
    return WAVE_TRIANGLE;
  }

  return WAVE_SINE;
}

/*
 * 从输入帧流中识别两个最主要的频率分量。
 *
 * 初始识别会对多帧结果求平均，降低瞬态噪声对判断的影响。
 * 幅度最大的两个频点会分别作为输出通道 0 和通道 1。
 */
static uint8_t Analyze_Frame(const uint16_t *samples, SignalComponent out[2])
{
  float32_t mean;
  float32_t amp[SIGSEP_FREQ_COUNT];
  uint32_t phase[SIGSEP_FREQ_COUNT];
  uint32_t freq_hz;
  uint32_t best0 = 0U;
  uint32_t best1 = 1U;
  uint32_t t;
  uint32_t i;

  mean = Frame_Mean(samples);

  for (i = 0U; i < SIGSEP_FREQ_COUNT; i++)
  {
    freq_hz = SIGSEP_FREQ_MIN_HZ + (i * SIGSEP_FREQ_STEP_HZ);
    Measure_Component(samples, mean, freq_hz, &amp[i], &phase[i]);
    identify_amp_acc[i] += amp[i];
  }

  identify_frame_count++;
  if (identify_frame_count < SIGSEP_IDENTIFY_FRAMES)
  {
    return 0U;
  }

  for (i = 0U; i < SIGSEP_FREQ_COUNT; i++)
  {
    amp[i] = identify_amp_acc[i] / (float32_t)identify_frame_count;
    identify_amp_acc[i] = 0.0f;
  }
  identify_frame_count = 0U;

  if (amp[best1] > amp[best0])
  {
    best0 = 1U;
    best1 = 0U;
  }

  for (i = 2U; i < SIGSEP_FREQ_COUNT; i++)
  {
    if (amp[i] > amp[best0])
    {
      best1 = best0;
      best0 = i;
    }
    else if (amp[i] > amp[best1])
    {
      best1 = i;
    }
  }

  if (best0 > best1)
  {
    t = best0;
    best0 = best1;
    best1 = t;
  }

  out[0].freq_index = (uint8_t)best0;
  out[0].freq_hz = SIGSEP_FREQ_MIN_HZ + (best0 * SIGSEP_FREQ_STEP_HZ);
  out[0].amp_adc = amp[best0];
  out[0].amp_dac = out[0].amp_adc * SIGSEP_ADC_TO_DAC_SCALE;
  out[0].phase_q32 = phase[best0];

  out[1].freq_index = (uint8_t)best1;
  out[1].freq_hz = SIGSEP_FREQ_MIN_HZ + (best1 * SIGSEP_FREQ_STEP_HZ);
  out[1].amp_adc = amp[best1];
  out[1].amp_dac = out[1].amp_adc * SIGSEP_ADC_TO_DAC_SCALE;
  out[1].phase_q32 = phase[best1];

  for (i = 0U; i < 2U; i++)
  {
    if (out[i].amp_dac < SIGSEP_DEFAULT_DAC_AMP)
    {
      out[i].amp_dac = SIGSEP_DEFAULT_DAC_AMP;
    }
    if (out[i].amp_dac > SIGSEP_MAX_DAC_AMP)
    {
      out[i].amp_dac = SIGSEP_MAX_DAC_AMP;
    }
  }

  out[0].wave = Detect_WaveType(samples, mean, out[0].freq_hz, out[1].freq_hz, out[0].amp_adc);
  out[1].wave = Detect_WaveType(samples, mean, out[1].freq_hz, out[0].freq_hz, out[1].amp_adc);

  return 1U;
}

/*
 * 为某个已识别分量生成一段 DAC 采样数据。
 *
 * 正弦波由查找表生成，三角波由 Q32 相位所在象限分段生成。
 * 输出值以 DAC 中点为中心，并限制在 12 位 DAC 有效范围内。
 */
static void Build_DacSamples(uint16_t *dst, const SignalComponent *comp, uint32_t start_phase,
                             uint32_t phase_step, uint32_t len)
{
  uint32_t phase = start_phase;
  uint32_t q;
  uint32_t frac;
  int32_t y_q15;
  int32_t amp = (int32_t)(comp->amp_dac + 0.5f);
  int32_t code;
  uint32_t i;

  for (i = 0U; i < len; i++)
  {
    if (comp->wave == WAVE_TRIANGLE)
    {
      q = phase >> 30;
      frac = (phase & 0x3FFFFFFFUL) >> 15;

      if (q == 0U)
      {
        y_q15 = (int32_t)frac;
      }
      else if (q == 1U)
      {
        y_q15 = 32767 - (int32_t)frac;
      }
      else if (q == 2U)
      {
        y_q15 = -(int32_t)frac;
      }
      else
      {
        y_q15 = -32767 + (int32_t)frac;
      }
    }
    else
    {
      y_q15 = sine_lut[phase >> SIGSEP_SINE_LUT_SHIFT];
    }

    code = (int32_t)SIGSEP_DAC_MID + ((amp * y_q15) >> 15);
    if (code < 0)
    {
      code = 0;
    }
    if (code > (int32_t)SIGSEP_DAC_MAX)
    {
      code = (int32_t)SIGSEP_DAC_MAX;
    }

    dst[i] = (uint16_t)code;
    phase += phase_step;
  }
}

/* 将 Hz 频率转换为每个采样点对应的 Q32 相位步进。 */
static uint32_t PhaseStep_Q32(uint32_t freq_hz)
{
  return (uint32_t)(((uint64_t)freq_hz * 4294967296ULL) / SIGSEP_SAMPLE_RATE_HZ);
}

/* 将角度偏移转换到 Q32 相位域。 */
static uint32_t PhaseOffsetDeg_To_Q32(int32_t deg)
{
  return (uint32_t)(((int64_t)deg * 4294967296LL) / 360LL);
}

/* 将用户设置的相位偏移限制到允许范围，并按支持的步进量化。 */
static int32_t Normalize_PhaseOffsetDeg(int32_t deg)
{
  int32_t rem;

  if (deg < SIGSEP_PHASE_OFFSET_MIN_DEG)
  {
    deg = SIGSEP_PHASE_OFFSET_MIN_DEG;
  }
  if (deg > SIGSEP_PHASE_OFFSET_MAX_DEG)
  {
    deg = SIGSEP_PHASE_OFFSET_MAX_DEG;
  }

  rem = deg % SIGSEP_PHASE_OFFSET_STEP_DEG;
  deg -= rem;
  if (rem >= ((SIGSEP_PHASE_OFFSET_STEP_DEG + 1) / 2))
  {
    deg += SIGSEP_PHASE_OFFSET_STEP_DEG;
  }

  if (deg > SIGSEP_PHASE_OFFSET_MAX_DEG)
  {
    deg = SIGSEP_PHASE_OFFSET_MAX_DEG;
  }

  return deg;
}

/* 根据已识别分量和当前采样时间初始化一个 NCO。 */
static void Nco_Init(uint32_t ch, const SignalComponent *comp, uint64_t sample_start)
{
  nco_state[ch].nominal_step = PhaseStep_Q32(comp->freq_hz);
  nco_state[ch].step_corr = 0;
  nco_state[ch].integrator = 0;
  nco_state[ch].phase_ref = comp->phase_q32;
  nco_state[ch].sample_ref = sample_start;
  nco_state[ch].last_error = 0;
}

/* 获取某个输出通道当前相位步进，包含 PLL 修正量。 */
static uint32_t Nco_Step(uint32_t ch)
{
  return nco_state[ch].nominal_step + (uint32_t)nco_state[ch].step_corr;
}

/* 根据绝对采样点编号预测 NCO 相位。 */
static uint32_t Nco_PhaseAtSample(uint32_t ch, uint64_t sample)
{
  uint64_t delta = 0U;

  if (sample >= nco_state[ch].sample_ref)
  {
    delta = sample - nco_state[ch].sample_ref;
  }

  return nco_state[ch].phase_ref + (uint32_t)((uint64_t)Nco_Step(ch) * delta);
}

/*
 * 使用测得相位更新某个 NCO 的 PLL。
 *
 * 环路同时修正相位参考和频率步进。修正量会被限幅，避免异常帧把再生输出
 * 拉偏过多。
 */
static int32_t Nco_UpdateLock(uint32_t ch, uint32_t measured_phase, uint64_t frame_start_sample)
{
  uint32_t predicted_phase = Nco_PhaseAtSample(ch, frame_start_sample);
  int32_t phase_error = (int32_t)(measured_phase - predicted_phase);
  int64_t integrator = ((nco_state[ch].integrator *
                         (int64_t)SIGSEP_PLL_INTEGRATOR_LEAK_NUM) /
                        65536LL) +
                       (int64_t)phase_error;
  int64_t correction;
  int64_t integrator_limit;
  int32_t max_corr;

  max_corr = (int32_t)(nco_state[ch].nominal_step / SIGSEP_PLL_MAX_CORR_DIV);
  if (max_corr < 1)
  {
    max_corr = 1;
  }

  integrator_limit = (int64_t)max_corr *
                     (int64_t)SIGSEP_FRAME_LEN *
                     (int64_t)SIGSEP_PLL_STEP_KI_DIV;
  if (integrator_limit > SIGSEP_PLL_INTEGRATOR_LIMIT)
  {
    integrator_limit = SIGSEP_PLL_INTEGRATOR_LIMIT;
  }

  if (integrator > integrator_limit)
  {
    integrator = integrator_limit;
  }
  if (integrator < -integrator_limit)
  {
    integrator = -integrator_limit;
  }

  correction = ((int64_t)phase_error / (int64_t)(SIGSEP_FRAME_LEN * SIGSEP_PLL_STEP_KP_DIV)) +
               (integrator / (int64_t)(SIGSEP_FRAME_LEN * SIGSEP_PLL_STEP_KI_DIV));

  if (correction > (int64_t)max_corr)
  {
    correction = max_corr;
  }
  if (correction < -(int64_t)max_corr)
  {
    correction = -(int64_t)max_corr;
  }

  nco_state[ch].integrator = integrator;
  nco_state[ch].step_corr = (int32_t)correction;
  nco_state[ch].phase_ref = predicted_phase +
                            (uint32_t)(phase_error /
                                       (int32_t)(1UL << SIGSEP_PLL_PHASE_KP_SHIFT));
  nco_state[ch].sample_ref = frame_start_sample;
  nco_state[ch].last_error = phase_error;

  return phase_error;
}

/* 在同源但频率不同的两个分量之间缩放相位误差。 */
static int32_t Scale_PhaseErrorByFreq(int32_t phase_error, uint32_t dst_hz, uint32_t src_hz)
{
  int64_t scaled;

  if (src_hz == 0U)
  {
    return 0;
  }

  scaled = ((int64_t)phase_error * (int64_t)dst_hz) / (int64_t)src_hz;
  return (int32_t)((uint32_t)scaled);
}

/* 将主通道 PLL 步进修正量缩放到跟随通道的频率域。 */
static int32_t Scale_StepCorrection(uint32_t follower_ch, uint32_t master_ch)
{
  int64_t scaled;
  int32_t max_corr;

  if (nco_state[master_ch].nominal_step == 0U)
  {
    return 0;
  }

  scaled = ((int64_t)nco_state[master_ch].step_corr *
            (int64_t)nco_state[follower_ch].nominal_step) /
           (int64_t)nco_state[master_ch].nominal_step;

  max_corr = (int32_t)(nco_state[follower_ch].nominal_step / SIGSEP_PLL_MAX_CORR_DIV);
  if (scaled > (int64_t)max_corr)
  {
    scaled = max_corr;
  }
  if (scaled < -(int64_t)max_corr)
  {
    scaled = -(int64_t)max_corr;
  }

  return (int32_t)scaled;
}

/*
 * 让一个通道跟随主通道的同源时间修正量。
 *
 * 当两个分离分量预计来自同一个相干输入源时，这样可以避免两个 PLL
 * 独立锁定导致相对关系漂移。
 */
static void Nco_FollowCommonSource(uint32_t follower_ch, uint32_t master_ch,
                                   int32_t master_phase_error, uint64_t frame_start_sample)
{
  uint32_t predicted_phase = Nco_PhaseAtSample(follower_ch, frame_start_sample);
  int32_t follower_phase_error = Scale_PhaseErrorByFreq(master_phase_error,
                                                        active_comp[follower_ch].freq_hz,
                                                        active_comp[master_ch].freq_hz);

  nco_state[follower_ch].integrator = 0;
  nco_state[follower_ch].step_corr = Scale_StepCorrection(follower_ch, master_ch);
  nco_state[follower_ch].phase_ref = predicted_phase +
                                     (uint32_t)(follower_phase_error /
                                                (int32_t)(1UL << SIGSEP_PLL_PHASE_KP_SHIFT));
  nco_state[follower_ch].sample_ref = frame_start_sample;
  nco_state[follower_ch].last_error = follower_phase_error;
}

/* 为两个输出通道填充一个已经释放的 DAC 半缓冲区。 */
static void Fill_DacHalf(uint32_t half_index, uint64_t play_sample)
{
  uint32_t dst_offset = half_index * SIGSEP_DAC_HALF_LEN;
  int32_t phase_offset_deg = output_phase_offset_deg;
  uint32_t phase0 = Nco_PhaseAtSample(0U, play_sample) + PhaseOffsetDeg_To_Q32(SIGSEP_DAC1_PHASE_OFFSET_DEG);
  uint32_t phase1 = Nco_PhaseAtSample(1U, play_sample) +
                    PhaseOffsetDeg_To_Q32(SIGSEP_DAC2_PHASE_OFFSET_DEG + phase_offset_deg);

  Build_DacSamples(&dac1_dma_buf[dst_offset], &active_comp[0], phase0, Nco_Step(0U), SIGSEP_DAC_HALF_LEN);
  Build_DacSamples(&dac2_dma_buf[dst_offset], &active_comp[1], phase1, Nco_Step(1U), SIGSEP_DAC_HALF_LEN);
}

void SignalSeparation_Start(void)
{
  /* 先配置统一采样时钟，并预计算数学表和输出表。 */
  Timer2_SetSampleRate(SIGSEP_SAMPLE_RATE_HZ);
  CORDIC_PrepareTables();
  Fill_DacMidscale();

  /* 启动环形 DMA 采样前先校准 ADC1。 */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  /* 运行时强制 ADC 转换数据进入环形 DMA 模式。 */
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_DMNGT, ADC_CONVERSIONDATA_DMA_CIRCULAR);

  /* 使能 DMA 前，先把两个 DAC 通道设置到中点电平。 */
  if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, SIGSEP_DAC_MID) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, SIGSEP_DAC_MID) != HAL_OK)
  {
    Error_Handler();
  }

  /* 先启动两个 DAC DMA 流，确保 TIM2 启动时已经有有效输出缓冲区。 */
  if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)dac1_dma_buf, SIGSEP_DAC_FRAME_LEN, DAC_ALIGN_12B_R) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t *)dac2_dma_buf, SIGSEP_DAC_FRAME_LEN, DAC_ALIGN_12B_R) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, SIGSEP_ADC_DMA_LEN) != HAL_OK)
  {
    Error_Handler();
  }

  /* 启动 TIM2 后，ADC 采样和 DAC 播放进入同步时间轴。 */
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  Startup_Print();
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
}

void SignalSeparation_Task(void)
{
  int32_t offset;
  uint64_t ready_sample_start;
  uint64_t ready_play_sample[2];
  uint32_t ready_mask;
  SignalComponent result[2];
  float32_t mean;
  float32_t measured_amp[2];
  uint32_t measured_phase[2];
  float32_t target_amp_dac[2];
  uint8_t smooth_amp = 0U;
  uint32_t master_ch = (SIGSEP_PHASE_MASTER_CH == 0U) ? 0U : 1U;
  uint32_t follower_ch = master_ch ^ 1U;
  int32_t master_phase_error;

  /*
   * 短暂关闭中断，复制由 ISR 更新的标志位。
   * 后续耗时处理都在临界区外完成。
   */
  __disable_irq();
  offset = adc_ready_offset;
  adc_ready_offset = -1;
  ready_sample_start = adc_ready_sample_start;
  ready_mask = dac_ready_mask;
  ready_play_sample[0] = dac_ready_play_sample[0];
  ready_play_sample[1] = dac_ready_play_sample[1];
  __enable_irq();

  if (separation_identified != 0U)
  {
    /* 只有两个 DAC 通道都释放半区 0 后，才重新填充该半区。 */
    if ((ready_mask & 0x03U) == 0x03U)
    {
      __disable_irq();
      dac_ready_mask &= ~0x03U;
      __enable_irq();
      Fill_DacHalf(0U, ready_play_sample[0]);
    }
    /* 只有两个 DAC 通道都释放半区 1 后，才重新填充该半区。 */
    if ((ready_mask & 0x0CU) == 0x0CU)
    {
      __disable_irq();
      dac_ready_mask &= ~0x0CU;
      __enable_irq();
      Fill_DacHalf(1U, ready_play_sample[1]);
    }
  }

  if (offset >= 0)
  {
    if (separation_identified == 0U)
    {
      /*
       * 识别阶段：对多帧 ADC 结果求平均，选出最强的两个频点，
       * 初始化输出 NCO，然后允许 DAC 播放分离后的信号。
       */
      if (Analyze_Frame(&adc_dma_buf[offset], result) == 0U)
      {
        return;
      }
      active_comp[0] = result[0];
      active_comp[1] = result[1];
      Nco_Init(0U, &active_comp[0], ready_sample_start);
      Nco_Init(1U, &active_comp[1], ready_sample_start);
      Print_SeparationResult(active_comp);
      separation_identified = 1U;
    }
    else
    {
      /*
       * 跟踪阶段：重新测量当前分量，平滑幅度，并更新 PLL，
       * 使 DAC 再生相位跟随输入信号。
       */
      mean = Frame_Mean(&adc_dma_buf[offset]);
#if (SIGSEP_COMMON_SOURCE_LOCK != 0U)
      /* 两个分量相干时，只用一个主 PLL 驱动两个通道。 */
      Measure_Component(&adc_dma_buf[offset], mean, active_comp[master_ch].freq_hz,
                        &measured_amp[master_ch], &measured_phase[master_ch]);
      measured_amp[follower_ch] = Measure_AmplitudeOnly(&adc_dma_buf[offset], mean,
                                                        active_comp[follower_ch].freq_hz);
      measured_phase[follower_ch] = Nco_PhaseAtSample(follower_ch, ready_sample_start);
#else
      /* 独立模式：每个分离分量各自拥有一个相位锁定环路。 */
      Measure_Component(&adc_dma_buf[offset], mean, active_comp[0].freq_hz,
                        &measured_amp[0], &measured_phase[0]);
      Measure_Component(&adc_dma_buf[offset], mean, active_comp[1].freq_hz,
                        &measured_amp[1], &measured_phase[1]);
#endif
      active_comp[0].phase_q32 = measured_phase[0];
      active_comp[1].phase_q32 = measured_phase[1];
      /* 对幅度做低通更新，避免相邻帧之间输出突变。 */
      active_comp[0].amp_adc += (measured_amp[0] - active_comp[0].amp_adc) / (float32_t)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
      active_comp[1].amp_adc += (measured_amp[1] - active_comp[1].amp_adc) / (float32_t)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
      smooth_amp = 1U;
#if (SIGSEP_COMMON_SOURCE_LOCK != 0U)
      master_phase_error = Nco_UpdateLock(master_ch, measured_phase[master_ch], ready_sample_start);
      Nco_FollowCommonSource(follower_ch, master_ch, master_phase_error, ready_sample_start);
#else
      (void)Nco_UpdateLock(0U, measured_phase[0], ready_sample_start);
      (void)Nco_UpdateLock(1U, measured_phase[1], ready_sample_start);
#endif
    }

    /* 将测得的输入幅度换算为 DAC 幅度，并限制到安全范围。 */
    target_amp_dac[0] = active_comp[0].amp_adc * SIGSEP_ADC_TO_DAC_SCALE;
    target_amp_dac[1] = active_comp[1].amp_adc * SIGSEP_ADC_TO_DAC_SCALE;
    if (target_amp_dac[0] < SIGSEP_DEFAULT_DAC_AMP)
    {
      target_amp_dac[0] = SIGSEP_DEFAULT_DAC_AMP;
    }
    if (target_amp_dac[1] < SIGSEP_DEFAULT_DAC_AMP)
    {
      target_amp_dac[1] = SIGSEP_DEFAULT_DAC_AMP;
    }
    if (target_amp_dac[0] > SIGSEP_MAX_DAC_AMP)
    {
      target_amp_dac[0] = SIGSEP_MAX_DAC_AMP;
    }
    if (target_amp_dac[1] > SIGSEP_MAX_DAC_AMP)
    {
      target_amp_dac[1] = SIGSEP_MAX_DAC_AMP;
    }
    if (smooth_amp != 0U)
    {
      /* 识别完成后继续平滑 DAC 幅度，防止输出抖动或跳变。 */
      active_comp[0].amp_dac += (target_amp_dac[0] - active_comp[0].amp_dac) / (float32_t)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
      active_comp[1].amp_dac += (target_amp_dac[1] - active_comp[1].amp_dac) / (float32_t)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
    }
    else
    {
      active_comp[0].amp_dac = target_amp_dac[0];
      active_comp[1].amp_dac = target_amp_dac[1];
    }
  }
}

void SignalSeparation_RestartIdentify(void)
{
  uint32_t i;

  /* 停止使用旧分量状态，并丢弃尚未处理的 ISR 交接标志。 */
  __disable_irq();
  separation_identified = 0U;
  adc_ready_offset = -1;
  dac_ready_mask = 0U;
  __enable_irq();

  /* 下一次搜索开始前清空频率识别的幅度累加值。 */
  identify_frame_count = 0U;
  for (i = 0U; i < SIGSEP_FREQ_COUNT; i++)
  {
    identify_amp_acc[i] = 0.0f;
  }

  /* 在新的识别结果有效前，将 DAC 输出保持在中点电平。 */
  Fill_DacMidscale();
}

void SignalSeparation_SetPhaseOffsetDeg(int32_t deg)
{
  /* 保存已限幅、已量化的偏移量，便于 DAC 路径无锁读取并应用。 */
  output_phase_offset_deg = Normalize_PhaseOffsetDeg(deg);
}

int32_t SignalSeparation_GetPhaseOffsetDeg(void)
{
  return output_phase_offset_deg;
}

uint8_t SignalSeparation_GetFrequencies(uint32_t *freq0_hz, uint32_t *freq1_hz)
{
  /* 只有识别阶段完成后，频率结果才有效。 */
  if (separation_identified == 0U)
  {
    return 0U;
  }

  if (freq0_hz != NULL)
  {
    *freq0_hz = active_comp[0].freq_hz;
  }
  if (freq1_hz != NULL)
  {
    *freq1_hz = active_comp[1].freq_hz;
  }

  return 1U;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    /* ADC 环形缓冲区前半区已经包含一帧完整数据。 */
    adc_frame_count++;
    adc_sample_count += SIGSEP_FRAME_LEN;
    if (adc_ready_offset < 0)
    {
      adc_ready_offset = 0;
      adc_ready_sample_start = adc_sample_count - SIGSEP_FRAME_LEN;
    }
    else
    {
      adc_frame_overrun++;
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    /* ADC 环形缓冲区后半区已经包含一帧完整数据。 */
    adc_frame_count++;
    adc_sample_count += SIGSEP_FRAME_LEN;
    if (adc_ready_offset < 0)
    {
      adc_ready_offset = (int32_t)SIGSEP_FRAME_LEN;
      adc_ready_sample_start = adc_sample_count - SIGSEP_FRAME_LEN;
    }
    else
    {
      adc_frame_overrun++;
    }
  }
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  if (hdac->Instance == DAC1)
  {
    /* CH1 作为 DAC 采样时间主通道；CH2 使用同一个 TIM2 触发。 */
    dac_sample_count += SIGSEP_DAC_HALF_LEN;
    if (separation_identified == 0U)
    {
      return;
    }
    dac_ready_play_sample[0] = dac_sample_count + SIGSEP_DAC_HALF_LEN;
    if ((dac_ready_mask & 0x01U) != 0U)
    {
      dac_half_overrun++;
    }
    dac_ready_mask |= 0x01U;
  }
}

void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
  if ((hdac->Instance == DAC1) && (separation_identified != 0U))
  {
    /* CH2 只用于确认该半缓冲区已经不再被 DMA 读取。 */
    if ((dac_ready_mask & 0x02U) != 0U)
    {
      dac_half_overrun++;
    }
    dac_ready_mask |= 0x02U;
  }
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  if (hdac->Instance == DAC1)
  {
    /* CH1 作为 DAC 采样时间主通道；CH2 使用同一个 TIM2 触发。 */
    dac_sample_count += SIGSEP_DAC_HALF_LEN;
    if (separation_identified == 0U)
    {
      return;
    }
    dac_ready_play_sample[1] = dac_sample_count + SIGSEP_DAC_HALF_LEN;
    if ((dac_ready_mask & 0x04U) != 0U)
    {
      dac_half_overrun++;
    }
    dac_ready_mask |= 0x04U;
  }
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
  if ((hdac->Instance == DAC1) && (separation_identified != 0U))
  {
    /* CH2 只用于确认该半缓冲区已经不再被 DMA 读取。 */
    if ((dac_ready_mask & 0x08U) != 0U)
    {
      dac_half_overrun++;
    }
    dac_ready_mask |= 0x08U;
  }
}
