#include "rlc_filter.h"
#include "rlc_display.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "fmac.h"
#include "tim.h"

#include "arm_math.h"
#include <math.h>

#define SWEEP_TIM2_CLOCK_HZ        (275000000.0f)
#define SWEEP_SAMPLE_RATE_HZ       (2500000.0f)
#define SWEEP_SAMPLES              (1000U)
#define SWEEP_LOW_CYCLES           (10U)
#define SWEEP_MID_CYCLES           (20U)
#define SWEEP_HIGH_CYCLES          (50U)
#define SWEEP_TOP_CYCLES           (100U)
#define SWEEP_LOW_BAND_HZ          (5000U)
#define SWEEP_MID_BAND_HZ          (20000U)
#define SWEEP_TOP_BAND_HZ          (96000U)
#define SWEEP_SETTLE_RECORDS       (2U)
#define SWEEP_ADC_TOTAL_SAMPLES    (SWEEP_SAMPLES * (SWEEP_SETTLE_RECORDS + 1U))
#define SWEEP_START_HZ             (1000U)
#define SWEEP_END_HZ               (100000U)
#define SWEEP_STEP_HZ              (200U)
#define SWEEP_MAX_POINTS           (((SWEEP_END_HZ - SWEEP_START_HZ) / SWEEP_STEP_HZ) + 1U)
#define SWEEP_DAC_MID              (2048.0f)
#define SWEEP_DAC_AMP              (1240.0f)
#define SWEEP_ADC_FULL_SCALE       (65535.0f)
#define SWEEP_DAC_FULL_SCALE       (4095.0f)
#define SWEEP_ADC_TIMEOUT_MARGIN_MS (100U)
#define FIT_MIN_POINTS             (12U)
#define FIT_VALID_MAG_RATIO        (0.025f)
#define FIT_MODEL_LOWPASS          (0U)
#define FIT_MODEL_BANDPASS         (1U)
#define FIT_MODEL_HIGHPASS         (2U)
#define FIT_MODEL_BANDSTOP         (3U)
#define IIR_SAMPLE_RATE_HZ         (1500000U)
#define IIR_BLOCK_SAMPLES          (512U)
#define IIR_DMA_SAMPLES            (IIR_BLOCK_SAMPLES * 2U)
#define IIR_USE_FMAC               (0U)
#define IIR_DEBUG_ADC_TO_DAC       (0U)
#define IIR_OUTPUT_GAIN            (0.8f)
#define IIR_FMAC_COEFF_B_SIZE      (3U)
#define IIR_FMAC_COEFF_A_SIZE      (2U)
#define PI_F                       (3.14159265358979323846f)

#define ADC_DMA_ATTR __attribute__((aligned(32)))
#define DAC_DMA_ATTR __attribute__((aligned(32)))

static uint16_t adc_buf[SWEEP_ADC_TOTAL_SAMPLES] ADC_DMA_ATTR;
static uint16_t dac_wave[SWEEP_SAMPLES] DAC_DMA_ATTR;
static volatile uint8_t adc_done;
static volatile uint8_t adc_error;
static volatile uint32_t dac_rounds;
static float sweep_cycles_per_record = 0.0f;
static float fit_freq_hz[SWEEP_MAX_POINTS];
static float fit_mag[SWEEP_MAX_POINTS];
static float fit_phase_deg[SWEEP_MAX_POINTS];
static uint32_t fit_count;
static uint16_t iir_adc_buf[IIR_DMA_SAMPLES] ADC_DMA_ATTR;
static uint16_t iir_dac_buf[IIR_DMA_SAMPLES] DAC_DMA_ATTR;
static volatile uint8_t iir_running;
static volatile uint8_t iir_process_flags;
static volatile uint8_t iir_overrun;
static volatile uint32_t iir_half_irq_count;
static volatile uint32_t iir_full_irq_count;
static volatile uint32_t iir_dac_half_irq_count;
static volatile uint32_t iir_dac_full_irq_count;
static volatile uint32_t iir_dac_error_count;
static volatile uint32_t iir_fmac_error_count;
static uint32_t iir_processed_count;
static uint16_t iir_adc_min;
static uint16_t iir_adc_max;
static uint16_t iir_dac_min;
static uint16_t iir_dac_max;
static float fit_n2;
static float fit_n1;
static float fit_n0;
static float fit_a;
static float fit_b;
static uint8_t fit_system_valid;
static uint32_t fit_model = FIT_MODEL_BANDPASS;
static uint8_t fit_model_valid;
static float iir_b0;
static float iir_b1;
static float iir_b2;
static float iir_a1;
static float iir_a2;
static uint8_t iir_coeff_valid;
static float32_t iir_coeffs[5];
static float32_t iir_state[4];
static float32_t iir_float_buf[IIR_BLOCK_SAMPLES];
static arm_biquad_casd_df1_inst_f32 iir_biquad;
static int16_t iir_fmac_input[IIR_BLOCK_SAMPLES];
static int16_t iir_fmac_output[IIR_BLOCK_SAMPLES];
static int16_t iir_fmac_coeff_b[IIR_FMAC_COEFF_B_SIZE];
static int16_t iir_fmac_coeff_a[IIR_FMAC_COEFF_A_SIZE];
static int16_t iir_fmac_zero[3];
static uint8_t iir_fmac_ready;
static uint8_t iir_fmac_coeff_clipped;

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_dac1_ch1;

/* 执行完整扫频：逐个频点输出正弦、采集 ADC、计算幅相、送屏显示，最后拟合模型。 */
static void sweep_run_once(void);

/* 准备单个扫频点：停止外设，重配 ADC/DAC DMA 模式，并切回 ADC 一次性采样。 */
static uint8_t sweep_prepare_point(void);

/* 强制 ADC DMA 使用一次性采样：写 HAL 配置和寄存器，再读回确认配置生效。 */
static uint8_t sweep_force_adc_dma_oneshot(void);

/* 选择采样记录中的周期数：低频少周期、高频多周期，兼顾频率分辨率和采样稳定性。 */
static uint32_t sweep_select_cycles(uint32_t target_freq_hz);

/* 配置扫频定时器：根据目标频率和周期数计算采样率，设置 TIM2 自动重装值。 */
static float sweep_config_timer_for_freq(uint32_t target_freq_hz);

/* 生成 DAC 波形：按当前周期数填充一段余弦表，并限制到 12 位 DAC 输出范围。 */
static void sweep_make_dac_wave(void);

/* 计算参考幅度：对 DAC 输出波形做同频解调，并换算成等效 ADC 计数用于归一化。 */
static float sweep_reference_amp_adc_counts(void);

/* 分析采样数据：去除直流均值后做同频正交解调，得到该频点的幅值和相位。 */
static void sweep_analyze(float *mag, float *phase_deg);

/* 保存扫频点：把频率、幅值和相位写入拟合数组，供后续 RLC 模型搜索使用。 */
static void sweep_fit_store(float mag, float phase_deg, uint32_t freq_hz);

/* 拟合并输出 RLC 模型：打印低通/带通/高通/带阻候选误差，确定模型、参数和数字滤波系数。 */
static void sweep_fit_rlc_and_print(void);

/* 计算模型幅值基函数：在给定 f0 和 Q 下，返回指定 RLC 模型的单位增益幅频响应。 */
static float sweep_rlc_mag_base(uint32_t model, float freq_hz, float f0_hz, float q);

/* 计算模型相位：根据低通、带通、高通或带阻分子相位，减去二阶分母相位得到角度。 */
static float sweep_rlc_phase_deg(uint32_t model, float freq_hz, float f0_hz, float q);

/* 获取模型名称：把内部模型编号转换成屏幕和调试串口使用的文本。 */
static const char *sweep_model_text(uint32_t model);

/* 打印模型名称：通过显示模块的调试串口接口输出当前拟合模型文本。 */
static void sweep_uart_print_model(uint32_t model);

/* 相位展开：通过加减 360 度让当前相位尽量接近上一点，便于拟合延迟斜率。 */
static float sweep_unwrap_phase(float phase_deg, float last_phase_deg);

/* 启动实时 IIR：检查系数，清状态和缓冲区，配置 DMA/定时器/可选 FMAC，再启动 ADC 和 DAC。 */
static void iir_start_from_fit(void);

/* 配置实时采样定时器：按 IIR_SAMPLE_RATE_HZ 计算 TIM2 自动重装值并更新计数器。 */
static void iir_config_timer(void);

/* 生成 IIR 系数：用双线性变换把拟合得到的模拟二阶系统转换成数字双二阶系数。 */
static uint8_t iir_make_coefficients(void);

/* 处理待处理标志：从中断置位的标志中取出前半/后半缓冲请求，并调用块处理函数。 */
static void iir_process_pending(void);

/* 处理一个实时数据块：读取 ADC 半缓冲，执行 IIR/FMAC/直通调试路径，再写回 DAC 半缓冲。 */
static void iir_process_block(uint32_t offset);

/* 对 DAC 码值施加输出增益：以 2048 为中心缩放，并限制到 0..4095。 */
static uint16_t iir_apply_output_gain(uint16_t sample);

/* 浮点输出转 DAC：把 -1..1 附近的归一化滤波结果映射到 12 位 DAC 码值。 */
static uint16_t iir_float_to_dac(float value);

/* 浮点系数转 FMAC Q15：将系数限制到 Q1.15 可表示范围，并记录是否发生裁剪。 */
static int16_t iir_float_to_fmac_q15(float value);

/* 配置 FMAC：把当前 IIR 系数量化后写入 FMAC 配置，设置输入、系数、输出缓冲区。 */
static uint8_t iir_fmac_configure_from_coefficients(void);

/* 使用 FMAC 处理一块数据：把 ADC 样本转为有符号输入，追加到 FMAC，并轮询取回输出。 */
static uint8_t iir_fmac_process_block(uint32_t offset);

/* 正弦函数封装：集中调用 sinf，方便后续替换成更快的近似或查表实现。 */
static float sweep_sinf(float x);

/* 余弦函数封装：集中调用 cosf，生成扫频 DAC 波形和解调用参考信号。 */
static float sweep_cosf(float x);

/* 平方根近似：先把输入缩放到合适范围，再用牛顿迭代求根，减少对库函数依赖。 */
static float sweep_sqrtf(float x);

/* 反正切近似：处理正负号和大于 1 的输入，再用有理近似计算角度。 */
static float sweep_atanf(float x);

/* atan2 近似：根据 x/y 所在象限修正 sweep_atanf 的结果，得到完整相位角。 */
static float sweep_atan2f(float y, float x);

/* 初始化滤波模块：启动 ADC 单端校准；如果校准失败，进入工程统一的 Error_Handler。 */
void rlc_filter_init(void)
{
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
}

/* 开始学习：如果实时滤波正在运行，先停止 TIM2、ADC DMA、DAC DMA，再打开调试输出并执行扫频。 */
void rlc_filter_start_learning(void)
{
  if (iir_running != 0U)
  {
    HAL_TIM_Base_Stop(&htim2);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    iir_running = 0U;
  }

  rlc_display_set_debug_enabled(1U);
  sweep_run_once();
}

/* 开始实时滤波：先把已识别的滤波类型发到屏幕，再根据拟合系数启动 IIR；启动成功后关闭调试打印。 */
void rlc_filter_start_realtime(void)
{
  if (fit_model_valid != 0U)
  {
    rlc_display_send_filter_type(sweep_model_text(fit_model));
  }

  rlc_display_set_debug_enabled(1U);
  iir_start_from_fit();
  if (iir_running != 0U)
  {
    rlc_display_set_debug_enabled(0U);
  }
}

/* 停止实时滤波：依次停止定时器、ADC DMA 和 DAC DMA，清除处理标志，并重新允许调试输出。 */
void rlc_filter_stop_realtime(void)
{
  HAL_TIM_Base_Stop(&htim2);
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
  iir_running = 0U;
  iir_process_flags = 0U;
  rlc_display_set_debug_enabled(1U);
}

/* 周期任务：主循环调用它来处理由 ADC DMA 中断设置的待处理半缓冲标志。 */
void rlc_filter_task(void)
{
  iir_process_pending();
}

/* 返回运行状态：直接读取 iir_running，非零表示实时滤波链路已经启动。 */
uint8_t rlc_filter_is_running(void)
{
  return iir_running;
}

/* 配置扫频定时器：先按目标频率选择周期数，再反推采样率和 ARR，最后返回实际采样率。 */
static float sweep_config_timer_for_freq(uint32_t target_freq_hz)
{
  uint32_t cycles = sweep_select_cycles(target_freq_hz);
  float target_sample_rate = ((float)target_freq_hz * (float)SWEEP_SAMPLES) / (float)cycles;
  uint32_t arr = (uint32_t)((SWEEP_TIM2_CLOCK_HZ / target_sample_rate) + 0.5f);
  float actual_sample_rate;

  if (arr > 0U)
  {
    arr--;
  }
  if (arr < 1U)
  {
    arr = 1U;
  }

  HAL_TIM_Base_Stop(&htim2);
  __HAL_TIM_SET_PRESCALER(&htim2, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  htim2.Instance->EGR = TIM_EGR_UG;

  actual_sample_rate = SWEEP_TIM2_CLOCK_HZ / ((float)arr + 1.0f);
  return actual_sample_rate;
}

/* 选择扫频周期数：高频使用更多周期来稳定测量，低频使用较少周期避免记录时间过长。 */
static uint32_t sweep_select_cycles(uint32_t target_freq_hz)
{
  if (target_freq_hz >= SWEEP_TOP_BAND_HZ)
  {
    return SWEEP_TOP_CYCLES;
  }
  if (target_freq_hz <= SWEEP_LOW_BAND_HZ)
  {
    return SWEEP_LOW_CYCLES;
  }
  if (target_freq_hz <= SWEEP_MID_BAND_HZ)
  {
    return SWEEP_MID_CYCLES;
  }
  return SWEEP_HIGH_CYCLES;
}

/* 生成扫频激励：按当前 sweep_cycles_per_record 计算相位，生成带直流偏置的 DAC 余弦波。 */
static void sweep_make_dac_wave(void)
{
  for (uint32_t n = 0; n < SWEEP_SAMPLES; n++)
  {
    float angle = 2.0f * PI_F * (float)sweep_cycles_per_record * (float)n / (float)SWEEP_SAMPLES;
    float sample = SWEEP_DAC_MID + (SWEEP_DAC_AMP * sweep_cosf(angle));

    if (sample < 0.0f)
    {
      sample = 0.0f;
    }
    else if (sample > 4095.0f)
    {
      sample = 4095.0f;
    }

    dac_wave[n] = (uint16_t)(sample + 0.5f);
  }
}

/* 设置 ADC 一次性 DMA：同步修改 HAL 结构体和硬件寄存器，并检查 DMNGT/OVRMOD 是否写入成功。 */
static uint8_t sweep_force_adc_dma_oneshot(void)
{
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_DMNGT, ADC_CONVERSIONDATA_DMA_ONESHOT);
  MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_OVRMOD, ADC_OVR_DATA_OVERWRITTEN);

  if (READ_BIT(hadc1.Instance->CFGR, ADC_CFGR_DMNGT) != ADC_CONVERSIONDATA_DMA_ONESHOT)
  {
    return 0U;
  }
  if (READ_BIT(hadc1.Instance->CFGR, ADC_CFGR_OVRMOD) != ADC_OVR_DATA_OVERWRITTEN)
  {
    return 0U;
  }

  return 1U;
}

/* 准备扫频采样点：停止外设，ADC DMA 配成 NORMAL，DAC DMA 配成 CIRCULAR，然后绑定回 HAL 句柄。 */
static uint8_t sweep_prepare_point(void)
{
  HAL_TIM_Base_Stop(&htim2);
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

  iir_running = 0U;
  iir_process_flags = 0U;

  (void)HAL_DMA_DeInit(&hdma_adc1);
  hdma_adc1.Init.Mode = DMA_NORMAL;
  hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
  {
    return 0U;
  }
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  (void)HAL_DMA_DeInit(&hdma_dac1_ch1);
  hdma_dac1_ch1.Init.Mode = DMA_CIRCULAR;
  hdma_dac1_ch1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_dac1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  if (HAL_DMA_Init(&hdma_dac1_ch1) != HAL_OK)
  {
    return 0U;
  }
  __HAL_LINKDMA(&hdac1, DMA_Handle1, hdma_dac1_ch1);

  return sweep_force_adc_dma_oneshot();
}

/* 计算参考幅度：对 DAC 波形做余弦/正弦投影，得到激励基波幅度，并换算为 ADC 满量程计数。 */
static float sweep_reference_amp_adc_counts(void)
{
  float ref_re = 0.0f;
  float ref_im = 0.0f;

  for (uint32_t n = 0; n < SWEEP_SAMPLES; n++)
  {
    float x = (float)dac_wave[n] - SWEEP_DAC_MID;
    float angle = 2.0f * PI_F * (float)sweep_cycles_per_record * (float)n / (float)SWEEP_SAMPLES;
    ref_re += x * sweep_cosf(angle);
    ref_im -= x * sweep_sinf(angle);
  }

  return ((2.0f * sweep_sqrtf((ref_re * ref_re) + (ref_im * ref_im))) /
          (float)SWEEP_SAMPLES) * SWEEP_ADC_FULL_SCALE / SWEEP_DAC_FULL_SCALE;
}

/* 分析 ADC 记录：跳过前面的稳定段，对最后一段采样去均值并做正交解调，输出幅值和相位。 */
static void sweep_analyze(float *mag, float *phase_deg)
{
  uint32_t sum = 0U;
  float mean;
  float re = 0.0f;
  float im = 0.0f;
  float ref_amp_adc_counts = sweep_reference_amp_adc_counts();
  uint32_t offset = SWEEP_SAMPLES * SWEEP_SETTLE_RECORDS;

  for (uint32_t n = 0; n < SWEEP_SAMPLES; n++)
  {
    sum += (uint32_t)adc_buf[offset + n];
  }
  mean = (float)sum / (float)SWEEP_SAMPLES;

  for (uint32_t n = 0; n < SWEEP_SAMPLES; n++)
  {
    float y = (float)adc_buf[offset + n] - mean;
    float angle = 2.0f * PI_F * (float)sweep_cycles_per_record * (float)n / (float)SWEEP_SAMPLES;
    re += y * sweep_cosf(angle);
    im -= y * sweep_sinf(angle);
  }

  if (ref_amp_adc_counts <= 1.0e-6f)
  {
    *mag = 0.0f;
  }
  else
  {
    *mag = (2.0f * sweep_sqrtf((re * re) + (im * im))) / ((float)SWEEP_SAMPLES * ref_amp_adc_counts);
  }
  *phase_deg = sweep_atan2f(im, re) * 180.0f / PI_F;
}

/* 执行扫频主流程：从起始频率按步进扫描到终止频率，每个频点配置定时器、启动 ADC/DAC、等待完成、分析并保存结果。 */
static void sweep_run_once(void)
{
  uint32_t last_freq_hz = 0U;

  fit_count = 0U;
  fit_model_valid = 0U;
  rlc_display_spectrum_begin(SWEEP_MAX_POINTS);

  for (uint32_t target_freq_hz = SWEEP_START_HZ; target_freq_hz <= SWEEP_END_HZ; target_freq_hz += SWEEP_STEP_HZ)
  {
    float mag;
    float phase_deg;
    uint32_t cycles = sweep_select_cycles(target_freq_hz);
    float actual_sample_rate = sweep_config_timer_for_freq(target_freq_hz);
    uint32_t freq_hz = (uint32_t)(((actual_sample_rate * (float)cycles) / (float)SWEEP_SAMPLES) + 0.5f);
    uint32_t adc_timeout_ms = SWEEP_ADC_TIMEOUT_MARGIN_MS +
      (uint32_t)(((float)SWEEP_ADC_TOTAL_SAMPLES * 1000.0f) / actual_sample_rate);
    sweep_cycles_per_record = (float)cycles;

    if (freq_hz == last_freq_hz)
    {
      continue;
    }
    last_freq_hz = freq_hz;

    if (sweep_prepare_point() == 0U)
    {
      rlc_display_debug_puts("SWEEP_DMA_CONFIG_ERROR,");
      rlc_display_debug_print_uint(freq_hz);
      rlc_display_debug_puts("\r\n");
      continue;
    }

    adc_done = 0U;
    adc_error = 0U;
    dac_rounds = 0U;
    sweep_make_dac_wave();
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
      SCB_CleanDCache_by_Addr((uint32_t *)dac_wave, (int32_t)sizeof(dac_wave));
      SCB_CleanInvalidateDCache_by_Addr((uint32_t *)adc_buf, (int32_t)sizeof(adc_buf));
    }

    if (sweep_force_adc_dma_oneshot() == 0U)
    {
      rlc_display_debug_puts("ADC_CFG_ERROR,");
      rlc_display_debug_print_uint(freq_hz);
      rlc_display_debug_puts("\r\n");
      continue;
    }
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, SWEEP_ADC_TOTAL_SAMPLES) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)dac_wave, SWEEP_SAMPLES, DAC_ALIGN_12B_R) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
    {
      Error_Handler();
    }

    uint32_t start = HAL_GetTick();
    while (adc_done == 0U)
    {
      if (adc_error != 0U)
      {
        rlc_display_debug_puts("ADC_ERROR,");
        rlc_display_debug_print_uint(HAL_ADC_GetError(&hadc1));
        rlc_display_debug_puts(",");
        rlc_display_debug_print_uint(freq_hz);
        rlc_display_debug_puts("\r\n");
        break;
      }
      if ((HAL_GetTick() - start) > adc_timeout_ms)
      {
        rlc_display_debug_puts("ADC_TIMEOUT,");
        rlc_display_debug_print_uint(HAL_ADC_GetError(&hadc1));
        rlc_display_debug_puts(",");
        rlc_display_debug_print_uint(freq_hz);
        rlc_display_debug_puts("\r\n");
        break;
      }
    }

    HAL_ADC_Stop_DMA(&hadc1);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_TIM_Base_Stop(&htim2);

    if (adc_done != 0U)
    {
      if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
      {
        SCB_InvalidateDCache_by_Addr((uint32_t *)adc_buf, (int32_t)sizeof(adc_buf));
      }
      sweep_analyze(&mag, &phase_deg);
      sweep_fit_store(mag, phase_deg, freq_hz);
      rlc_display_send_spectrum_point((fit_count > 0U) ? (fit_count - 1U) : 0U, SWEEP_MAX_POINTS, mag, phase_deg);
    }

    HAL_Delay(2U);
  }

  rlc_display_debug_puts("SWEEP_DONE\r\n");
  sweep_fit_rlc_and_print();
}

/* 保存拟合样本：如果数组未满，就按当前 fit_count 写入频率、幅值、相位并递增计数。 */
static void sweep_fit_store(float mag, float phase_deg, uint32_t freq_hz)
{
  if (fit_count >= SWEEP_MAX_POINTS)
  {
    return;
  }

  fit_freq_hz[fit_count] = (float)freq_hz;
  fit_mag[fit_count] = mag;
  fit_phase_deg[fit_count] = phase_deg;
  fit_count++;
}

/* 计算 RLC 幅频基函数：根据模型类型选择低通、高通、带通或带阻分子，再除以二阶分母模值。 */
static float sweep_rlc_mag_base(uint32_t model, float freq_hz, float f0_hz, float q)
{
  float w = 2.0f * PI_F * freq_hz;
  float w0 = 2.0f * PI_F * f0_hz;
  float a = w0 / q;
  float b = w0 * w0;
  float real = (w0 * w0) - (w * w);
  float imag = a * w;
  float den = sweep_sqrtf((real * real) + (imag * imag));

  if (den <= 0.0f)
  {
    return 0.0f;
  }

  if (model == FIT_MODEL_LOWPASS)
  {
    return b / den;
  }
  if (model == FIT_MODEL_HIGHPASS)
  {
    return (w * w) / den;
  }
  if (model == FIT_MODEL_BANDSTOP)
  {
    return ((real >= 0.0f) ? real : -real) / den;
  }
  return imag / den;
}

/* 计算 RLC 相频响应：先确定分子相位，再减去分母 atan2 得到的相位，返回角度值。 */
static float sweep_rlc_phase_deg(uint32_t model, float freq_hz, float f0_hz, float q)
{
  float w = 2.0f * PI_F * freq_hz;
  float w0 = 2.0f * PI_F * f0_hz;
  float a = w0 / q;
  float real = (w0 * w0) - (w * w);
  float imag = a * w;
  float num_phase_deg = 90.0f;

  if (model == FIT_MODEL_LOWPASS)
  {
    num_phase_deg = 0.0f;
  }
  else if (model == FIT_MODEL_HIGHPASS)
  {
    num_phase_deg = 180.0f;
  }
  else if (model == FIT_MODEL_BANDSTOP)
  {
    num_phase_deg = (real >= 0.0f) ? 0.0f : 180.0f;
  }

  return num_phase_deg - (sweep_atan2f(imag, real) * 180.0f / PI_F);
}

/* 模型编号转文本：低通、带通、高通、带阻分别返回对应的英文显示名。 */
static const char *sweep_model_text(uint32_t model)
{
  if (model == FIT_MODEL_LOWPASS)
  {
    return "LOWPASS";
  }
  if (model == FIT_MODEL_HIGHPASS)
  {
    return "HIGHPASS";
  }
  if (model == FIT_MODEL_BANDSTOP)
  {
    return "BANDSTOP";
  }
  return "BANDPASS";
}

/* 输出模型名称：复用 sweep_model_text 得到字符串，再交给调试串口发送。 */
static void sweep_uart_print_model(uint32_t model)
{
  rlc_display_debug_puts(sweep_model_text(model));
}

/* 相位展开处理：如果相邻相位差超过 180 度，就加/减 360 度消除跳变。 */
static float sweep_unwrap_phase(float phase_deg, float last_phase_deg)
{
  while ((phase_deg - last_phase_deg) > 180.0f)
  {
    phase_deg -= 360.0f;
  }
  while ((phase_deg - last_phase_deg) < -180.0f)
  {
    phase_deg += 360.0f;
  }

  return phase_deg;
}

/* 拟合 RLC 并打印结果：低通/带通/高通筛掉太小幅值点，带阻保留陷波点；逐个打印候选误差后选择最小者。 */
static void sweep_fit_rlc_and_print(void)
{
  float peak_mag = 0.0f;
  float best_f0 = 0.0f;
  float best_q = 1.0f;
  float best_k = 1.0f;
  float best_err = 3.4e38f;
  uint32_t best_model = FIT_MODEL_BANDPASS;
  float valid_min_mag;

  if (fit_count < FIT_MIN_POINTS)
  {
    rlc_display_debug_puts("FIT_ERROR,not_enough_points\r\n");
    return;
  }

  for (uint32_t i = 0; i < fit_count; i++)
  {
    if (fit_mag[i] > peak_mag)
    {
      peak_mag = fit_mag[i];
    }
  }

  if (peak_mag <= 0.0f)
  {
    rlc_display_debug_puts("FIT_ERROR,bad_data\r\n");
    return;
  }

  best_f0 = fit_freq_hz[fit_count / 2U];
  valid_min_mag = peak_mag * FIT_VALID_MAG_RATIO;

  for (uint32_t model = FIT_MODEL_LOWPASS; model <= FIT_MODEL_BANDSTOP; model++)
  {
    float model_best_f0 = best_f0;
    float model_best_q = 1.0f;
    float model_best_k = peak_mag;
    float model_best_k_at_err = peak_mag;
    float model_best_err = 3.4e38f;
    uint32_t model_best_used = 0U;

    for (uint32_t pass = 0; pass < 3U; pass++)
    {
      float f_min;
      float f_max;
      float q_min;
      float q_max;

      if (pass == 0U)
      {
        f_min = (float)SWEEP_START_HZ;
        f_max = (float)SWEEP_END_HZ;
        q_min = 0.10f;
        q_max = 20.0f;
      }
      else
      {
        float f_span = model_best_f0 * ((pass == 1U) ? 0.35f : 0.12f);
        float q_span = model_best_q * ((pass == 1U) ? 0.55f : 0.20f);

        f_min = model_best_f0 - f_span;
        f_max = model_best_f0 + f_span;
        q_min = model_best_q - q_span;
        q_max = model_best_q + q_span;
        if (f_min < (float)SWEEP_START_HZ)
        {
          f_min = (float)SWEEP_START_HZ;
        }
        if (f_max > (float)SWEEP_END_HZ)
        {
          f_max = (float)SWEEP_END_HZ;
        }
        if (q_min < 0.08f)
        {
          q_min = 0.08f;
        }
      }

      for (uint32_t fi = 0; fi <= 24U; fi++)
      {
        float f0 = f_min + (((f_max - f_min) * (float)fi) / 24.0f);

        for (uint32_t qi = 0; qi <= 24U; qi++)
        {
          float q = q_min + (((q_max - q_min) * (float)qi) / 24.0f);
          float k_num = 0.0f;
          float k_den = 0.0f;
          float err = 0.0f;
          uint32_t used = 0U;
          uint32_t err_used = 0U;

          for (uint32_t i = 0; i < fit_count; i++)
          {
            float base;

            if ((model != FIT_MODEL_BANDSTOP) && (fit_mag[i] < valid_min_mag))
            {
              continue;
            }

            base = sweep_rlc_mag_base(model, fit_freq_hz[i], f0, q);
            if (base > 1.0e-9f)
            {
              k_num += fit_mag[i] * base;
              k_den += base * base;
              used++;
            }
          }

          if ((used < FIT_MIN_POINTS) || (k_den <= 0.0f))
          {
            continue;
          }

          model_best_k = k_num / k_den;

          for (uint32_t i = 0; i < fit_count; i++)
          {
            float base;
            float pred;
            float denom;
            float e;

            if ((model != FIT_MODEL_BANDSTOP) && (fit_mag[i] < valid_min_mag))
            {
              continue;
            }

            base = sweep_rlc_mag_base(model, fit_freq_hz[i], f0, q);
            pred = model_best_k * base;
            denom = fit_mag[i];
            if (denom < valid_min_mag)
            {
              denom = valid_min_mag;
            }
            e = (pred - fit_mag[i]) / denom;
            err += e * e;
            err_used++;
          }

          if (err_used < FIT_MIN_POINTS)
          {
            continue;
          }

          err /= (float)err_used;
          if (err < model_best_err)
          {
            model_best_err = err;
            model_best_f0 = f0;
            model_best_q = q;
            model_best_k_at_err = model_best_k;
            model_best_used = err_used;
          }
        }
      }
    }

    rlc_display_debug_puts("FIT_CANDIDATE,type,");
    sweep_uart_print_model(model);
    rlc_display_debug_puts(",K,");
    rlc_display_debug_print_fixed(model_best_k_at_err, 1000000U);
    rlc_display_debug_puts(",f0_Hz,");
    rlc_display_debug_print_fixed(model_best_f0, 100U);
    rlc_display_debug_puts(",Q,");
    rlc_display_debug_print_fixed(model_best_q, 10000U);
    rlc_display_debug_puts(",used,");
    rlc_display_debug_print_uint(model_best_used);
    rlc_display_debug_puts(",err,");
    rlc_display_debug_print_sci(model_best_err);
    rlc_display_debug_puts("\r\n");

    if (model_best_err < best_err)
    {
      best_err = model_best_err;
      best_model = model;
      best_f0 = model_best_f0;
      best_q = model_best_q;
      best_k = model_best_k_at_err;
    }
  }

  {
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_xx = 0.0f;
    float sum_xy = 0.0f;
    float last_phase = fit_phase_deg[0];
    float phase_offset_deg = 0.0f;
    float phase_slope_deg_per_hz = 0.0f;
    float w0 = 2.0f * PI_F * best_f0;
    float a = w0 / best_q;
    float b = w0 * w0;
    uint32_t used = 0U;

    fit_model = best_model;
    fit_model_valid = 1U;
    rlc_display_send_filter_type(sweep_model_text(best_model));

    for (uint32_t i = 0; i < fit_count; i++)
    {
      float unwrapped = sweep_unwrap_phase(fit_phase_deg[i], last_phase);
      float residual;

      last_phase = unwrapped;
      if (fit_mag[i] < valid_min_mag)
      {
        continue;
      }

      residual = unwrapped - sweep_rlc_phase_deg(best_model, fit_freq_hz[i], best_f0, best_q);
      sum_x += fit_freq_hz[i];
      sum_y += residual;
      sum_xx += fit_freq_hz[i] * fit_freq_hz[i];
      sum_xy += fit_freq_hz[i] * residual;
      used++;
    }

    if (used > 1U)
    {
      float den = ((float)used * sum_xx) - (sum_x * sum_x);
      if ((den > 0.0f) || (den < 0.0f))
      {
        phase_slope_deg_per_hz = (((float)used * sum_xy) - (sum_x * sum_y)) / den;
        phase_offset_deg = (sum_y - (phase_slope_deg_per_hz * sum_x)) / (float)used;
      }
    }

    rlc_display_debug_puts("FIT_SYSTEM,type,");
    sweep_uart_print_model(best_model);
    rlc_display_debug_puts(",K,");
    rlc_display_debug_print_fixed(best_k, 1000000U);
    rlc_display_debug_puts(",f0_Hz,");
    rlc_display_debug_print_fixed(best_f0, 100U);
    rlc_display_debug_puts(",Q,");
    rlc_display_debug_print_fixed(best_q, 10000U);
    rlc_display_debug_puts(",delay_us,");
    rlc_display_debug_print_fixed((-phase_slope_deg_per_hz / 360.0f) * 1000000.0f, 1000U);
    rlc_display_debug_puts(",phase0_deg,");
    rlc_display_debug_print_fixed(phase_offset_deg, 100U);
    rlc_display_debug_puts(",err,");
    rlc_display_debug_print_sci(best_err);
    rlc_display_debug_puts("\r\n");

    rlc_display_debug_puts("FIT_COEFF,Hs=(n2*s^2+n1*s+n0)/(s^2+a*s+b),n2,");
    if ((best_model == FIT_MODEL_HIGHPASS) || (best_model == FIT_MODEL_BANDSTOP))
    {
      fit_n2 = best_k;
      rlc_display_debug_print_sci(best_k);
    }
    else
    {
      fit_n2 = 0.0f;
      rlc_display_debug_print_sci(0.0f);
    }
    rlc_display_debug_puts(",n1,");
    if (best_model == FIT_MODEL_BANDPASS)
    {
      fit_n1 = best_k * a;
      rlc_display_debug_print_sci(best_k * a);
    }
    else
    {
      fit_n1 = 0.0f;
      rlc_display_debug_print_sci(0.0f);
    }
    rlc_display_debug_puts(",n0,");
    if ((best_model == FIT_MODEL_LOWPASS) || (best_model == FIT_MODEL_BANDSTOP))
    {
      fit_n0 = best_k * b;
      rlc_display_debug_print_sci(best_k * b);
    }
    else
    {
      fit_n0 = 0.0f;
      rlc_display_debug_print_sci(0.0f);
    }
    fit_a = a;
    fit_b = b;
    fit_system_valid = 1U;
    iir_coeff_valid = iir_make_coefficients();
    rlc_display_debug_puts(",a,");
    rlc_display_debug_print_sci(a);
    rlc_display_debug_puts(",b,");
    rlc_display_debug_print_sci(b);
    rlc_display_debug_puts("\r\n");

    if (iir_coeff_valid != 0U)
    {
      rlc_display_debug_puts("IIR_COEFF,b0,");
      rlc_display_debug_print_sci(iir_b0);
      rlc_display_debug_puts(",b1,");
      rlc_display_debug_print_sci(iir_b1);
      rlc_display_debug_puts(",b2,");
      rlc_display_debug_print_sci(iir_b2);
      rlc_display_debug_puts(",a1,");
      rlc_display_debug_print_sci(iir_a1);
      rlc_display_debug_puts(",a2,");
      rlc_display_debug_print_sci(iir_a2);
      rlc_display_debug_puts("\r\n");
    }
    else
    {
      rlc_display_debug_puts("IIR_COEFF_ERROR\r\n");
    }
  }
}

/* 配置实时滤波采样率：用固定 IIR_SAMPLE_RATE_HZ 计算 TIM2 的 ARR，并触发更新事件。 */
static void iir_config_timer(void)
{
  uint32_t arr = (uint32_t)((SWEEP_TIM2_CLOCK_HZ / (float)IIR_SAMPLE_RATE_HZ) + 0.5f);

  if (arr > 0U)
  {
    arr--;
  }
  if (arr < 1U)
  {
    arr = 1U;
  }

  HAL_TIM_Base_Stop(&htim2);
  __HAL_TIM_SET_PRESCALER(&htim2, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  htim2.Instance->EGR = TIM_EGR_UG;
}

/* 生成数字滤波系数：读取拟合出的 n2/n1/n0/a/b，使用双线性变换计算 b0/b1/b2/a1/a2 并初始化 CMSIS biquad。 */
static uint8_t iir_make_coefficients(void)
{
  float c = 2.0f * (float)IIR_SAMPLE_RATE_HZ;
  float c2 = c * c;
  float n2 = fit_n2;
  float n1 = fit_n1;
  float n0 = fit_n0;
  float a = fit_a;
  float b = fit_b;
  float d0;
  float d1;
  float d2;
  float m0;
  float m1;
  float m2;

  iir_coeff_valid = 0U;
  if (fit_system_valid == 0U)
  {
    return 0U;
  }

  d0 = c2 + (a * c) + b;
  d1 = (-2.0f * c2) + (2.0f * b);
  d2 = c2 - (a * c) + b;
  m0 = (n2 * c2) + (n1 * c) + n0;
  m1 = (-2.0f * n2 * c2) + (2.0f * n0);
  m2 = (n2 * c2) - (n1 * c) + n0;

  if ((d0 > -1.0e-9f) && (d0 < 1.0e-9f))
  {
    return 0U;
  }

  iir_b0 = m0 / d0;
  iir_b1 = m1 / d0;
  iir_b2 = m2 / d0;
  iir_a1 = d1 / d0;
  iir_a2 = d2 / d0;
  iir_coeffs[0] = iir_b0;
  iir_coeffs[1] = iir_b1;
  iir_coeffs[2] = iir_b2;
  iir_coeffs[3] = -iir_a1;
  iir_coeffs[4] = -iir_a2;
  arm_biquad_cascade_df1_init_f32(&iir_biquad, 1U, iir_coeffs, iir_state);
  iir_coeff_valid = 1U;

  return 1U;
}

/* CMSIS-DSP 初始化弱实现：当库中没有强定义时，设置级数、系数、状态指针并清零状态数组。 */
__WEAK void arm_biquad_cascade_df1_init_f32(arm_biquad_casd_df1_inst_f32 *S,
                                            uint8_t numStages,
                                            const float32_t *pCoeffs,
                                            float32_t *pState)
{
  S->numStages = numStages;
  S->pCoeffs = pCoeffs;
  S->pState = pState;

  for (uint32_t i = 0; i < (4U * (uint32_t)numStages); i++)
  {
    pState[i] = 0.0f;
  }
}

/* CMSIS-DSP 滤波弱实现：当库函数不可用时，用直接 I 型结构逐点计算 biquad 输出并更新状态。 */
__WEAK void arm_biquad_cascade_df1_f32(const arm_biquad_casd_df1_inst_f32 *S,
                                       const float32_t *pSrc,
                                       float32_t *pDst,
                                       uint32_t blockSize)
{
  const float32_t *pCoeffs = S->pCoeffs;
  float32_t *pState = S->pState;

  for (uint32_t stage = 0; stage < (uint32_t)S->numStages; stage++)
  {
    float32_t b0 = pCoeffs[(stage * 5U) + 0U];
    float32_t b1 = pCoeffs[(stage * 5U) + 1U];
    float32_t b2 = pCoeffs[(stage * 5U) + 2U];
    float32_t a1 = pCoeffs[(stage * 5U) + 3U];
    float32_t a2 = pCoeffs[(stage * 5U) + 4U];
    float32_t x1 = pState[(stage * 4U) + 0U];
    float32_t x2 = pState[(stage * 4U) + 1U];
    float32_t y1 = pState[(stage * 4U) + 2U];
    float32_t y2 = pState[(stage * 4U) + 3U];

    for (uint32_t n = 0; n < blockSize; n++)
    {
      float32_t x = (stage == 0U) ? pSrc[n] : pDst[n];
      float32_t y = (b0 * x) + (b1 * x1) + (b2 * x2) + (a1 * y1) + (a2 * y2);

      x2 = x1;
      x1 = x;
      y2 = y1;
      y1 = y;
      pDst[n] = y;
    }

    pState[(stage * 4U) + 0U] = x1;
    pState[(stage * 4U) + 1U] = x2;
    pState[(stage * 4U) + 2U] = y1;
    pState[(stage * 4U) + 3U] = y2;
  }
}

/* 启动实时 IIR 链路：检查系数有效性，复位状态和统计量，初始化 ADC/DAC 缓冲，配置 DMA、定时器和可选 FMAC，最后启动外设。 */
static void iir_start_from_fit(void)
{
  if (iir_coeff_valid == 0U)
  {
    rlc_display_debug_puts("IIR_NO_COEFF\r\n");
    return;
  }

  if (iir_running != 0U)
  {
    HAL_TIM_Base_Stop(&htim2);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
  }

  for (uint32_t i = 0; i < 4U; i++)
  {
    iir_state[i] = 0.0f;
  }
  arm_biquad_cascade_df1_init_f32(&iir_biquad, 1U, iir_coeffs, iir_state);
  iir_fmac_ready = 0U;
  iir_process_flags = 0U;
  iir_overrun = 0U;
  iir_half_irq_count = 0U;
  iir_full_irq_count = 0U;
  iir_dac_half_irq_count = 0U;
  iir_dac_full_irq_count = 0U;
  iir_dac_error_count = 0U;
  iir_fmac_error_count = 0U;
  iir_processed_count = 0U;
  iir_adc_min = 65535U;
  iir_adc_max = 0U;
  iir_dac_min = 4095U;
  iir_dac_max = 0U;

  for (uint32_t i = 0; i < IIR_DMA_SAMPLES; i++)
  {
    iir_adc_buf[i] = 32768U;
    iir_dac_buf[i] = 2048U;
  }
  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
  {
    SCB_CleanDCache_by_Addr((uint32_t *)iir_dac_buf, (int32_t)sizeof(iir_dac_buf));
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)iir_adc_buf, (int32_t)sizeof(iir_adc_buf));
  }

#if (IIR_USE_FMAC != 0U)
  if (iir_fmac_configure_from_coefficients() == 0U)
  {
    rlc_display_debug_puts("IIR_FAIL,FMAC_CONFIG\r\n");
    return;
  }
#endif

  if (hdma_adc1.Init.Mode != DMA_CIRCULAR)
  {
    (void)HAL_DMA_DeInit(&hdma_adc1);
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      rlc_display_debug_puts("IIR_FAIL,DMA_INIT\r\n");
      return;
    }
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
  }

  (void)HAL_DMA_DeInit(&hdma_dac1_ch1);
  hdma_dac1_ch1.Init.Mode = DMA_CIRCULAR;
  hdma_dac1_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_dac1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  if (HAL_DMA_Init(&hdma_dac1_ch1) != HAL_OK)
  {
    rlc_display_debug_puts("IIR_FAIL,DAC_DMA_INIT\r\n");
    return;
  }
  __HAL_LINKDMA(&hdac1, DMA_Handle1, hdma_dac1_ch1);

  iir_config_timer();
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_DMNGT, ADC_CONVERSIONDATA_DMA_CIRCULAR);
  MODIFY_REG(hadc1.Instance->CFGR, ADC_CFGR_OVRMOD, ADC_OVR_DATA_OVERWRITTEN);
  iir_running = 1U;

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)iir_adc_buf, IIR_DMA_SAMPLES) != HAL_OK)
  {
    iir_running = 0U;
    rlc_display_debug_puts("IIR_FAIL,ADC_START,err,");
    rlc_display_debug_print_uint(HAL_ADC_GetError(&hadc1));
    rlc_display_debug_puts("\r\n");
    return;
  }
  if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)iir_dac_buf, IIR_DMA_SAMPLES, DAC_ALIGN_12B_R) != HAL_OK)
  {
    iir_running = 0U;
    rlc_display_debug_puts("IIR_FAIL,DAC_START\r\n");
    return;
  }
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    iir_running = 0U;
    rlc_display_debug_puts("IIR_FAIL,TIM_START\r\n");
    return;
  }
}

/* 处理待滤波数据块：在临界区读取 flags，优先处理前半缓冲，再处理后半缓冲，避免中断和主循环抢同一标志。 */
static void iir_process_pending(void)
{
  uint8_t flags;

  __disable_irq();
  flags = iir_process_flags;
  if (flags == 0U)
  {
    __enable_irq();
    return;
  }

  if ((flags & 0x01U) != 0U)
  {
    iir_process_flags &= (uint8_t)~0x01U;
    __enable_irq();
    iir_process_block(0U);
    return;
  }

  if ((flags & 0x02U) != 0U)
  {
    iir_process_flags &= (uint8_t)~0x02U;
    __enable_irq();
    iir_process_block(IIR_BLOCK_SAMPLES);
    return;
  }

  __enable_irq();
}

/* 应用输出增益：以 DAC 中点 2048 为零点缩放波形幅度，随后裁剪到 12 位范围。 */
static uint16_t iir_apply_output_gain(uint16_t sample)
{
  float dac = 2048.0f + (((float)sample - 2048.0f) * IIR_OUTPUT_GAIN);

  if (dac < 0.0f)
  {
    dac = 0.0f;
  }
  else if (dac > 4095.0f)
  {
    dac = 4095.0f;
  }

  return (uint16_t)(dac + 0.5f);
}

/* 浮点样本转 DAC：把归一化浮点值乘以 2047 和输出增益，再加中点并裁剪。 */
static uint16_t iir_float_to_dac(float value)
{
  float dac = 2048.0f + (value * 2047.0f * IIR_OUTPUT_GAIN);

  if (dac < 0.0f)
  {
    dac = 0.0f;
  }
  else if (dac > 4095.0f)
  {
    dac = 4095.0f;
  }

  return (uint16_t)(dac + 0.5f);
}

/* FMAC 系数量化：把浮点数限制到 Q15 范围，按正负分别四舍五入，并标记是否发生裁剪。 */
static int16_t iir_float_to_fmac_q15(float value)
{
  if (value > 0.9999695f)
  {
    iir_fmac_coeff_clipped = 1U;
    value = 0.9999695f;
  }
  else if (value < -1.0f)
  {
    iir_fmac_coeff_clipped = 1U;
    value = -1.0f;
  }

  if (value >= 0.0f)
  {
    return (int16_t)((value * 32768.0f) + 0.5f);
  }
  return (int16_t)((value * 32768.0f) - 0.5f);
}

/* 配置硬件 FMAC：先量化 b/a 系数，重启 FMAC，再设置输入、系数和输出缓冲区以及 IIR 直接 I 型参数。 */
static uint8_t iir_fmac_configure_from_coefficients(void)
{
  FMAC_FilterConfigTypeDef config = {0};

  iir_fmac_coeff_clipped = 0U;
  iir_fmac_coeff_b[0] = iir_float_to_fmac_q15(iir_b0);
  iir_fmac_coeff_b[1] = iir_float_to_fmac_q15(iir_b1);
  iir_fmac_coeff_b[2] = iir_float_to_fmac_q15(iir_b2);
  iir_fmac_coeff_a[0] = iir_float_to_fmac_q15(-iir_a1);
  iir_fmac_coeff_a[1] = iir_float_to_fmac_q15(-iir_a2);
  if (iir_fmac_coeff_clipped != 0U)
  {
    return 0U;
  }

  iir_fmac_zero[0] = 0;
  iir_fmac_zero[1] = 0;
  iir_fmac_zero[2] = 0;

  (void)HAL_FMAC_FilterStop(&hfmac);
  if (HAL_FMAC_DeInit(&hfmac) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_FMAC_Init(&hfmac) != HAL_OK)
  {
    return 0U;
  }

  config.InputBaseAddress = 0U;
  config.InputBufferSize = 128U;
  config.InputThreshold = FMAC_THRESHOLD_1;
  config.CoeffBaseAddress = 128U;
  config.CoeffBufferSize = 8U;
  config.OutputBaseAddress = 136U;
  config.OutputBufferSize = 120U;
  config.OutputThreshold = FMAC_THRESHOLD_1;
  config.pCoeffB = iir_fmac_coeff_b;
  config.CoeffBSize = IIR_FMAC_COEFF_B_SIZE;
  config.pCoeffA = iir_fmac_coeff_a;
  config.CoeffASize = IIR_FMAC_COEFF_A_SIZE;
  config.InputAccess = FMAC_BUFFER_ACCESS_POLLING;
  config.OutputAccess = FMAC_BUFFER_ACCESS_POLLING;
  config.Clip = FMAC_CLIP_ENABLED;
  config.Filter = FMAC_FUNC_IIR_DIRECT_FORM_1;
  config.P = IIR_FMAC_COEFF_B_SIZE;
  config.Q = IIR_FMAC_COEFF_A_SIZE;
  config.R = 0U;

  if (HAL_FMAC_FilterConfig(&hfmac, &config) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_FMAC_FilterPreload(&hfmac, iir_fmac_zero, IIR_FMAC_COEFF_A_SIZE,
                             iir_fmac_zero, IIR_FMAC_COEFF_A_SIZE) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_FMAC_FilterStart(&hfmac, NULL, NULL) != HAL_OK)
  {
    return 0U;
  }

  iir_fmac_ready = 1U;
  return 1U;
}

/* FMAC 块处理：把 ADC 半缓冲转为有符号输入，配置输出缓冲，追加输入数据并轮询等待完整输出。 */
static uint8_t iir_fmac_process_block(uint32_t offset)
{
  uint16_t input_size = IIR_BLOCK_SAMPLES;
  uint16_t output_size = IIR_BLOCK_SAMPLES;

  if (iir_fmac_ready == 0U)
  {
    return 0U;
  }

  for (uint32_t i = 0; i < IIR_BLOCK_SAMPLES; i++)
  {
    iir_fmac_input[i] = (int16_t)((int32_t)iir_adc_buf[offset + i] - 32768);
  }

  if (HAL_FMAC_ConfigFilterOutputBuffer(&hfmac, iir_fmac_output, &output_size) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_FMAC_AppendFilterData(&hfmac, iir_fmac_input, &input_size) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_FMAC_PollFilterData(&hfmac, 10U) != HAL_OK)
  {
    return 0U;
  }
  if ((input_size != IIR_BLOCK_SAMPLES) || (output_size != IIR_BLOCK_SAMPLES))
  {
    return 0U;
  }

  return 1U;
}

/* 实时数据块处理：失效 ADC 缓存，转换为归一化浮点，执行调试直通、FMAC 或 CMSIS IIR 路径，写入 DAC 缓冲并清理缓存。 */
static void iir_process_block(uint32_t offset)
{
  uint16_t adc_min = 65535U;
  uint16_t adc_max = 0U;
  uint16_t dac_min = 4095U;
  uint16_t dac_max = 0U;

  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
  {
    SCB_InvalidateDCache_by_Addr((uint32_t *)&iir_adc_buf[offset],
                                 (int32_t)(IIR_BLOCK_SAMPLES * sizeof(iir_adc_buf[0])));
  }

  for (uint32_t i = 0; i < IIR_BLOCK_SAMPLES; i++)
  {
    uint16_t adc_sample = iir_adc_buf[offset + i];

    if (adc_sample < adc_min)
    {
      adc_min = adc_sample;
    }
    if (adc_sample > adc_max)
    {
      adc_max = adc_sample;
    }

    iir_float_buf[i] = ((float32_t)adc_sample - 32768.0f) / 32768.0f;
  }

#if (IIR_DEBUG_ADC_TO_DAC != 0U)
  for (uint32_t i = 0; i < IIR_BLOCK_SAMPLES; i++)
  {
    uint16_t dac_sample = (uint16_t)(iir_adc_buf[offset + i] >> 4);

    if (dac_sample > 4095U)
    {
      dac_sample = 4095U;
    }
    dac_sample = iir_apply_output_gain(dac_sample);
    if (dac_sample < dac_min)
    {
      dac_min = dac_sample;
    }
    if (dac_sample > dac_max)
    {
      dac_max = dac_sample;
    }
    iir_dac_buf[offset + i] = dac_sample;
  }
#elif (IIR_USE_FMAC != 0U)
  if (iir_fmac_process_block(offset) != 0U)
  {
    for (uint32_t i = 0; i < IIR_BLOCK_SAMPLES; i++)
    {
      int32_t dac_value = ((int32_t)iir_fmac_output[i] + 32768) >> 4;
      uint16_t dac_sample;

      if (dac_value < 0)
      {
        dac_sample = 0U;
      }
      else if (dac_value > 4095)
      {
        dac_sample = 4095U;
      }
      else
      {
        dac_sample = (uint16_t)dac_value;
      }
      dac_sample = iir_apply_output_gain(dac_sample);

      if (dac_sample < dac_min)
      {
        dac_min = dac_sample;
      }
      if (dac_sample > dac_max)
      {
        dac_max = dac_sample;
      }
      iir_dac_buf[offset + i] = dac_sample;
    }
  }
  else
  {
    iir_fmac_error_count++;
    arm_biquad_cascade_df1_f32(&iir_biquad, iir_float_buf, iir_float_buf, IIR_BLOCK_SAMPLES);

    for (uint32_t i = 0; i < IIR_BLOCK_SAMPLES; i++)
    {
      uint16_t dac_sample = iir_float_to_dac(iir_float_buf[i]);

      if (dac_sample < dac_min)
      {
        dac_min = dac_sample;
      }
      if (dac_sample > dac_max)
      {
        dac_max = dac_sample;
      }
      iir_dac_buf[offset + i] = dac_sample;
    }
  }
#else
  arm_biquad_cascade_df1_f32(&iir_biquad, iir_float_buf, iir_float_buf, IIR_BLOCK_SAMPLES);

  for (uint32_t i = 0; i < IIR_BLOCK_SAMPLES; i++)
  {
    uint16_t dac_sample = iir_float_to_dac(iir_float_buf[i]);

    if (dac_sample < dac_min)
    {
      dac_min = dac_sample;
    }
    if (dac_sample > dac_max)
    {
      dac_max = dac_sample;
    }
    iir_dac_buf[offset + i] = dac_sample;
  }
#endif

  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
  {
    SCB_CleanDCache_by_Addr((uint32_t *)&iir_dac_buf[offset],
                            (int32_t)(IIR_BLOCK_SAMPLES * sizeof(iir_dac_buf[0])));
  }

  if (adc_min < iir_adc_min)
  {
    iir_adc_min = adc_min;
  }
  if (adc_max > iir_adc_max)
  {
    iir_adc_max = adc_max;
  }
  if (dac_min < iir_dac_min)
  {
    iir_dac_min = dac_min;
  }
  if (dac_max > iir_dac_max)
  {
    iir_dac_max = dac_max;
  }
  iir_processed_count++;
}

/* ADC 全缓冲完成：实时模式下设置后半缓冲处理标志并检测过载；扫频模式下置 adc_done 表示采样结束。 */
void rlc_filter_adc_conv_cplt_callback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    if (iir_running != 0U)
    {
      iir_full_irq_count++;
      if ((iir_process_flags & 0x02U) != 0U)
      {
        iir_overrun = 1U;
      }
      iir_process_flags |= 0x02U;
    }
    else
    {
      adc_done = 1U;
    }
  }
}

/* ADC 半缓冲完成：实时模式下设置前半缓冲处理标志，如果上一块还没处理完则记录 overrun。 */
void rlc_filter_adc_conv_half_cplt_callback(ADC_HandleTypeDef *hadc)
{
  if ((hadc->Instance == ADC1) && (iir_running != 0U))
  {
    iir_half_irq_count++;
    if ((iir_process_flags & 0x01U) != 0U)
    {
      iir_overrun = 1U;
    }
    iir_process_flags |= 0x01U;
  }
}

/* ADC 错误处理：确认是 ADC1 后置位 adc_error，扫频等待循环会打印错误并跳过当前频点。 */
void rlc_filter_adc_error_callback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_error = 1U;
  }
}

/* DAC 全缓冲完成：记录 DAC 循环轮数；实时模式下额外累计全缓冲中断次数。 */
void rlc_filter_dac_conv_cplt_ch1_callback(DAC_HandleTypeDef *hdac)
{
  if (hdac->Instance == DAC1)
  {
    dac_rounds++;
    if (iir_running != 0U)
    {
      iir_dac_full_irq_count++;
    }
  }
}

/* DAC 半缓冲完成：实时模式下累计半缓冲中断次数，用于观察 DAC 输出节拍。 */
void rlc_filter_dac_conv_half_cplt_ch1_callback(DAC_HandleTypeDef *hdac)
{
  if ((hdac->Instance == DAC1) && (iir_running != 0U))
  {
    iir_dac_half_irq_count++;
  }
}

/* DAC 错误处理：确认 DAC1 后累计错误次数，并清除通道 1 DMA underrun 标志。 */
void rlc_filter_dac_error_ch1_callback(DAC_HandleTypeDef *hdac)
{
  if (hdac->Instance == DAC1)
  {
    iir_dac_error_count++;
    __HAL_DAC_CLEAR_FLAG(hdac, DAC_FLAG_DMAUDR1);
  }
}

/* 正弦封装：当前直接调用 sinf，保留封装层方便在无 FPU 或性能紧张时替换。 */
static float sweep_sinf(float x)
{
  return sinf(x);
}

/* 余弦封装：当前直接调用 cosf，扫频波形生成和解调参考信号都通过这里计算。 */
static float sweep_cosf(float x)
{
  return cosf(x);
}

/* 平方根近似：对输入做范围归一化后进行多次牛顿迭代，最后恢复缩放比例。 */
static float sweep_sqrtf(float x)
{
  float normalized;
  float scale;
  float y;

  if (x <= 0.0f)
  {
    return 0.0f;
  }

  normalized = x;
  scale = 1.0f;
  while (normalized > 4.0f)
  {
    normalized *= 0.25f;
    scale *= 2.0f;
  }
  while (normalized < 0.25f)
  {
    normalized *= 4.0f;
    scale *= 0.5f;
  }

  y = normalized;
  for (uint32_t i = 0; i < 6U; i++)
  {
    y = 0.5f * (y + (normalized / y));
  }

  return y * scale;
}

/* 反正切近似：先处理符号和倒数变换，再用 x/(1+0.280872*x*x) 近似 atan。 */
static float sweep_atanf(float x)
{
  float sign = 1.0f;

  if (x < 0.0f)
  {
    sign = -1.0f;
    x = -x;
  }

  if (x > 1.0f)
  {
    return sign * ((0.5f * PI_F) - sweep_atanf(1.0f / x));
  }

  return sign * (x / (1.0f + (0.280872f * x * x)));
}

/* atan2 近似：根据 x、y 的符号选择象限，把 sweep_atanf 的主值修正为完整相位。 */
static float sweep_atan2f(float y, float x)
{
  if (x > 0.0f)
  {
    return sweep_atanf(y / x);
  }
  if (x < 0.0f)
  {
    if (y >= 0.0f)
    {
      return sweep_atanf(y / x) + PI_F;
    }
    return sweep_atanf(y / x) - PI_F;
  }
  if (y > 0.0f)
  {
    return 0.5f * PI_F;
  }
  if (y < 0.0f)
  {
    return -0.5f * PI_F;
  }
  return 0.0f;
}
