#ifndef STM32_CPPTOOLS_FIX_H
#define STM32_CPPTOOLS_FIX_H

#include <stdint.h>
#include <stddef.h>

#ifndef USE_PWR_LDO_SUPPLY
#define USE_PWR_LDO_SUPPLY
#endif

#ifndef USE_HAL_DRIVER
#define USE_HAL_DRIVER
#endif

#ifndef STM32H723xx
#define STM32H723xx
#endif

#ifndef __GNUC__
#define __GNUC__ 10
#define __GNUC_MINOR__ 3
#define __GNUC_PATCHLEVEL__ 1
#endif

#ifndef __ARM_ARCH_7EM__
#define __ARM_ARCH_7EM__ 1
#endif

#ifndef __ARM_FP
#define __ARM_FP 14
#endif

#ifndef __VFP_FP__
#define __VFP_FP__ 1
#endif

#ifndef __MPU_PRESENT
#define __MPU_PRESENT 1U
#endif

#ifndef __FPU_PRESENT
#define __FPU_PRESENT 1U
#endif

#ifndef __NVIC_PRIO_BITS
#define __NVIC_PRIO_BITS 4U
#endif

#ifndef __Vendor_SysTickConfig
#define __Vendor_SysTickConfig 0U
#endif

#ifndef __ICACHE_PRESENT
#define __ICACHE_PRESENT 1U
#endif

#ifndef __DCACHE_PRESENT
#define __DCACHE_PRESENT 1U
#endif

#endif