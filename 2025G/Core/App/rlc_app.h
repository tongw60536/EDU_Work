#ifndef RLC_APP_H
#define RLC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 初始化 RLC 应用：完成滤波模块、显示模块和应用状态机的初始设置。 */
void rlc_app_init(void);

/* 应用主循环任务：周期调用，用来检查按键/串口命令并驱动学习或实时滤波流程。 */
void rlc_app_task(void);

#ifdef __cplusplus
}
#endif

#endif /* RLC_APP_H */
