#ifndef STM32_LOCAL_STDDEF_H
#define STM32_LOCAL_STDDEF_H

typedef unsigned int size_t;
typedef int ptrdiff_t;
#ifndef NULL
#define NULL ((void *)0)
#endif
#define offsetof(type, member) ((size_t)&(((type *)0)->member))

#endif