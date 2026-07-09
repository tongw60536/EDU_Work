#ifndef STM32_LOCAL_MATH_H
#define STM32_LOCAL_MATH_H

#define HUGE_VAL 1e999
#define HUGE_VALF 1e999f
#define HUGE_VALL 1e999L
#define INFINITY 1e999f
#define NAN (0.0f/0.0f)

double fabs(double x);
float fabsf(float x);
double sqrt(double x);
float sqrtf(float x);
double sin(double x);
float sinf(float x);
double cos(double x);
float cosf(float x);
double atan2(double y, double x);
float atan2f(float y, float x);
double log(double x);
float logf(float x);
double exp(double x);
float expf(float x);
double pow(double x, double y);
float powf(float x, float y);

#endif