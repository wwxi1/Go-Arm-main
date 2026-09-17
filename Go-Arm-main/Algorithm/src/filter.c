/**
 * @file    filter.c
 * @brief   First-order low-pass filter.
 */
#include "filter.h"
#include <math.h>

void LowPassFilter_Init(LowPassFilter_t *f, float sample_time_s, float cutoff_hz)
{
    f->sample_time_s = sample_time_s;
    f->cutoff_hz = cutoff_hz;
    f->output = 0.0f;
    f->initialized = 0;

    if (sample_time_s > 0.0f && cutoff_hz > 0.0f) {
        float rc = 1.0f / (2.0f * 3.14159265358979f * cutoff_hz);
        f->alpha = sample_time_s / (rc + sample_time_s);
    } else {
        f->alpha = 1.0f;
    }
}

float LowPassFilter_Update(LowPassFilter_t *f, float input)
{
    if (!f->initialized) {
        f->output = input;
        f->initialized = 1;
    } else {
        f->output += f->alpha * (input - f->output);
    }
    return f->output;
}

void LowPassFilter_Reset(LowPassFilter_t *f, float value)
{
    f->output = value;
    f->initialized = 1;
}
