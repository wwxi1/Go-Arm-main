/**
 * @file    filter.h
 * @brief   First-order low-pass filter.
 */
#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float alpha;      /* smoothing factor: 0..1, 0 = no filtering */
    float output;
    float sample_time_s;
    float cutoff_hz;
    int   initialized;
} LowPassFilter_t;

void  LowPassFilter_Init(LowPassFilter_t *f, float sample_time_s, float cutoff_hz);
float LowPassFilter_Update(LowPassFilter_t *f, float input);
void  LowPassFilter_Reset(LowPassFilter_t *f, float value);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */
