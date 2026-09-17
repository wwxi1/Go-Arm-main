/**
 * @file    ring_buffer.h
 * @brief   Byte ring buffer, useful for UART/referee protocols.
 */
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t *buffer;
    uint16_t size;
    volatile uint16_t head;   /* write index */
    volatile uint16_t tail;   /* read index  */
    volatile uint16_t count;
} RingBuffer_t;

void     RingBuffer_Init(RingBuffer_t *rb, uint8_t *buffer, uint16_t size);
bool     RingBuffer_Push(RingBuffer_t *rb, uint8_t byte);
bool     RingBuffer_Pop(RingBuffer_t *rb, uint8_t *byte);
uint16_t RingBuffer_Available(const RingBuffer_t *rb);
bool     RingBuffer_IsEmpty(const RingBuffer_t *rb);
bool     RingBuffer_IsFull(const RingBuffer_t *rb);
void     RingBuffer_Flush(RingBuffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */
