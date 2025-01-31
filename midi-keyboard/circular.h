#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdbool.h>

#define BUFFER_SIZE 20

typedef struct {
    uint8_t buffer[BUFFER_SIZE];
    int head;
    int tail;
    int size;
} CircularBuffer;

// Initialize the circular buffer
void initBuffer(CircularBuffer *cb);

// Check if the buffer is full
bool isFull(CircularBuffer *cb);

// Check if the buffer is empty
bool isEmpty(CircularBuffer *cb);

// Add an element to the buffer
bool enqueue(CircularBuffer *cb, uint8_t item);

// Remove an element from the buffer
bool dequeue(CircularBuffer *cb, uint8_t *item);

#endif // CIRCULAR_BUFFER_H