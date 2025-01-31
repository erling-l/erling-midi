#include <stdint.h>
#include "hardware/irq.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
//#include "circular.h"
#define BUFFER_SIZE 20

typedef struct {
    uint8_t buffer[BUFFER_SIZE];
    int head;
    int tail;
    int size;
} CircularBuffer;

// Initialize the circular buffer
void initBuffer(CircularBuffer *cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->size = 0;
}

// Check if the buffer is full
bool isFull(CircularBuffer *cb) {
    return cb->size >= BUFFER_SIZE;
}

// Check if the buffer is empty
bool isEmpty(CircularBuffer *cb) {
    return cb->size <= 0;
}

// Add an element to the buffer
bool enqueue(CircularBuffer *cb, uint8_t item) {
        // Disable interrupts
    //__disable_irq();

    if (isFull(cb)) {
        // Enable interrupts
        //__enable_irq();
        return false; // Buffer is full
    }
    cb->buffer[cb->head] = item;
    cb->head = (cb->head + 1) % BUFFER_SIZE;
    cb->size++;
    // Enable interrupts
    //__enable_irq();
    return true;
}

// Remove an element from the buffer
bool dequeue(CircularBuffer *cb, uint8_t *item) {
    // Disable interrupts
    //__disable_irq();
    if (isEmpty(cb)) {
        // Enable interrupts
        //__enable_irq();
        return false; // Buffer is empty
    }
    *item = cb->buffer[cb->tail];
    cb->tail = (cb->tail + 1) % BUFFER_SIZE;
    cb->size--;
    // Enable interrupts
    //__enable_irq();
    return true;
}

// // Example usage
// int test_main() {
//     CircularBuffer cb;
//     initBuffer(&cb);

//     // Enqueue elements
//     for (int i = 0; i < 12; i++) {
//         if (enqueue(&cb, i)) {
//             printf("Enqueued %d\n", i);
//         } else {
//             printf("Buffer is full, cannot enqueue %d\n", i);
//         }
//     }

//     // Dequeue elements
//     int item;
//     while (dequeue(&cb, &item)) {
//         printf("Dequeued %d\n", item);
//     }

//     return 0;
// }