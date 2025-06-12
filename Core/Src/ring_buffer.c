#include "ring_buffer.h"

/**
 * @brief Initializes the ring buffer variables to their initial values.
 * @param rb Pointer to the ring buffer structure.
 * @param buffer Pointer to the memory area used as the ring buffer.
 * @param capacity The maximum number of elements the ring buffer can hold.
*/
void ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t capacity) 
{
    rb->buffer = buffer;
    rb->head = 0;
    rb->tail = 0;
    rb->capacity = capacity;
    rb->is_full = false;
}

/**
 * @brief Writes a byte of data to the ring buffer, discards the oldest data if is_full.
 * @param rb Pointer to the ring buffer structure.
 * @param data The byte of data to write.
 * @return true if the write was successful.
 */
bool ring_buffer_write(ring_buffer_t *rb, uint8_t data)
{
    // Check if the buffer is full
    if (rb->is_full) {
        // If the buffer is full, we overwrite new data
        rb->tail = (rb->tail + 1) % rb->capacity; // Discard the oldest data
    }
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % rb->capacity;
    if (rb->head == rb->tail) {
        // If head catches up with tail, the buffer is full
        rb->is_full = true;
    } 
    return true;
}

/**
 * @brief Reads a byte of data from the ring buffer.
 * @param rb Pointer to the ring buffer structure.
 * @param data Pointer to where the read data will be stored.
 * @return true if the read was successful, false if the buffer is empty.
 */
bool ring_buffer_read(ring_buffer_t *rb, uint8_t *data)
{
    // Check if the buffer is empty
    if (rb->head == rb->tail && !rb->is_full) {
        return false; // Buffer is empty
    }
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->capacity;
    rb->is_full = false; // After reading, the buffer can't be full
    return true;
}

/**
 * @brief Returns the number of elements currently in the ring buffer.
 * @param rb Pointer to the ring buffer structure.
 * @return The number of elements in the buffer.
 */
uint16_t ring_buffer_count(ring_buffer_t *rb)
{
    if (rb->is_full) {
        return rb->capacity;
    }
    if (rb->head >= rb->tail) {
        return rb->head - rb->tail;
    }
    else {
        // If head is less than tail, it means we have wrapped around
        return rb->capacity - rb->tail + rb->head;
    }
}

/**
 * @brief Checks if the ring buffer is empty.
 * @param rb Pointer to the ring buffer structure.
 * @return true if the buffer is empty, false otherwise.
 */
bool ring_buffer_is_empty(ring_buffer_t *rb)
{
    return (rb->head == rb->tail && !rb->is_full);
}

/**
 * @brief Checks if the ring buffer is full.
 * @param rb Pointer to the ring buffer structure.
 * @return true if the buffer is full, false otherwise.
 */
bool ring_buffer_is_full(ring_buffer_t *rb)
{
    return (rb->is_full || (rb->head + 1) % rb->capacity == rb->tail);
}

/**
 * @brief Flushes the ring buffer, resetting its head and tail pointers.
 * @param rb Pointer to the ring buffer structure.
 */
void ring_buffer_flush(ring_buffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->is_full = false; // Reset the full flag
    // Optionally, you can clear the buffer memory
    for (uint16_t i = 0; i < rb->capacity; i++) {
        rb->buffer[i] = 0;
    }
}