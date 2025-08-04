/***************************************************************************************************
 * @file CircularBuffer.h
 * @brief A simple Circular or Ring buffer implementation.
 * 
 * @version 1.0
 * @date    2024-12-07
 * @author  @dabecart
 * 
 * @license This project is licensed under the MIT License - see the LICENSE file for details.
***********************************************************************************************/

#ifndef CIRCULAR_BUFFER_h
#define CIRCULAR_BUFFER_h

#include <string.h>
#include <stdint.h>

template <typename T, int SIZE>
class CircularBuffer {
    public:
    /**
     * @brief Construct a new Circular Buffer object
     */
    CircularBuffer() {
        empty();
    }

    /**
     * @brief Destroy the Circular Buffer object
     */
    ~CircularBuffer() {}

    /**
     * @brief Empties a CircularBuffer. 
     */
    void empty() {
        head = 0;
        tail = 0;
        len  = 0;
        memset(data, 0, size*sizeof(T));
    }

    /**
     * @brief Pushes a single byte into a CircularBuffer. Advances the tail index.
     * 
     * @param item. Byte to be store into the buffer.
     * @return uint8_t 1 if the push was successful. 
     */
    uint8_t push(T item) {
        if(len >= size) return 0;

        lockRoutine();

        data[head] = item;
        head++;
        if(head >= size) head = 0;
        len++; 
        return 1;
    }

    /**
     * @brief Pushes N elements into a CircularBuffer. Advances the tail index.
     * 
     * @param items. Elements to be stored into the buffer. 
     * @param count. Number of elements to push.  
     * @return uint8_t 1 if the push was successful. 
     */
    uint8_t pushN(T* items, uint32_t count) {
        if(items == NULL) return 0;

        if((len + count) > size) return 0;
        
        lockRoutine();

        uint32_t ullNextHead = head + count;
        if(ullNextHead > size) {
            uint32_t ullHeadBytes = size - head;
            memcpy(data+head, items, ullHeadBytes*sizeof(T));
            memcpy(data, items + ullHeadBytes, (count - ullHeadBytes)*sizeof(T));
        }else {
            memcpy(data+head, items, count*sizeof(T));
        }

        head = ullNextHead % size;
        len += count; 
        return 1;
    }

    /**
     * @brief Reads a byte from a CircularBuffer. Advances the head index.
     * 
     * @param item. Where the popped byte will be stored. 
     * @return uint8_t 1 if the read item is valid. 
     */
    uint8_t pop(T* item) {
        if(len < 1) return 0;

        lockRoutine();

        if(item != NULL) *item = data[tail];
        
        tail++;
        if(tail >= size) tail = 0;
        len--;
        return 1;
    }

    /**
     * @brief Reads N elements from a CircularBuffer. Advances the head index.
     * 
     * @param count. How many elements want to be popped. 
     * @param items. Where the popped elements will be stored. If it's NULL the indices will still 
     * be updated but no result will be returned. 
     * @return uint8_t 1 if the read items are valid. 
     */
    uint8_t popN(uint32_t count, T* items) {
        if(len < count) return 0;
        if(count == 0) return 1;
        
        lockRoutine();

        uint32_t nextTail = tail + count;
        if(items != NULL) {
            if(nextTail > size) {
                uint32_t tailBytes = size-tail;
                memcpy(items, data+tail, tailBytes*sizeof(T));
                memcpy(items + tailBytes, data, (count - tailBytes)*sizeof(T));
            }else {
                memcpy(items, data + tail, count*sizeof(T));
            }
        }

        tail = nextTail % size;
        len -= count;
        return 1;
    }

    /**
     * @brief Reads a byte from a CircularBuffer. Does not advance the head index.
     * 
     * @param item. Where the read byte will be stored. 
     * @return uint8_t 1 if the read item is valid. 
     */
    uint8_t peek(T* item) {
        if((len < 1) || (item == NULL)) return 0;
        
        *item = data[tail];
        return 1;
    }

    /**
     * @brief Reads N elements from a CircularBuffer. Does not advance the head index.
     * 
     * @param count. How many elements want to be peeked. 
     * @param items. Where the peeked elements will be stored. 
     * @return uint8_t 1 if the read items are valid. 
     */
    uint8_t peekN(uint32_t count, T* items) {
        if(items == NULL) return 0;
        if(count == 0) return 1;

        if(len < count) return 0;
        
        uint32_t nextTail = tail + count;
        if(nextTail > size) {
            uint32_t tailBytes = size-tail;
            memcpy(items, data+tail, tailBytes*sizeof(T));
            memcpy(items + tailBytes, data, (count - tailBytes)*sizeof(T));
        }else {
            memcpy(items, data + tail, count*sizeof(T));
        }

        return 1;
    }

    /**
     * @brief Reads a elements from a CircularBuffer at position "index". Does not advance the head 
     * index.
     * 
     * @param pCB. Pointer to the CircularBuffer struct.
     * @param index. The index into the array to look at.
     * @param items. Where the peeked byte will be stored.
     * @return 1 if the read item is valid. 
     */
    uint8_t peekAt(uint32_t index, T* item) {
        if(item == NULL) return 0;

        if(len <= index) return 0;
        
        uint32_t nextTail = tail + index;
        if(nextTail > size) {
            uint32_t tailBytes = size - tail;
            *item = data[index - tailBytes];
        }else {
            *item = data[tail + index];
        }

        return 1;
    }

    /**
     * @brief The DMA functions automatically treats a buffer as a circular buffer. The callbacks 
     * return the new head of the buffer, so this function is used to update the head index 
     * accordingly.
     * 
     * @param newHeadIndex. The head index returned by the callback. 
     * @return uint8_t 1 if the update was OK. 
     */
    uint8_t updateIndices(uint32_t newHeadIndex) {
        uint32_t readBytes = 0;
        if(newHeadIndex >= head) {
            readBytes = newHeadIndex - head;
        }else {
            readBytes = newHeadIndex + size - head;
        }

        // Is data being overwritten without being processed? 
        if((readBytes + len) > size) {
            // Update the tail index too.
            tail += readBytes - len;
            tail %= size;
            len = size;
        }else {
            len += readBytes;
        }

        head = newHeadIndex;

        return 1;
    }
    
    private:
    /**
     * @brief Returns only when the buffer isn't locked.
     */
    void lockRoutine() {
        while(locked) {
            // Nothing to do but wait here.
        }
    }

    public:
    uint32_t    size = SIZE;  // Full size of the buffer.    
    uint32_t    len = 0;      // Number of elements to read (stored elements count).
    uint32_t    head = 0;     // Index to read from.
    uint32_t    tail = 0;     // Index to write to.
    uint8_t     locked = 0;   // When locked, no modifications can be done.
    T           data[SIZE];   // Data buffer.
};

#endif // CIRCULAR_BUFFER_h