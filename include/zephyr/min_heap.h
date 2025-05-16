/*
 * Copyright (c) 2025 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_MIN_HEAP_H_
#define ZEPHYR_INCLUDE_MIN_HEAP_H_

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Node Structure to hold heap data
 */
struct heap_node {
    /** Pointer to the data held. */
    void *data;
    /** Priority value of the data. */
    uint32_t priority;
};

/**
 * @brief Represents a fixed-size min-heap data structure.
 */
struct min_heap {
    /** Holds the list of minimum heap nodes. */
    struct heap_node *nodes;
    /** Total Capacity of the minimum heap. */
    size_t capacity;
    /** The used size of the minimum heap. */
    size_t size;
    /** Lock */
    struct k_spinlock lock;
};

/**
 * @brief Initialize the minimum heap.
 *
 * Initializes the minimum heap with the required capacity.
 *
 * @param heap Pointer to heap structure.
 * @param buffer Pointer to heap buffer.
 * @param capacity Total Capacity of the minimum heap.
 *
 * @return No return.
 */
void min_heap_init(struct min_heap *heap, struct heap_node *buffer,
                   size_t capacity);

/**
 * @brief Restores the minimum heap property after insertion of New Node
 *
 * Arranges the nodes according to the priotity of nodes
 *
 * @param heap Pointer to heap structure.
 * @param index index of the insertion of node.
 *
 * @return No return.
 */
static void heapify_up(struct min_heap *heap, size_t index);

/**
 * @brief Insert to minimum heap.
 *
 * Inserts a node to the minimum heap and re-arranges the priority
 * if the elements in the buffer.
 *
 * @param heap Pointer to heap structure.
 * @param data Data to be stored in the buffer.
 * @param priority priority of the data.
 *
 * @return 0 if successful, -1 if fails to insert.
 */
int min_heap_insert(struct min_heap *heap, void *data, uint32_t priority);

/**
 * @brief Return the highest priority node in the heap
 *
 * Returns the top most element of the minimum heap.
 *
 * @param heap Pointer to the heap structure.
 *
 * @return NULL if the buffer is empty. Node Data if the list is non-empty.
 */
void *min_heap_peek_min(const struct min_heap *heap);

/**
 * @brief Restores the minimum heap property after deletion of node
 *
 * Re-arrange the Nodes according to the priority after a node gets
 * removed from the minimum heap.
 *
 * @param heap Pointer to the heap structure.
 * @param index Index of deleted node.
 *
 * @return No return.
 */
static void heapify_down(struct min_heap *heap, size_t index);

/**
 * @brief Remove and return the highest priority Node.
 *
 * Remove and return the top most Node in th minimum heap and re-arrange
 * the buffer according to the priority.
 *
 * @param heap Pointer to the heap structure.
 *
 * @return NULL if the buffer is empty. Node Data if the list is non-empty.
 */
void *min_heap_extract_min(struct min_heap *heap);

/**
 * @brief Check if the minimum heap is empty
 *
 * If the size of the minimum heap is zero, return empty.
 *
 * @param heap Pointer to the heap structure.
 *
 * @return true if buffer is non-empty, else false.
 */
bool min_heap_is_empty(const struct min_heap *heap);
#ifdef __cplusplus
}
#endif
