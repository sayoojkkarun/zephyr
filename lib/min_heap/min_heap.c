/*
 * Copyright (c) 2025 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/min_heap.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mim_heap);

void min_heap_init(struct min_heap *heap, struct heap_node *buffer,
                   size_t capacity)
{
    heap->nodes = buffer;
    heap->capacity = capacity;
    heap->size = 0;

    return;
}

static void heapify_up(struct min_heap *heap, size_t index)
{
    while (index > 0) {
        size_t parent = (index - 1) / 2;

        if (heap->nodes[index].priority >= heap->nodes[parent].priority) {
            break;
        }

        struct heap_node temp = heap->nodes[index];
        heap->nodes[index] = heap->nodes[parent];
        heap->nodes[parent] = temp;

        index = parent;
    }

    return;
}

int min_heap_insert(struct min_heap *heap, void *data, uint32_t priority)
{
    if (heap->size >= heap->capacity) {
        LOG_DBG("");
        return -1;
    }

    heap->nodes[heap->size].data = data;
    heap->nodes[heap->size].priority = priority;
    heapify_up(heap, heap->size);
    heap->size++;

    return 0;
}

void *min_heap_peek_min(const struct min_heap *heap)
{
    if (heap->size == 0) {
        return NULL;
    }

    return heap->nodes[0].data;
}

static void heapify_down(struct min_heap *heap, size_t index)
{
    size_t smallest, left, right;

    while (1) {
        left = 2 * index + 1;
        right = 2 * index + 2;
        smallest = index;

        if (left < heap->size &&
            heap->nodes[left].priority < heap->nodes[smallest].priority) {
            smallest = left;
        }

        if (right < heap->size &&
            heap->nodes[right].priority < heap->nodes[smallest].priority) {
            smallest = right;
        }

        if (smallest == index) {
            break;
        }

        struct heap_node temp = heap->nodes[index];
        heap->nodes[index] = heap->nodes[smallest];
        heap->nodes[smallest] = temp;

        index = smallest;
    }
}

void *min_heap_extract_min(struct min_heap *heap)
{
    if (heap->size == 0) {
        return NULL;
    }

    void *min_data = heap->nodes[0].data;
    heap->nodes[0] = heap->nodes[heap->size - 1];
    heap->size--;
    heapify_down(heap, 0);

    return min_data;
}

bool min_heap_is_empty(const struct min_heap *heap)
{
    return heap->size == 0;
}
