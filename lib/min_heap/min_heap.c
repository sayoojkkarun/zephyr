/*
 * Copyright (c) 2025 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/sys/min_heap.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(min_heap);

/**
 * @brief Restore heap order by moving a node up the tree.
 *
 * Moves the node at the given index upward in the heap until the min-heap
 * property is restored.
 *
 * @param heap Pointer to the min-heap.
 * @param index Index of the node to heapify upwards.
 */
static void heapify_up(struct min_heap *heap, size_t index)
{
	while (index > 0) {
		size_t parent = (index - 1) / 2;
		void *curr = min_heap_get_element(heap, index);
		void *par = min_heap_get_element(heap, parent);

		if (heap->cmp(curr, par) >= 0) {
			break;
		}
		byteswp(curr, par, heap->elem_size);
		if (heap->on_move != NULL) {
			heap->on_move(par, parent, heap->move_ctx);
			heap->on_move(curr, index, heap->move_ctx);
		}
		index = parent;
	}
}

/**
 * @brief Restore heap order by moving a node down the tree.
 *
 * Moves the node at the specified index downward in the heap until the
 * min-heap property is restored.
 *
 * @param heap Pointer to the min-heap.
 * @param index Index of the node to heapify downward.
 */

static void heapify_down(struct min_heap *heap, size_t index)
{
	/* Terminate the loop naturally when the left child is out of bounds */
	for (size_t left = 2 * index + 1; left < heap->size; left = 2 * index + 1) {

		size_t right = left + 1;
		size_t smallest = index;
		void *elem_index = min_heap_get_element(heap, index);
		void *elem_left = min_heap_get_element(heap, left);
		void *elem_smallest = elem_index;

		if (heap->cmp(elem_left, elem_index) < 0) {
			smallest = left;
			elem_smallest = elem_left;
		}

		if (right < heap->size) {
			void *elem_right = min_heap_get_element(heap, right);

			if (heap->cmp(elem_right, elem_smallest) < 0) {
				smallest = right;
				elem_smallest = elem_right;
			}
		}

		if (smallest == index) {
			break;
		}

		byteswp(elem_index, elem_smallest, heap->elem_size);
		if (heap->on_move != NULL) {
			heap->on_move(elem_index, index, heap->move_ctx);
			heap->on_move(elem_smallest, smallest, heap->move_ctx);
		}
		index = smallest;
	}
}


void min_heap_init(struct min_heap *heap, void *storage, size_t cap,
		   size_t elem_size, min_heap_cmp_t cmp)
{
	heap->storage = storage;
	heap->capacity = cap;
	heap->elem_size = elem_size;
	heap->cmp = cmp;
	heap->size = 0;
	heap->on_move = NULL;
	heap->move_ctx = NULL;
}

void min_heap_init_ex(struct min_heap *heap, void *storage, size_t cap,
		size_t elem_size, min_heap_cmp_t cmp,
		min_heap_move_t on_move, void *move_ctx)
{
	min_heap_init(heap, storage, cap, elem_size, cmp);
	heap->on_move  = on_move;
	heap->move_ctx = move_ctx;
}

void *min_heap_peek(const struct min_heap *heap)
{
	if (heap->size == 0) {
		return NULL;
	}

	return min_heap_get_element(heap, 0);
}

int min_heap_push(struct min_heap *heap, const void *item)
{
	if (heap->size >= heap->capacity) {
		return -ENOMEM;
	}

	void *dest = min_heap_get_element(heap, heap->size);

	memcpy(dest, item, heap->elem_size);
	if (heap->on_move != NULL) {
		heap->on_move(dest, heap->size, heap->move_ctx);
	}
	heapify_up(heap, heap->size);
	heap->size++;

	return 0;
}

bool min_heap_remove(struct min_heap *heap, size_t id, void *out_buf)
{
	if (id >= heap->size) {
		return false;
	}

	void *removed = min_heap_get_element(heap, id);

	memcpy(out_buf, removed, heap->elem_size);
	heap->size--;
	if (id != heap->size) {
		void *last = min_heap_get_element(heap, heap->size);

		memcpy(removed, last, heap->elem_size);
		if (heap->on_move != NULL) {
			heap->on_move(removed, id, heap->move_ctx);
		}
		heapify_down(heap, id);
		heapify_up(heap, id);
	}

	return true;
}

bool min_heap_pop(struct min_heap *heap, void *out_buf)
{
	return min_heap_remove(heap, 0, out_buf);
}

void *min_heap_find(struct min_heap *heap, min_heap_eq_t eq,
		    const void *other, size_t *out_id)
{
	void *element;

	for (size_t i = 0; i < heap->size; ++i) {

		element = min_heap_get_element(heap, i);
		if (eq(element, other)) {
			if (out_id) {
				*out_id = i;
			}
			return element;
		}
	}

	return NULL;
}
