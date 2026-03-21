/*
 * Copyright (c) 2025 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SYS_MIN_HEAP_H_
#define ZEPHYR_INCLUDE_SYS_MIN_HEAP_H_

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief min_heap
 * @defgroup min_heap_apis Min-Heap service
 * @ingroup datastructure_apis
 * @{
 */

/**
 * @brief Comparator function type for min-heap ordering.
 *
 * This function compares two heap nodes to establish their relative order.
 * It must be implemented by the user and provided at min-heap
 * initialization.
 *
 * @param a First user defined data pointer for comparison.
 * @param b Second user defined data pointer for comparison.
 *
 * @return Negative value if @p a is less than @p b,
 *         positive value if @p a is greater than @p b,
 *         zero if they are equal.
 */
typedef int (*min_heap_cmp_t)(const void *a,
			       const void *b);

/**
 * @brief Equality function for finding a node in the heap.
 *
 * @param node Pointer to a user defined data.
 * @param other Pointer to a user-defined key or structure to compare with.
 *
 * @return true if the node matches the search criteria, false otherwise.
 */
typedef bool (*min_heap_eq_t)(const void *node,
			       const void *other);

/**
 * @brief position update callback for min-heap elements.
 *
 * Called when an element is placed at a new index in the heap storage,
 * both on initial insertion and after every swap performed by heapify
 * functions.
 *
 * @param elem Pointer to the element's current storage slot.
 * @param index Index of the slot where the element now resides.
 * @param user_data Opaque pointer supplied at heap init.
 */
typedef void (*min_heap_move_t)(void *elem, size_t index, void *user_data);

/**
 * @brief min-heap data structure with user-provided comparator.
 */
struct min_heap {
	/** Raw pointer to contiguous memory for elements */
	void *storage;
	/** Maximum number of elements */
	size_t capacity;
	/** Size of each element*/
	size_t elem_size;
	/** Current elements count */
	size_t size;
	/** Comparator function */
	min_heap_cmp_t cmp;
	/**
	 * Optional callback invoked when an element moves to a new slot.
	 * Set to NULL if position tracking is not required.
	 */
	min_heap_move_t on_move;
	/** Opaque context pointer forwareded to every on_move call. */
	void *move_ctx;
};

/**
 * @brief Define a min-heap instance.
 *
 * @param name Base name for the heap instance.
 * @param cap Capacity (number of elements).
 * @param elem_sz Size in bytes of each element.
 * @param align Required alignment of each element.
 * @param cmp_func Comparator function used by the heap
 */
#define MIN_HEAP_DEFINE(name, cap, elem_sz, align, cmp_func)                                       \
	static uint8_t name##_storage[(cap) * (elem_sz)] __aligned(align);                         \
	struct min_heap name = {.storage = name##_storage,                                         \
				.capacity = (cap),                                                 \
				.elem_size = (elem_sz),                                            \
				.size = 0,                                                         \
				.cmp = (cmp_func),                                                 \
				.on_move = NULL,                                                   \
				.mov_ctx = NULL}

/**
 * @brief Define a statically allocated and aligned min-heap instance.
 *
 * @param name Base name for the heap instance.
 * @param cap Capacity (number of elements).
 * @param elem_sz Size in bytes of each element.
 * @param align Required alignment of each element.
 * @param cmp_func Comparator function used by the heap
 */
#define MIN_HEAP_DEFINE_STATIC(name, cap, elem_sz, align, cmp_func) \
	static uint8_t name##_storage[(cap) * (elem_sz)] __aligned(align); \
	static struct min_heap name = { \
			.storage = name##_storage, \
			.capacity = (cap), \
			.elem_size = (elem_sz), \
			.size = 0, \
			.cmp = (cmp_func), \
			.on_move = NULL, \
			.move_ctx = NULL \
	}

/**
 * @brief Initialize a min-heap instance at runtime.
 *
 * Sets up the internal structure of a min heap using a user-provided
 * memory block, capacity, and comparator function. This function must
 * be called before using the heap if not statically defined.
 * The on_move callback is set to NULL, no position tracking.
 *
 * @param heap Pointer to the min-heap structure.
 * @param storage Pointer to memory block for storing elements.
 * @param cap Maximum number of elements the heap can store.
 * @param elem_size Size in bytes of each element.
 * @param cmp Comparator function used to order the heap elements.
 *
 * @note All arguments must be valid. This function does not allocate memory.
 *
 */
void min_heap_init(struct min_heap *heap, void *storage, size_t cap,
		   size_t elem_size, min_heap_cmp_t cmp);

/**
 * @brief Initialize a min-heap with position update callback.
 *
 * Same as min_heap_init() but also registers @p on_move so that
 * callers embedding a heap index inside their element can keep
 * track that index in sync as elements are repositioned.
 *
 * @param heap Pointer to the min-heap structure.
 * @param storage Pointer to memory block for storing elements.
 * @param cap Maximum number of elements the heap can store.
 * @param elem_size Size in bytes of each element.
 * @param cmp Comparator function used to order the heap elements.
 * @param on_move Callback invoked whenever an element moves to a new slot,
 *		   or NULL if not needed.
 * @param move_ctx Opaque pointer forwarded to every on_move call.
 */
void min_heap_init_ex(struct min_heap *heap, void *storage, size_t cap,
		size_t elem_size, min_heap_cmp_t cmp,
		min_heap_move_t on_move, void *move_ctx);

/**
 * @brief Push an element into the min-heap.
 *
 * Adds a new element to the min-heap and restores the heap order by moving it
 * upward as necessary. Insert operation will fail if the min-heap
 * has reached full capacity.
 *
 * @param heap Pointer to the min-heap.
 * @param item Pointer to the item to insert.
 *
 * @return 0 on Success, -ENOMEM if the heap is full.
 */
int min_heap_push(struct min_heap *heap, const void *item);

/**
 * @brief Peek at the top element of the min-heap.
 *
 * The function will not remove the element from the min-heap.
 *
 * @param heap Pointer to the min-heap.
 *
 * @return Pointer to the top priority element, or NULL if the heap is empty.
 */
void *min_heap_peek(const struct min_heap *heap);

/**
 * @brief Remove a specific element from the min-heap.
 *
 * Removes the specified node from the min-heap based on the ID it stores
 * internally. The min-heap is rebalanced after removal to ensure
 * proper ordering.
 * The caller gains ownership of the returned element and is responsible for
 * any further management of its memory or reuse.
 *
 * @param heap Pointer to the min-heap.
 * @param id element ID to be removed.
 * @param out_buf User-provided buffer where the removed element will be copied.
 *
 * @return true in success, false otherwise.
 */
bool min_heap_remove(struct min_heap *heap, size_t id, void *out_buf);

/**
 * @brief Check if the min heap is empty.
 *
 * This function checks whether the heap contains any elements.
 *
 * @param heap Pointer to the min heap.
 *
 * @return true if heap is empty, false otherwise.
 */
static inline bool min_heap_is_empty(struct min_heap *heap)
{
	__ASSERT_NO_MSG(heap != NULL);

	return (heap->size == 0);
}

/**
 * @brief Remove and return the highest priority element in the heap.
 *
 * The caller gains ownership of the returned element and is responsible for
 * any further management of its memory or reuse. The min-heap is rebalanced
 * after removal to ensure proper ordering.
 *
 * @param heap Pointer to heap.
 * @param out_buf User-provided buffer where the removed element will be copied.
 *
 * @return true in success, false otherwise.
 */
bool min_heap_pop(struct min_heap *heap, void *out_buf);

/**
 * @brief Search for a node in the heap matching a condition.
 *
 * @param heap Pointer to the heap structure.
 * @param eq Function used to compare each node with the search target.
 * @param other Pointer to the data used for comparison in the eq function.
 * @param out_id Optional output parameter to store the index of the found node.
 *
 * @return Pointer to the first matching element, or NULL if not found.
 */
void *min_heap_find(struct min_heap *heap, min_heap_eq_t eq,
				     const void *other, size_t *out_id);

/**
 * @brief Get a pointer to the element at the specified index.
 *
 * @param heap Pointer to the min-heap.
 * @param index Index of the element to retrieve.
 *
 * @return Pointer to the element at the given index.
 */
static inline void *min_heap_get_element(const struct min_heap *heap,
					  size_t index)
{
	__ASSERT_NO_MSG(heap != NULL);

	return (void *)((uintptr_t)heap->storage + index * heap->elem_size);
}

/**
 * @brief Iterate over each node in the heap.
 *
 * @param heap Pointer to the heap.
 * @param node_var The loop variable used to reference each node.
 *
 * Example:
 * ```
 * void *node;
 * MIN_HEAP_FOREACH(&heap, node) {
 *	printk("Value: %d\n", node->value);
 * }
 * ```
 */
#define MIN_HEAP_FOREACH(heap, node_var)                                                           \
	for (size_t _i = 0;                                                                        \
	     _i < (heap)->size && (((node_var) = min_heap_get_element((heap), _i)) || true); ++_i)

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SYS_MIN_HEAP_H_ */
