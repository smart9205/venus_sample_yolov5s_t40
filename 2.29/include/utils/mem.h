/*
 *         (C) COPYRIGHT Ingenic Limited
 *              ALL RIGHT RESERVED
 *
 * File        : mem.h
 * Authors     : klyu
 * Create Time : 2021-06-11 09:20:33 (CST)
 * Description :
 *
 */

#ifndef __MAGIK_INFERENCEKIT_VENUS_INCLUDE_UTILS_MEM_H__
#define __MAGIK_INFERENCEKIT_VENUS_INCLUDE_UTILS_MEM_H__
#include "core/type.h"

ALG_PACK_START
namespace magik {
namespace venus {

/*alloc some memory from nmem*/
VENUS_API void *nmem_malloc(unsigned int size);

/*alloc some memory from nmem, and aligning the pointer with align*/
VENUS_API void *nmem_memalign(unsigned int align, unsigned int size);

/*alloc some memory by  nmem_memalign or nmem_malloc*/
VENUS_API void *nmem_vaddr2paddr(void *vaddr);

/*free some memory alloced by nmem_malloc and nmem_memalign*/
VENUS_API void nmem_free(void *ptr);

VENUS_API void memcopy(void *dst, void *src, int n);

} // namespace venus
} // namespace magik
ALG_PACK_END
#endif /* __MAGIK_INFERENCEKIT_VENUS_INCLUDE_UTILS_MEM_H__ */
