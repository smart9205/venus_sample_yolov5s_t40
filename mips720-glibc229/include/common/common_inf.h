/*
 *         (C) COPYRIGHT Ingenic Limited
 *              ALL RIGHT RESERVED
 *
 * File        : common_inf.h
 * Authors     : lzwang
 * Create Time : 2022-05-13 18:24:03 (CST)
 * Description :
 *
 */

#ifndef __MAGIK_INFERENCEKIT_VENUS_INCLUDE_COMMON_COMMON_INF_H__
#define __MAGIK_INFERENCEKIT_VENUS_INCLUDE_COMMON_COMMON_INF_H__
#include "common/common_type.h"
ALG_PACK_START
namespace magik {
namespace venus {

VENUS_API int venus_init(int size = 0); // size need 0x1000 alignment
VENUS_API int venus_deinit(void);
VENUS_API int venus_check();

} // namespace venus
} // namespace magik
ALG_PACK_END
#endif /* __MAGIK_INFERENCEKIT_VENUS_INCLUDE_COMMON_COMMON_INF_H__ */
