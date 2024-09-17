/*
 *         (C) COPYRIGHT Ingenic Limited
 *              ALL RIGHT RESERVED
 *
 * File        : type.h
 * Authors     : klyu
 * Create Time : 2020-10-28 11:57:38 (CST)
 * Description :
 *
 */

#ifndef __MAGIK_INFERENCEKIT_VENUS_INCLUDE_CORE_TYPE_H__
#define __MAGIK_INFERENCEKIT_VENUS_INCLUDE_CORE_TYPE_H__
#include "common/common_type.h"

ALG_PACK_START
namespace magik {
namespace venus {

typedef DataFormat TensorFormat;
enum class UppType : int {
    NONE = -1,
    FILL_ZERO = 0,
    NEARST = 1,
    PIXEL_SHUFFLE = 2,
    BILINEAR = 3,
};
} // namespace venus
} // namespace magik
ALG_PACK_END
#endif /* __MAGIK_INFERENCEKIT_VENUS_INCLUDE_CORE_TYPE_H__ */
