/*
 *         (C) COPYRIGHT Ingenic Limited
 *              ALL RIGHT RESERVED
 *
 * File        : common_type.h
 * Authors     : lzwang
 * Create Time : 2021-10-14 18:14:03 (CST)
 * Description :
 *
 */

#ifndef __MAGIK_INFERENCEKIT_VENUS_INCLUDE_COMMON_COMMON_TYPE_H__
#define __MAGIK_INFERENCEKIT_VENUS_INCLUDE_COMMON_COMMON_TYPE_H__
#include "common_def.h"

ALG_PACK_START
namespace magik {
namespace venus {
enum class ShareMemoryMode : int VENUS_API {
    DEFAULT = 0,           /*SHARE_ONE_NETWORK*/
    ALL_SEPARABLE_MEM = 1, /*NO_SHARE, DYNAMIC MANAGE, FOR INTERNAL*/
    SMART_REUSE_MEM = 2,   /*SMART REUSE MEMORY, FOR INTERNAL*/
};

enum class ChannelLayout : int VENUS_API {
    NONE = -1,
    NV12 = 0,
    BGRA = 1,
    RGBA = 2,
    ARGB = 3,
    RGB = 4,
    GRAY = 5,
    FP = 6
};

enum class TransformType : int VENUS_API {
    NONE = -1,
    NV12_NV12 = 0,
};

enum class DataType : int VENUS_API {
    NONE = -1, /*is not supported*/
    AUTO = 0,
    FP32 = 1,
    UINT32 = 2,
    INT32 = 3,
    UINT16 = 4,
    INT16 = 5,
    UINT8 = 6,
    INT8 = 7,
    UINT6B = 8,  /*is not supported*/
    UINT6BP = 9, /*is not supported*/
    UINT4B = 10,
    UINT2B = 11,
    UINT10 = 12,
    INT10 = 13,
    UINT12 = 14,
    INT12 = 15,
    UINT14 = 16, /*is not supported*/
    INT14 = 17,  /*is not supported*/
    BOOL = 18,
};

enum class DataFormat : int VENUS_API {
    NONE = -1,
    /*
     * NHWC OR NDHWC32
     */
    AUTO = 0,
    /*
     * 1. image format.
     * (1). BGR0, eg: shape=[1, 3, 6, 4]:
     * +------------------------+
     * |BGR0BGR0BGR0BGR0BGR0BGR0|
     * |BGR0BGR0BGR0BGR0BGR0BGR0|
     * |BGR0BGR0BGR0BGR0BGR0BGR0|
     * +------------------------+
     * (2). RGB0, eg: shape=[1, 3, 6, 4]:
     * +------------------------+
     * |RGB0RGB0RGB0RGB0RGB0RGB0|
     * |RGB0RGB0RGB0RGB0RGB0RGB0|
     * |RGB0BGR0BGR0RGB0RGB0RGB0|
     * +------------------------+
     * (3). GRAY, eg: shape=[1, 3, 24, 1]:
     * +------------------------+
     * |YYYYYYYYYYYYYYYYYYYYYYYY|
     * |YYYYYYYYYYYYYYYYYYYYYYYY|
     * |YYYYYYYYYYYYYYYYYYYYYYYY|
     * +------------------------+
     * 2.Feature layout
     * [N_BATCH, HEIGHT, WIDTH, CHN]
     */
    NHWC = 1,
    /*
     * Feature layout
     * [N_BATCH, D_C32, HEIGHT, WIDTH, CHN_32]
     */
    NDHWC32 = 2,
    /*
     * Weight layout
     * [KERNEL_H, KERNEL_W, I_CHN, O_CHN]
     */
    HWIO = 3,
    /*
     * Weight layout
     * [N_OFP, M_IFP, KERNEL_H, KERNEL_W, S_BIT2, OFP, IFP]
     */
    NMHWSOIB2 = 4,
    /*
     * image format for nv12.
     * eg: shape=[1, 4, 24, 1]:
     * +------------------------+
     * |YYYYYYYYYYYYYYYYYYYYYYYY|
     * |YYYYYYYYYYYYYYYYYYYYYYYY|
     * |YYYYYYYYYYYYYYYYYYYYYYYY|
     * |YYYYYYYYYYYYYYYYYYYYYYYY|
     * +------------------------+
     * |UVUVUVUVUVUVUVUVUVUVUVUV|
     * |UVUVUVUVUVUVUVUVUVUVUVUV|
     * +------------------------+
     */
    NV12 = 5,
    /*
     * Weight layout
     * [O_CHN, KERNEL_H, KERNEL_W, I_CHN]
     */
    OHWI = 6,
    /*
     * Feature layout
     * [N_BATCH, CHN, HEIGHT, WIDTH]
     */
    NCHW = 7
};

enum class Device : int VENUS_API {
    /*Unsupported*/
    NONE = -1,
    /*nmem, alloc memory by call nmem_malloc,nmem_memalign, mudata*/
    CPU = 1,
    /*RAM of NPU, users disabled*/
    AIE = 2,
    /*external memory, must be contiguous memory block and aligned with 64 bytes*/
    EXTERNAL = 3
};

struct VENUS_API ConvParam {
    int kernel_w = 1;
    int kernel_h = 1;
    int stride_w = 1;
    int stride_h = 1;
    int pad_top = 0;
    int pad_bottom = 0;
    int pad_left = 0;
    int pad_right = 0;
    int dialation_w = 1;
    int dialation_h = 1;
};

} // namespace venus
} // namespace magik
ALG_PACK_END
#endif /* __MAGIK_INFERENCEKIT_VENUS_INCLUDE_COMMON_COMMON_TYPE_H__ */
