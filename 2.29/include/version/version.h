#ifndef __MAGIK_INFERENCEKIT_VENUS_INCLUDE_VERSION_VERSION_H__
#define __MAGIK_INFERENCEKIT_VENUS_INCLUDE_VERSION_VERSION_H__
#include "common/common_def.h"
ALG_PACK_START
namespace magik {
namespace venus {
struct VENUS_API VersionDesc {
    /*
     *bit30-31: reserved
     *bit28-29: AIE
     *bit26-27: LIB
     *bit24-25: SYS
     *bit16-23: info.major
     *bit12-15: reserved
     *bit8-11: info.minor
     *bit4-7: info.patch
     *bit0-3: info.build
     */
    unsigned int master;
    /*
     *0: no patch
     *>0: patch num(patch_list.txt/patch_define.h)
     */
    unsigned int patch0;
    /*internal alg*/
    unsigned int alg0;
    /*public alg*/
    unsigned int alg1;
    /*custom ops id*/
    unsigned int custom_id;
};
} // namespace venus
} // namespace magik
ALG_PACK_END
#endif //__MAGIK_INFERENCEKIT_VENUS_INCLUDE_VERSION_VERSION_H__
