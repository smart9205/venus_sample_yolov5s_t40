/*
 *         (C) COPYRIGHT Ingenic Limited
 *              ALL RIGHT RESERVED
 *
 * File        : venus.h
 * Authors     : klyu
 * Create Time : 2020-10-27 10:48:19 (CST)
 * Description :
 *
 */

#ifndef __MAGIK_INFERENCEKIT_VENUS_INCLUDE_VENUS_H__
#define __MAGIK_INFERENCEKIT_VENUS_INCLUDE_VENUS_H__
#include "common/common_inf.h"
#include "core/data.h"
#include "core/tensor.h"
#include "core/type.h"
#include "utils/all.h"
#include "version/version.h"
#include <memory>
#include <vector>

ALG_PACK_START
namespace magik {
namespace venus {
class VENUS_API BaseNet {
public:
    BaseNet();
    virtual ~BaseNet() = 0;
    /*memory_model:
     * 0:model path
     * 1:memory address
     *   allocate memory by malloc function, copy model from memory to nmem
     *   in load_model, free memory after load_model
     * 2:model handle
     * 3:nmem address
     *   set parameter "force_align" true in ModelSerial,
     *   and generate model file
     *   allocate memory by nmem_memalign function, address alignment to 64
     *   use nmem directly, nmem_free after destory BaseNet;
     *   Advantages: no need to copy model from mempry to nmem
     *   Disadvantages: size of nmem maybe larger than memory to satisfy
     *                  requirements of nmem
     * 4:EXTERNAL memory address(rmem)
     *   set parameter "force_align" true in ModelSerial,
     *   and generate model file
     *   address alignment to 64, addr_desc must be configured
     *   use memory directly, free memory after destory BaseNet;
     *
     * 0/2/3/4 are recommended
     */
    virtual int load_model(const void *model_path, int memory_model = 0, int start_off = 0,
                           AddressDesc *addr_desc = nullptr);
    virtual int get_forward_memory_size(size_t &memory_size);
    /*init all memory*/
    virtual int init();
    /*free all memory*/
    virtual int deinit();
    /*free all memory except for input tensors*/
    virtual int free_forward_memory();
    /*free memory of input tensors*/
    virtual int free_inputs_memory();
    /*set internal memory management status, when smem_mode=ALL_SEPARABLE_MEM or SMART_REUSE_MEM
     *status=true: memory of input Tensors maybe free by BaseNet, be careful, data pointer should be
     *checked before read/write status=false: memory of input Tensors is managed by user
     */
    virtual void set_internal_mm_status(bool status);
    /*get internal memory management status*/
    virtual bool get_internal_mm_status();
    virtual void set_profiler_per_frame(bool status = false);
    virtual std::unique_ptr<Tensor> get_input(int index);
    virtual std::unique_ptr<Tensor> get_input_by_name(std::string &name);
    virtual std::vector<std::string> get_input_names();
    virtual std::unique_ptr<Tensor> get_output(int index);
    virtual std::unique_ptr<Tensor> get_output_by_name(std::string &name);
    virtual std::vector<std::string> get_output_names();
    /*get output names of given step*/
    virtual std::vector<std::string> get_output_names_step(int step);
    /*get color channel layout of input weight from model if set*/
    virtual ChannelLayout get_input_channel_layout(std::string &name);
    /*set color channel layout of input image for run network from NV12 data_fmt
     *pelease set same channel layout with model
     */
    virtual void set_input_channel_layout(std::string name, ChannelLayout layout);

    /*do inference, get all outputs*/
    virtual int run();
    /*get number of steps*/
    virtual int steps();
    /*do inference, get outputs for each step*/
    virtual int run_step();
};

/*
 * create inference handle.
 * input_data_fmt: NHWC or NV12
 */
VENUS_API std::unique_ptr<BaseNet> net_create(TensorFormat input_data_fmt = TensorFormat::NHWC,
                                              ShareMemoryMode smem_mode = ShareMemoryMode::DEFAULT);
/*delete net ptr*/
VENUS_API int net_delete(std::unique_ptr<BaseNet> &network_ptr);
VENUS_API int venus_lock();
VENUS_API int venus_unlock();
VENUS_API void venus_dump(bool enable_data);
/*removed in future, replace by VersionDesc venus_version_desc()*/
VENUS_API uint32_t venus_get_version_info();
VENUS_API VersionDesc venus_version_desc();
VENUS_API uint32_t venus_get_used_mem_size();
typedef Tensor TensorM;
} // namespace venus
} // namespace magik
namespace venus = magik::venus;
ALG_PACK_END

#endif /* __MAGIK_INFERENCEKIT_VENUS_INCLUDE_VENUS_H__ */
