/*
 *         (C) COPYRIGHT Ingenic Limited
 *              ALL RIGHT RESERVED
 *
 * File        : tensor.h
 * Authors     : klyu
 * Create Time : 2020-08-04 14:58:26 (CST)
 * Description : tensor of api
 *
 */

#ifndef __MAGIK_INFERENCEKIT_VENUS_INCLUDE_CORE_TENSOR_H__
#define __MAGIK_INFERENCEKIT_VENUS_INCLUDE_CORE_TENSOR_H__

#include "type.h"
#include <initializer_list>
#include <stdint.h>
#include <vector>

ALG_PACK_START
namespace magik {
namespace venus {
using shape_t = std::vector<int32_t>;
typedef shape_t Shape;
class VENUS_API Tensor {
public:
    Tensor(shape_t shape, TensorFormat fmt = TensorFormat::NHWC,
           DataType data_type = DataType::UINT8);
    Tensor(std::initializer_list<int32_t> shape, TensorFormat fmt = TensorFormat::NHWC);
    /*data must be alloced by nmem_memalign, and should be aligned with 64 bytes*/
    Tensor(void *vdata, size_t bytes_size, TensorFormat fmt = TensorFormat::NHWC);
    /*shallow copy constructor*/
    Tensor(const Tensor &t);
    /*deep: true, deep copy constructor
     *      false, shallow copy contructor
     */
    Tensor(const Tensor &t, bool deep);
    Tensor(void *tsx);       /*for internal*/
    Tensor(const void *tsx); /*for internal*/
    Tensor();                /*for ai*/
    virtual ~Tensor();

    shape_t shape() const;
    DataType data_type() const;
    void reshape(shape_t &shape) const;
    void reshape(std::initializer_list<int32_t> shape) const;
    template <typename T>
    const T *data() const;
    template <typename T>
    T *mudata() const;
    void free_data() const;
    /*data must be alloced by Device::CPU, and should be aligned with 64 bytes*/
    int set_data(void *vdata, size_t bytes_size);
    /*data must be alloced by Device::CPU/EXTERNAL, and should be aligned with 64 bytes*/
    int set_data(void *vdata, void *pdata, size_t bytes_size, Device dev);
    void *get_tsx() const; /*for internal*/
    int step(int dim) const;
    int get_bytes_size() const;
    void set_dim_align(int dim, int align);

private:
    void *tensorx = NULL;
    int *ref_count = NULL;
};
} // namespace venus
} // namespace magik
ALG_PACK_END

#endif /* __MAGIK_INFERENCEKIT_VENUS_INCLUDE_CORE_TENSOR_H__ */
