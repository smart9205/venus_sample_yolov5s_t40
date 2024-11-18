#ifndef __MAGIK_INFERENCEKIT_VENUS_INCLUDE_UTILS_MATRIX_H__
#define __MAGIK_INFERENCEKIT_VENUS_INCLUDE_UTILS_MATRIX_H__
#include "common/common_type.h"
#include "core/tensor.h"
#include "core/type.h"

ALG_PACK_START
namespace magik {
namespace venus {

/**
 * @param input: a tensor with input format NHWC (UINT16)
 * @param output: a tensor with output format NHWC (UINT16)
 * @example input_tensor shape (N,H,W,C) -> output_tensor shape (N, H/2, W/2, C*4)
 */
VENUS_API void space_to_depth(Tensor &input0, Tensor &output);
/**
 * @param input: a tensor with input format NHWC (UINT16)
 * @param output: a tensor with output format NHWC (UINT16)
 * @example input_tensor_shape (N, H, W, C) -> output_tensor shape(N, H*2, W*2, C/4)
 */
VENUS_API void depth_to_space(Tensor &input0, Tensor &output);
/**
 * @brief Weight sum four tensors of input
 * @param inputs vector for storing intput Tensor(input feature and weight)
 * @warning input 、output has same shape, datatype only support FLOAT
 * @return void
 * @version:
 */
VENUS_API void eltwise_weight_sum(std::vector<Tensor> &inputs, Tensor &output);
/**
 * @param input: a tensor with input format NHWC
 * @param output: a tensor with output format NHWC(Internal data has been converted to a target
 * format such as NCHW)
 * @param dim: a vector with type int. for example NHWC => NCHW, the dim that needs to be passed in
 * is [0, 3, 1, 2]
 */
VENUS_API void permute(Tensor &input, Tensor &output, std::vector<int> &dim);
/**
 * @brief Sample input tensor based on the coordinates of the grid tensor.
 * Tensor input0, Tensor grid, Tensor output
 * @param input0 tensor shape [N, IH, IW, C]
 * @param grid tensor shape [N, OH, OW, 2]
 * @param output tensor shape [N, OH, OW, C]
 * @warning input0, grid, output tensor datatype only support FLOAT
 * @return: void
 * @version:
 */
VENUS_API void grid_sample(Tensor &input0, Tensor &grid, Tensor &output);
/**
 * @brief Add two tensors of input
 * @param Tensor input0, Tensor input1, Tensor output
 *         input0, input1, output has same shape, datatype only support FLOAT
 * @return void
 * @version:
 */
VENUS_API void tensor_add(Tensor &input0, Tensor &input1, Tensor &output);
/**
 * @brief Sub two tensors of input
 * @param Tensor input0, Tensor input1, Tensor output
 *         input0, input1, output has same shape, datatype support UINT8/UINT16/FLOAT
 * @return void
 * @version:
 */
VENUS_API void tensor_sub(Tensor &input0, Tensor &input1, Tensor &output);
/**
 * @param input0: a tensor with input0 format NHWC
 * @param input1: a tensor with input1 format NHWC
 * @param output: a tensor with output format NHWC
 * @warning input0.shape = input1.shape(fp32)
 * **/
VENUS_API void tensor_mul(Tensor &input0, Tensor &input1, Tensor &output);
/**
 * @brief Div two tensors of input
 * @param Tensor input0, Tensor input1, Tensor output
 *         input0, input1, output has same shape, datatype only support FLOAT
 * @return void
 * @version:
 */
VENUS_API void tensor_div(Tensor &input0, Tensor &input1, Tensor &output);

/**
 * @param input: a tensor with input format NHWC(FP32)
 * @param output: a tensor with output format NHWC(FP32)
 * @param mode:algorithm used for upsampleing("nearst" or "bilinear")
 * @warning This interface only supports bilinear interpolation upsampling with magnification of 8
 * **/
VENUS_API void tensor_upsample(Tensor &input0, Tensor &output, UppType mode);

/**
 * @param input: a tensor with input format NHWC
 * @param weight: shape{kernel_w,kernel_h,ic,oc}
 * @param bt: shape{2,oc} bias(oc) + scaler(oc)
 * @param output: a tensor with output format NHWC
 * **/
VENUS_API void tensor_conv2d(Tensor &input, Tensor &weight, Tensor &bt, Tensor &output,
                             ConvParam &conv_param);

/**
 * @param input: a tensor with input format NHWC, (INT10/INT12/UINT10/UINT12)
 * @param kernels: ksize_w = kernels[0], ksize_h = kernels[1]
 * @param strides: stride_w = strides[0], stride_h = strides[1]
 * @param output: a tensor with output format NHWC, (UINT16/INT16)
 * @warning 0 when width/height is not align kernels
 */
VENUS_API void tensor_blocksum(Tensor &input, std::vector<int> &kernels, std::vector<int> &strides,
                               Tensor &output);

/**
 * @brief Averaging a given dimension
 * @param Tensor input(uint16), Tensor output(float)
 * @param axes: a integer vector,Used to indicate which dimensions to average.
 * @example If you want to average the input channels, you need to assign axes to axes=[3]
 * @return  void.
 * @version
 */
VENUS_API void reduce_mean(Tensor &input, Tensor &output, std::vector<int> &axes);

/**
 * @param input: a tensor with input format NHWC or NCHW, (FP32)
 * @param kernels ksize_w = kernels[0], ksize_h = kernels[1]
 * @param strides: stride_w = strides[0], stride_h = strides[1]
 * @param output: a tensor with output format NHWC, (FP32)
 * pad 0
 * @warning output tensor W dimension must be divisible by 2.
 * @warning if the format of thd input data is NCHW, currently only 1 input channel number and NHWC
 * output data format are supported
 */
VENUS_API void tensor_avgpool(std::vector<Tensor> &inputs, std::vector<int> &kernels,
                              std::vector<int> &strides, std::vector<Tensor> &outputs);
/**
 * @details the main functions of this interface are: Y=a*X +b;
 * @param inputs: a tensor with input format NHWC, data type is FP32;
 * @param outputs: a tensor with output format NHWC, data type is FP32;
 * @param weight: a 32bit integer vector, store the weight corresponding to each input data;
 * @param bias: a 32bit float-point vector, store the bias corresponding to each input data;
 */
VENUS_API void tensor_linear_trans(std::vector<Tensor> &inputs, std::vector<Tensor> &outputs,
                                   std::vector<int> &weight, std::vector<float> &bias);

} // namespace venus
} // namespace magik
ALG_PACK_END
#endif
