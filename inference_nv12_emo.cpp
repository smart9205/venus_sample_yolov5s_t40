/*
 *         (C) COPYRIGHT Ingenic Limited
 *              ALL RIGHT RESERVED
 *
 * File        : model_run.cc
 * Authors     : klyu
 * Create Time : 2020-10-28 12:22:44 (CST)
 * Description :
 *
 */

#include "inference_nv12.h"
#include <math.h>
#include <fstream>
#include <iostream>
#include <memory.h>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#define IS_ALIGN_64(x) (((size_t)x) & 0x3F)

extern std::unique_ptr<venus::BaseNet> test_net;

void write_output_bin(const float* out_ptr, int size)
{
    std::string out_name = "out_res.bin";
    std::ofstream owput;
    owput.open(out_name, std::ios::binary);
    if (!owput || !owput.is_open() || !owput.good()) {
        owput.close();
        return ;
    }
    owput.write((char *)out_ptr, size * sizeof(float));
    owput.close();
    return ;
}


typedef struct
{
    unsigned char* image;  
    int w;
    int h;
}input_info_t;

struct PixelOffset {
    int top;
    int bottom;
    int left;
    int right;
};


void check_pixel_offset(PixelOffset &pixel_offset){
    // 5 5 -> 6 4
    // padding size not is Odd number
    if(pixel_offset.top % 2 == 1){
        pixel_offset.top += 1;
        pixel_offset.bottom -=1;
    }
    if(pixel_offset.left % 2 == 1){
        pixel_offset.left += 1;
        pixel_offset.right -=1;
    }
}

void trans_coords(std::vector<magik::venus::ObjBbox_t> &in_boxes, PixelOffset &pixel_offset,float scale){
    
    printf("pad_x:%d pad_y:%d scale:%f \n",pixel_offset.left,pixel_offset.top,scale);
    for(int i = 0; i < in_boxes.size(); i++) {

        in_boxes[i].box.x0 = (in_boxes[i].box.x0 - pixel_offset.left) / scale;
        in_boxes[i].box.x1 = (in_boxes[i].box.x1 - pixel_offset.left) / scale;
        in_boxes[i].box.y0 = (in_boxes[i].box.y0 - pixel_offset.top) / scale;
        in_boxes[i].box.y1 = (in_boxes[i].box.y1 - pixel_offset.top) / scale;
    }
}


void generateBBox(std::vector<venus::Tensor> out_res, std::vector<magik::venus::ObjBbox_t>& candidate_boxes, int img_w, int img_h);
void manyclass_nms(std::vector<magik::venus::ObjBbox_t> &input, std::vector<magik::venus::ObjBbox_t> &output, int classnums, int type, float nms_threshold);

int Goto_Magik_Detect(char * nv12Data, int width, int height)
{
    /*set*/
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(0, &mask);
    if (sched_setaffinity(0, sizeof(mask), &mask) == -1) {
            printf("warning: could not set CPU affinity, continuing...\n");
    }


    bool cvtbgra;
    cvtbgra = false;
    void *handle = NULL;

    int ori_img_h;
    int ori_img_w;
    float scale;
    // int in_w = 640, in_h = 384;
    int in_w = 48, in_h = 48;
    
    PixelOffset pixel_offset;
 

	std::unique_ptr<venus::Tensor> input;
    input_info_t input_src;
    //
    input_src.w = width;
    input_src.h = height;
    input_src.image = (unsigned char*)nv12Data;
    
    //---------------------process-------------------------------
    // get ori image w h
    ori_img_w = input_src.w;
    ori_img_h = input_src.h;
    printf("ori_image w,h: %d ,%d \n",ori_img_w,ori_img_h);
    
    // int line_stride = ori_img_w;
    input = test_net->get_input(0);
    magik::venus::shape_t rgba_input_shape = input->shape();
    printf("model-->%d %d %d \n",rgba_input_shape[1], rgba_input_shape[2], rgba_input_shape[3]);   // [batch, height, width, channel]
    
    in_h = rgba_input_shape[1];
    in_w = rgba_input_shape[2];
    input->reshape({1, in_h, in_w, 1});

    // uint8_t *indata = input->mudata<uint8_t>();
    
    std::cout << "input shape:" << std::endl;
    printf("-->%d %d \n",in_h, in_w);

    //resize and padding
    magik::venus::Tensor temp_ori_input({1, ori_img_h, ori_img_w, 1}, TensorFormat::NV12);
    uint8_t *tensor_data = temp_ori_input.mudata<uint8_t>();
    int src_size = int(ori_img_h * ori_img_w * 1.5);
    magik::venus::memcopy((void*)tensor_data, (void*)input_src.image, src_size * sizeof(uint8_t));

    float scale_x = (float)in_w/(float)ori_img_w;
    float scale_y = (float)in_h/(float)ori_img_h;
    scale = scale_x < scale_y ? scale_x:scale_y;  //min scale
    printf("scale---> %f\n",scale);

    int valid_dst_w = (int)(scale*ori_img_w);
    if (valid_dst_w % 2 == 1)
        valid_dst_w = valid_dst_w + 1;
    int valid_dst_h = (int)(scale*ori_img_h);
    if (valid_dst_h % 2 == 1)
        valid_dst_h = valid_dst_h + 1;

    int dw = in_w - valid_dst_w;
    int dh = in_h - valid_dst_h;

    pixel_offset.top = int(round(float(dh)/2 - 0.1));
    pixel_offset.bottom = int(round(float(dh)/2 + 0.1));
    pixel_offset.left = int(round(float(dw)/2 - 0.1));
    pixel_offset.right = int(round(float(dw)/2 + 0.1));
    
//    check_pixel_offset(pixel_offset);

    // Initialize parameters for the common_resize function to resize the input NV12 image to 48x48  
    magik::venus::BsCommonParam param;
    param.pad_val = 0;
    param.pad_type = magik::venus::BsPadType::SYMMETRY;
    param.input_height = ori_img_h;
    param.input_width = ori_img_w;
    param.input_line_stride = ori_img_w;
    param.in_layout = magik::venus::ChannelLayout::NV12;
    param.out_layout = magik::venus::ChannelLayout::NV12;

    // Create an output tensor to hold the resized NV12 image  
    magik::venus::Tensor resized_tensor({1, in_h, in_w, 1}, magik::venus::TensorFormat::NV12); // Prepare for resized NV12 image  
    
    // Perform the resize operation  
    magik::venus::common_resize((const void*)tensor_data, resized_tensor, magik::venus::AddressLocate::NMEM_VIRTUAL, &param);  
    
    // The resized_tensor now contains the resized NV12 image.  
    // Since the first part of NV12 data is the Y component representing the grayscale image,  
    // extract the Y component to use as input to your face emotion detection model.  
    // Assuming the resized Y component is directly usable as model input  
    
    // Prepare the model's input tensor   
    input->reshape({1, in_h, in_w, 1}); // Reshape for a single-channel 48x48 input  
    
    // The Y component is the first 48*48 bytes of the resized_tensor data  
    uint8_t *resized_data = resized_tensor.mudata<uint8_t>();  
    
    // Copy the resized Y component into the model's input tensor  
    memcpy(input->mudata<uint8_t>(), resized_data, in_h * in_w);  
 
    printf("resize padding over: \n");
    printf("resize valid_dst, w:%d h %d\n",valid_dst_w,valid_dst_h);
    printf("padding info top :%d bottom %d left:%d right:%d \n",pixel_offset.top,pixel_offset.bottom,pixel_offset.left,pixel_offset.right);

    test_net->run();

        
    // Retrieve model output  
    std::unique_ptr<const venus::Tensor> output_tensor = test_net->get_output(0); // Assuming single output  
    const float* output_data = output_tensor->data<float>();  
    
    // The output data processing depends on the model's output format  
    // If the model's final layer was a LogSoftmax that has been removed,  
    // you might need to apply Softmax or LogSoftmax manually to interpret the outputs  
    
    int num_classes = output_tensor->shape()[1]; // Assuming the output shape is [1, num_classes]  
    std::vector<float> softmax_output(num_classes);  
    float max_val = *std::max_element(output_data, output_data + num_classes);  
    float sum_exp = 0.0;  
    
    for (int i = 0; i < num_classes; ++i) {  
        softmax_output[i] = exp(output_data[i] - max_val); // Compute softmax  
        sum_exp += softmax_output[i];  
    }  
    for (int i = 0; i < num_classes; ++i) {  
        softmax_output[i] /= sum_exp; // Normalize  
    }  
    
    // Optionally convert softmax to log-softmax if necessary  
    for (int i = 0; i < num_classes; ++i) {  
        softmax_output[i] = log(softmax_output[i]);  
    }  
    
    // At this point, `softmax_output` contains the log-softmax probabilities  
    // You can now interpret these probabilities according to your application's needs  
    // For example, finding the class with the highest probability:  
    int predicted_class = std::distance(softmax_output.begin(), std::max_element(softmax_output.begin(), softmax_output.end()));  
    float confidence = softmax_output[predicted_class];  
    
    std::cout << "Predicted class: " << predicted_class << " with confidence: " << confidence << std::endl;  

    return 0;
}

void generateBBox(std::vector<venus::Tensor> out_res, std::vector<magik::venus::ObjBbox_t>& candidate_boxes, int img_w, int img_h){

  float person_threshold = 0.3;
  int classes = 80;
  float nms_threshold = 0.6;
  std::vector<float> strides = {8.0, 16.0, 32.0};
  int box_num = 3;
  std::vector<float> anchor = {10,13,  16,30,  33,23, 30,61,  62,45,  59,119, 116,90,  156,198,  373,326};


  std::vector<magik::venus::ObjBbox_t>  temp_boxes;
  venus::generate_box(out_res, strides, anchor, temp_boxes, img_w, img_h, classes, box_num, person_threshold, magik::venus::DetectorType::YOLOV5);
//  venus::nms(temp_boxes, candidate_boxes, nms_threshold); 
  manyclass_nms(temp_boxes, candidate_boxes, classes, 0, nms_threshold);

}

void manyclass_nms(std::vector<magik::venus::ObjBbox_t> &input, std::vector<magik::venus::ObjBbox_t> &output, int classnums, int type, float nms_threshold) {
  int box_num = input.size();
  std::vector<int> merged(box_num, 0);
  std::vector<magik::venus::ObjBbox_t> classbuf;
  for (int clsid = 0; clsid < classnums; clsid++) {
    classbuf.clear();
    for (int i = 0; i < box_num; i++) {
      if (merged[i])
        continue;
      if(clsid!=input[i].class_id)
        continue;
      classbuf.push_back(input[i]);
      merged[i] = 1;

    }
    magik::venus::nms(classbuf, output, nms_threshold, magik::venus::NmsType::HARD_NMS);
  }
}






