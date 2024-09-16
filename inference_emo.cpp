/*
 *         (C) COPYRIGHT Ingenic Limited
 *              ALL RIGHT RESERVED
 *
 * File        : inference.cc
 * Authors     : ffzhou
 * Create Time : 2022-07-16 09:22:44 (CST)
 * Description :
 *
 */
#define STB_IMAGE_IMPLEMENTATION
#include "./stb/stb_image.h"
#include "./stb/drawing.hpp"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "./stb/stb_image_write.h"
static const uint8_t color[3] = {0xff, 0, 0};

#include "venus.h"
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
#include <algorithm>

#define STB_IMAGE_RESIZE_IMPLEMENTATION  
#include "stb_image_resize.h"  


#define TIME
#ifdef TIME
#include <sys/time.h>
#endif

#ifdef VENUS_PROFILE
#define RUN_CNT 10
#else
#define RUN_CNT 1
#endif

#define IS_ALIGN_64(x) (((size_t)x) & 0x3F)

using namespace std;
using namespace magik::venus;

struct PixelOffset {
    int top;
    int bottom;
    int left;
    int right;
};

uint8_t* read_bin(const char* path)
{
    std::ifstream infile;
    infile.open(path, std::ios::binary | std::ios::in);
    infile.seekg(0, std::ios::end);
    int length = infile.tellg();
    infile.seekg(0, std::ios::beg);
    uint8_t* buffer_pointer = new uint8_t[length];
    infile.read((char*)buffer_pointer, length);
    infile.close();
    return buffer_pointer;
}

std::vector<std::string> splitString(std::string srcStr, std::string delimStr,bool repeatedCharIgnored = false)
{
    std::vector<std::string> resultStringVector;
    std::replace_if(srcStr.begin(), srcStr.end(), [&](const char& c){if(delimStr.find(c)!=std::string::npos){return true;}else{return false;}}, delimStr.at(0));
    size_t pos=srcStr.find(delimStr.at(0));
    std::string addedString="";
    while (pos!=std::string::npos) {
        addedString=srcStr.substr(0,pos);
        if (!addedString.empty()||!repeatedCharIgnored) {
            resultStringVector.push_back(addedString);
        }
        srcStr.erase(srcStr.begin(), srcStr.begin()+pos+1);
        pos=srcStr.find(delimStr.at(0));
    }
    addedString=srcStr;
    if (!addedString.empty()||!repeatedCharIgnored) {
        resultStringVector.push_back(addedString);
    }
    return resultStringVector;
}

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
    for(int i = 0; i < (int)in_boxes.size(); i++) {
        in_boxes[i].box.x0 = (in_boxes[i].box.x0 - pixel_offset.left) / scale;
        in_boxes[i].box.x1 = (in_boxes[i].box.x1 - pixel_offset.left) / scale;
        in_boxes[i].box.y0 = (in_boxes[i].box.y0 - pixel_offset.top) / scale;
        in_boxes[i].box.y1 = (in_boxes[i].box.y1 - pixel_offset.top) / scale;
    }
}

void generateBBox(std::vector<venus::Tensor> out_res, std::vector<magik::venus::ObjBbox_t>& candidate_boxes, int img_w, int img_h);
void manyclass_nms(std::vector<magik::venus::ObjBbox_t> &input, std::vector<magik::venus::ObjBbox_t> &output, int classnums, int type, float nms_threshold);


int main(int argc, char **argv) {
    /*set*/
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(0, &mask);
    if (sched_setaffinity(0, sizeof(mask), &mask) == -1) {
            printf("warning: could not set CPU affinity, continuing...\n");
    }

#ifdef VENUS_DEBUG
    int ret = 0;
    if (argc != 3)
    {
        printf("%s model_path image_bin\n", argv[0]);
        exit(0);
    }
    std::string model_path = argv[1];
    std::string image_bin = argv[2];

    uint8_t* imagedata = read_bin(image_bin.c_str());
	std::vector<std::string> result_str = splitString(splitString(image_bin, ".")[0],"_");
    int vec_size = result_str.size();

    int n = atoi(result_str[vec_size - 4].c_str());
    int in_h = atoi(result_str[vec_size - 3].c_str());
    int in_w = atoi(result_str[vec_size - 2].c_str());
    int c = atoi(result_str[vec_size - 1].c_str());
    printf("image_bin shape:%d %d %d %d\n", n, in_h, in_w, c);

    std::unique_ptr<venus::Tensor> input;
    ret = venus::venus_init();
    if (0 != ret) {
        fprintf(stderr, "venus init failed.\n");
        exit(0);
    }

    std::unique_ptr<venus::BaseNet> test_net;
    test_net = venus::net_create(TensorFormat::NHWC);
    ret = test_net->load_model(model_path.c_str());

    input = test_net->get_input(0);
    magik::venus::shape_t rgba_input_shape = input->shape();
    printf("model-->%d ,%d %d \n",rgba_input_shape[1], rgba_input_shape[2], rgba_input_shape[3]);
    input->reshape({1, in_h, in_w , 4});
    uint8_t *indata = input->mudata<uint8_t>();
    std::cout << "input shape:" << std::endl;
    printf("-->%d %d \n",in_h, in_w);
    int data_cnt = 1;
    for (auto i : input->shape()) 
    {
        std::cout << i << ",";
        data_cnt *= i;
    }
    std::cout << std::endl;

	for (int i = 0; i < in_h; i ++)
	{
		for (int j = 0; j < in_w; j++)
		{
			indata[i*in_w*4 + j*4 + 0] = imagedata[i*in_w*3 + j*3 + 0];
			indata[i*in_w*4 + j*4 + 1] = imagedata[i*in_w*3 + j*3 + 1];
			indata[i*in_w*4 + j*4 + 2] = imagedata[i*in_w*3 + j*3 + 2];
			indata[i*in_w*4 + j*4 + 3] = 0;
		}
	}

    test_net->run();

#else

    int ret = 0;
    if (argc < 3) {  
        std::cerr << "Usage: " << argv[0] << " <model_path> <image_path>" << std::endl;  
        exit(0);
    }  
  
    int ori_img_h = -1;  
    int ori_img_w = -1;  
    float scale = 1.0;  
    int in_w = 48, in_h = 48; // Assuming these are the model's expected input dimensions  
  
    std::unique_ptr<venus::Tensor> input;  
    int ret = venus::venus_init();  
    if (0 != ret) {  
        std::cerr << "venus init failed.\n";  
        exit(0);  
    }  
    std::unique_ptr<venus::BaseNet> test_net;  
    test_net = venus::net_create(TensorFormat::NHWC);  
  
    std::string model_path = argv[1];  
    ret = test_net->load_model(model_path.c_str());  
  

    // Load the color image as RGB  
    int comp = 0;   
    unsigned char* imagedata = stbi_load(argv[2], &ori_img_w, &ori_img_h, &comp, 3); // Load as RGB  
    if (imagedata == nullptr) {  
        std::cerr << "Error loading image\n";  
        return 1;  
    }  
    
    // Convert RGB to Grayscale  
    unsigned char* grayscaleData = new unsigned char[ori_img_w * ori_img_h];  
    for (int i = 0; i < ori_img_h; ++i) {  
        for (int j = 0; j < ori_img_w; ++j) {  
            int index = i * ori_img_w + j;  
            unsigned char r = imagedata[index * 3];
            unsigned char g = imagedata[index * 3 + 1];  
            unsigned char b = imagedata[index * 3 + 2];  
            grayscaleData[index] = static_cast<unsigned char>(0.299 * r + 0.587 * g + 0.114 * b);  
        }  
    }  

  
    // Resize grayscale image to model's expected input dimensions  
    unsigned char* resizedData = new unsigned char[in_w * in_h];  
    stbir_resize_uint8(grayscaleData, ori_img_w, ori_img_h, 0, resizedData, in_w, in_h, 0, 1);  
  
    // Prepare model input  
    input = test_net->get_input(0);  
    input->reshape({1, in_h, in_w, 1}); // For grayscale, the channel dimension is 1  
    memcpy(input->mudata<uint8_t>(), resizedData, in_h * in_w);  
  
    test_net->run();  
  
    // Process model output
    std::unique_ptr<const venus::Tensor> output_tensor = test_net->get_output(0); // Assuming single output  
    const float* output_data = output_tensor->data<float>();  
  
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
    int predicted_class = std::distance(softmax_output.begin(), std::max_element(softmax_output.begin(), softmax_output.end()));  
    float confidence = softmax_output[predicted_class];  
  
    std::cout << "Predicted class: " << predicted_class << " with confidence: " << confidence << std::endl;  
  
    // Cleanup  
    stbi_image_free(imagedata); // Free the original image data  
    delete[] grayscaleData; // Free the grayscale image data  
    delete[] resizedData; // Free the resized image data  
  
    return 0;  
#endif

}

void generateBBox(std::vector<venus::Tensor> out_res, std::vector<magik::venus::ObjBbox_t>& candidate_boxes, int img_w, int img_h)
{
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


