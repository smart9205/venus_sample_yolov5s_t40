t40, gc2063, single mode.
make;make release.And top directory will generate a directory named "execuate_file", which contains venus.so,execuatable file and model. You can use them to get stream and detect. And then, please review the code.


# Run with a single image
inference.cpp
./venus_yolov5s_img yolov5s_t40_magik.bin fall_1054_sys.jpg

# Run with a T40 camera
inference_nv12.cpp sample-Magik.cpp
./venus_yolov5s_cam