/*
 * sample-IVS-unbind-move.c
 *
 * Copyright (C) 2016 Ingenic Semiconductor Co.,Ltd
 */
#include <string.h>
#include <stdlib.h>
#include <imp/imp_log.h>
#include <imp/imp_common.h>
#include <imp/imp_system.h>
#include <imp/imp_framesource.h>
#include <imp/imp_ivs.h>
#include <imp/imp_ivs_move.h>
#include "sample-common.h"
#include "inference_nv12.h"

#define TAG "Sample-IVS-unbind-move"

#define MODEL_PATH "yolov5s_t40_magik.bin"

extern struct chn_conf chn[];
std::unique_ptr<venus::BaseNet> test_net;

static int sample_venus_init()
{
	int ret = 0;
 
    ret = venus::venus_init();
    if (0 != ret) {
        fprintf(stderr, "venus init failed.\n");
		return -1;
    }

	test_net = venus::net_create(TensorFormat::NV12);

    ret = test_net->load_model(MODEL_PATH);

	return 0;
}
static int sample_venus_deinit()
{
    int ret = venus::venus_deinit();
    if (0 != ret) {
        fprintf(stderr, "venus deinit failed.\n");
    }

	return ret;
}

int main(int argc, char *argv[])
{
	int i, ret;
	IMPIVSInterface *interface = NULL;
	IMP_IVS_MoveParam param;
	IMP_IVS_MoveOutput *result = NULL;
	IMPFrameInfo frame;
	unsigned char * g_sub_nv12_buf_move = 0;
	chn[0].enable = 0;
	chn[1].enable = 1;
	int sensor_sub_width = 640;
	int sensor_sub_height = 360;
	/* Step.1 System init */
	ret = sample_system_init();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_System_Init() failed\n");
		return -1;
	}
	/* Step.2 FrameSource init */
	ret = sample_framesource_init();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "FrameSource init failed\n");
		return -1;
	}
	g_sub_nv12_buf_move = (unsigned char *)malloc(sensor_sub_width * sensor_sub_height * 3 / 2);
	if (g_sub_nv12_buf_move == 0) {
		printf("error(%s,%d): malloc buf failed \n", __func__, __LINE__);
		return NULL;
	}
	/* Step.3 framesource Stream On */
	ret = sample_framesource_streamon();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "ImpStreamOn failed\n");
		return -1;
	}
	/* Step.4 ivs move start */
	ret = sample_venus_init();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_venus_init failed\n");
		return -1;
	}

	/* Step.5 start to get ivs move result */
	for (i = 0; i < NR_FRAMES_TO_SAVE; i++) {

		ret = IMP_FrameSource_SnapFrame(1, PIX_FMT_NV12, sensor_sub_width, sensor_sub_height, g_sub_nv12_buf_move, &frame);
		if (ret < 0) {
			printf("%d get frame failed try again\n", 0);
			usleep(30*1000);
		}
		frame.virAddr = (unsigned int)g_sub_nv12_buf_move;
		Goto_Magik_Detect((char *)frame.virAddr, sensor_sub_width, sensor_sub_height);
	}

	free(g_sub_nv12_buf_move);
	/* Step.6 ivs move stop */
	ret = sample_venus_deinit();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_venus_deinit() failed\n");
		return -1;
	}
	/* Step.7 Stream Off */
	ret = sample_framesource_streamoff();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "FrameSource StreamOff failed\n");
		return -1;
	}
	/* Step.8 FrameSource exit */
	ret = sample_framesource_exit();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "FrameSource exit failed\n");
		return -1;
	}
	/* Step.9 System exit */
	ret = sample_system_exit();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_system_exit() failed\n");
		return -1;
	}
	return 0;
}
