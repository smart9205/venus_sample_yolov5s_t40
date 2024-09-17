/*
 * sample-Encoder-video-change-dualsensor.c
 *
 * Copyright (C) 2014 Ingenic Semiconductor Co.,Ltd
 *
 * All specific API descriptions called in this document can be viewed in the header files under the proj/sdk-lv3/include/api/cn/imp/ directory.
 *
 * */
#include <stdio.h>
#include <stdlib.h>
#include <imp/imp_log.h>
#include <imp/imp_common.h>
#include <imp/imp_system.h>
#include <imp/imp_framesource.h>
#include <imp/imp_encoder.h>

#include "sample-common.h"

#define TAG "Sample-Encoder-video-change-dualsensor"

extern struct chn_conf chn[];
static int byGetFd = 0;
IMPISPCameraInputMode dualmode = {
	.sensor_num = SENSOR_NUM,
	.dual_mode = DUALSENSOR_MODE,
	.dual_mode_switch = {
		.en = 0,
	},
	.joint_mode = JOINT_MODE,
};

extern struct chn_conf chn[];
extern int jointmode_en;
char jj[]="";

int sample_change_dual_sensor_mode()
{
	int ret,i;
	
	/* Step.1 System init */
	ret = sample_system_init();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_System_Init() failed\n");
		return -1;
	}
#if 1
	/* Step.2 FrameSource init */
	ret = sample_framesource_init();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "FrameSource init failed\n");
		return -1;
	}

	/* Step.3 Encoder init */
	for (i = 0; i < FS_CHN_NUM; i++) {
		if (jointmode_en == 1 && i == 3){
			continue;			
		}
		
		if (chn[i].enable) {
			ret = IMP_Encoder_CreateGroup(chn[i].index);
			if (ret < 0) {
				IMP_LOG_ERR(TAG, "IMP_Encoder_CreateGroup(%d) error !\n", chn[i].index);
				return -1;
			}
		}
	}

	ret = sample_encoder_init();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "Encoder init failed\n");
		return -1;
	}

	/* Step.4 Bind */
	for (i = 0; i < FS_CHN_NUM; i++) {
		if (jointmode_en == 1 && i == 3){
			continue;			
		}
		
		if (chn[i].enable) {
			ret = IMP_System_Bind(&chn[i].framesource_chn, &chn[i].imp_encoder);
			if (ret < 0) {
				IMP_LOG_ERR(TAG, "Bind FrameSource channel%d and Encoder failed\n",i);
				return -1;
			}
		}
	}

	/* Step.5 Stream On */
	ret = sample_framesource_streamon();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "ImpStreamOn failed\n");
		return -1;
	}

	/* Step.6 Get stream */
    if (byGetFd) {
        ret = sample_get_video_stream_byfd();
        if (ret < 0) {
            IMP_LOG_ERR(TAG, "Get video stream byfd failed\n");
            return -1;
        }
    } else {
        ret = sample_get_video_stream();
        if (ret < 0) {
            IMP_LOG_ERR(TAG, "Get video stream failed\n");
            return -1;
        }
    }

	/* Exit sequence as follow */
	/* Step.7 Stream Off */
	ret = sample_framesource_streamoff();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "FrameSource StreamOff failed\n");
		return -1;
	}

	/* Step.8 UnBind */
	for (i = 0; i < FS_CHN_NUM; i++) {
		if (jointmode_en == 1 && i == 3){
			continue;			
		}
		
		if (chn[i].enable) {
			ret = IMP_System_UnBind(&chn[i].framesource_chn, &chn[i].imp_encoder);
			if (ret < 0) {
				IMP_LOG_ERR(TAG, "UnBind FrameSource channel%d and Encoder failed\n",i);
				return -1;
			}
		}
	}

	/* Step.9 Encoder exit */
	ret = sample_encoder_exit();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "Encoder exit failed\n");
		return -1;
	}

	/* Step.10 FrameSource exit */
	ret = sample_framesource_exit();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "FrameSource exit failed\n");
		return -1;
	}
#endif
	/* Step.11 System exit */
	ret = sample_system_exit();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_system_exit() failed\n");
		return -1;
	}
	
	return 0;
}

int main(int argc, char *argv[])
{
	int i, ret;
	int count = 1;
	chn[0].enable = 1;
	chn[1].enable = 0;
	chn[2].enable = 1;
	chn[3].enable = 1;
	chn[4].enable = 0;
	chn[5].enable = 1;
    
	if (argc >= 2) {
        byGetFd = atoi(argv[1]);
    }

	//while(count--)
	while(1)
	{
#if 0
		usleep(30000);
		/*JOINT mode*/
		jointmode_en = 1;
		dualmode.joint_mode = IMPISP_MAIN_ON_THE_ABOVE;
		chn[0].fs_chn_attr.crop.enable = 0;
		chn[0].fs_chn_attr.crop.top = 0;
		chn[0].fs_chn_attr.crop.left = 0;
		chn[0].fs_chn_attr.crop.width = 2560;
		chn[0].fs_chn_attr.crop.height = 1920;
		chn[0].fs_chn_attr.scaler.enable = 1;
		chn[0].fs_chn_attr.scaler.outwidth = 1600;
		chn[0].fs_chn_attr.scaler.outheight = 1200;
		chn[0].fs_chn_attr.picWidth = 1600;
		chn[0].fs_chn_attr.picHeight = 2400;
		
		chn[2].fs_chn_attr.crop.enable = 0;
		chn[2].fs_chn_attr.crop.top = 0;
		chn[2].fs_chn_attr.crop.left = 0;
		chn[2].fs_chn_attr.crop.width = 1920;
		chn[2].fs_chn_attr.crop.height = 1080;
		chn[2].fs_chn_attr.scaler.enable = 1;
		chn[2].fs_chn_attr.scaler.outwidth = 800;
		chn[2].fs_chn_attr.scaler.outheight = 600;
		chn[2].fs_chn_attr.picWidth = 800;
		chn[2].fs_chn_attr.picHeight = 600;

		chn[3].fs_chn_attr.crop.enable = 0;
		chn[3].fs_chn_attr.crop.top = 0;
		chn[3].fs_chn_attr.crop.left = 0;
		chn[3].fs_chn_attr.crop.width = 1920;
		chn[3].fs_chn_attr.crop.height = 1080;
		chn[3].fs_chn_attr.scaler.enable = 1;
		chn[3].fs_chn_attr.scaler.outwidth = 1600;
		chn[3].fs_chn_attr.scaler.outheight = 1200;
		chn[3].fs_chn_attr.picWidth = 1600;
		chn[3].fs_chn_attr.picHeight = 1200;

		chn[5].fs_chn_attr.crop.enable = 0;
		chn[5].fs_chn_attr.crop.top = 0;
		chn[5].fs_chn_attr.crop.left = 0;
		chn[5].fs_chn_attr.crop.width = 1920;
		chn[5].fs_chn_attr.crop.height = 1080;
		chn[5].fs_chn_attr.scaler.enable = 1;
		chn[5].fs_chn_attr.scaler.outwidth = 800;
		chn[5].fs_chn_attr.scaler.outheight = 600;
		chn[5].fs_chn_attr.picWidth = 800;
		chn[5].fs_chn_attr.picHeight = 600;
		sample_change_joint_mode(&dualmode);
		sample_change_dual_sensor_mode();
	
#endif
		usleep(30000);
		/*change to NOT JOINT mode*/
		jointmode_en = 0;
		dualmode.joint_mode = IMPISP_NOT_JOINT;
		
		chn[0].fs_chn_attr.crop.enable = 0;
		chn[0].fs_chn_attr.crop.top = 0;
		chn[0].fs_chn_attr.crop.left = 0;
		chn[0].fs_chn_attr.crop.width = 2560;
		chn[0].fs_chn_attr.crop.height = 1920;
		chn[0].fs_chn_attr.scaler.enable = 1;
		chn[0].fs_chn_attr.scaler.outwidth = 2560;
		chn[0].fs_chn_attr.scaler.outheight = 1920;
		chn[0].fs_chn_attr.picWidth = 2560;
		chn[0].fs_chn_attr.picHeight = 1920;

		chn[2].fs_chn_attr.crop.enable = 0;
		chn[2].fs_chn_attr.crop.top = 0;
		chn[2].fs_chn_attr.crop.left = 0;
		chn[2].fs_chn_attr.crop.width = 1920;
		chn[2].fs_chn_attr.crop.height = 1080;
		chn[2].fs_chn_attr.scaler.enable = 1;
		chn[2].fs_chn_attr.scaler.outwidth = 800;
		chn[2].fs_chn_attr.scaler.outheight = 600;
		chn[2].fs_chn_attr.picWidth = 800;
		chn[2].fs_chn_attr.picHeight = 600;

		chn[3].fs_chn_attr.crop.enable = 0;
		chn[3].fs_chn_attr.crop.top = 0;
		chn[3].fs_chn_attr.crop.left = 0;
		chn[3].fs_chn_attr.crop.width = 1920;
		chn[3].fs_chn_attr.crop.height = 1080;
		chn[3].fs_chn_attr.scaler.enable = 1;
		chn[3].fs_chn_attr.scaler.outwidth = 800;
		chn[3].fs_chn_attr.scaler.outheight = 600;
		chn[3].fs_chn_attr.picWidth = 800;
		chn[3].fs_chn_attr.picHeight = 600;

		chn[5].fs_chn_attr.crop.enable = 0;
		chn[5].fs_chn_attr.crop.top = 0;
		chn[5].fs_chn_attr.crop.left = 0;
		chn[5].fs_chn_attr.crop.width = 1920;
		chn[5].fs_chn_attr.crop.height = 1080;
		chn[5].fs_chn_attr.scaler.enable = 1;
		chn[5].fs_chn_attr.scaler.outwidth = 800;
		chn[5].fs_chn_attr.scaler.outheight = 600;
		chn[5].fs_chn_attr.picWidth = 800;
		chn[5].fs_chn_attr.picHeight = 600;

		sample_change_joint_mode(&dualmode);
		sample_change_dual_sensor_mode();
#if 1		
	//	sleep(2);
		usleep(30000);
		/*change to JOINT mode*/
		jointmode_en = 1;
		dualmode.joint_mode = IMPISP_MAIN_ON_THE_ABOVE;
		
		chn[0].fs_chn_attr.crop.enable = 0;
		chn[0].fs_chn_attr.crop.top = 0;
		chn[0].fs_chn_attr.crop.left = 0;
		chn[0].fs_chn_attr.crop.width = 2560;
		chn[0].fs_chn_attr.crop.height = 1920;
		chn[0].fs_chn_attr.scaler.enable = 1;
		chn[0].fs_chn_attr.scaler.outwidth = 1600;
		chn[0].fs_chn_attr.scaler.outheight = 1200;
		chn[0].fs_chn_attr.picWidth = 1600;
		chn[0].fs_chn_attr.picHeight = 2400;
		
		chn[2].fs_chn_attr.crop.enable = 0;
		chn[2].fs_chn_attr.crop.top = 0;
		chn[2].fs_chn_attr.crop.left = 0;
		chn[2].fs_chn_attr.crop.width = 1920;
		chn[2].fs_chn_attr.crop.height = 1080;
		chn[2].fs_chn_attr.scaler.enable = 1;
		chn[2].fs_chn_attr.scaler.outwidth = 1280;
		chn[2].fs_chn_attr.scaler.outheight = 720;
		chn[2].fs_chn_attr.picWidth = 1280;
		chn[2].fs_chn_attr.picHeight = 720;

		chn[3].fs_chn_attr.crop.enable = 0;
		chn[3].fs_chn_attr.crop.top = 0;
		chn[3].fs_chn_attr.crop.left = 0;
		chn[3].fs_chn_attr.crop.width = 1920;
		chn[3].fs_chn_attr.crop.height = 1080;
		chn[3].fs_chn_attr.scaler.enable = 1;
		chn[3].fs_chn_attr.scaler.outwidth = 1600;
		chn[3].fs_chn_attr.scaler.outheight = 1200;
		chn[3].fs_chn_attr.picWidth = 1600;
		chn[3].fs_chn_attr.picHeight = 1200;

		chn[5].fs_chn_attr.crop.enable = 0;
		chn[5].fs_chn_attr.crop.top = 0;
		chn[5].fs_chn_attr.crop.left = 0;
		chn[5].fs_chn_attr.crop.width = 1920;
		chn[5].fs_chn_attr.crop.height = 1080;
		chn[5].fs_chn_attr.scaler.enable = 1;
		chn[5].fs_chn_attr.scaler.outwidth = 640;
		chn[5].fs_chn_attr.scaler.outheight = 360;
		chn[5].fs_chn_attr.picWidth = 640;
		chn[5].fs_chn_attr.picHeight = 360;
		
		sample_change_joint_mode(&dualmode);
		sample_change_dual_sensor_mode();
#endif
	}

	IMP_System_Exit();
	
	return 0;
}
