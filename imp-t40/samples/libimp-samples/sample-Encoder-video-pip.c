/*
 * sample-Encoder-video-change-dualsensor.c
 *
 * Copyright (C) 2014 Ingenic Semiconductor Co.,Ltd
 *
 * All specific API descriptions called in this document can be viewed in the header files under the proj/sdk-lv3/include/api/cn/imp/ directory.
 *
 * */
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>

#include <imp/imp_log.h>
#include <imp/imp_common.h>
#include <imp/imp_system.h>
#include <imp/imp_framesource.h>
#include <imp/imp_encoder.h>
#include <imp/imp_osd.h>
#include <imp/imp_utils.h>
#include <sys/prctl.h>
#include "sample-common.h"

#define OSD_LETTER_NUM 20
#define TAG "Sample-Encoder-video-pip"

pthread_t pip_thread_id = 0;

extern struct chn_conf chn[];
static int byGetFd = 0;
int grpNum = 0;
IMPRgnHandle *prHander;
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

static void *pip_update_thread(void *args)
{
	int index = (int)args;
	int chnNum = chn[index].index;
	int i = 0, ret = 0;
	IMPFrameInfo *frame = NULL;
	int count = 0;

	IMPOSDRgnAttrData rAttrData;
	char thread_name[64];

	IMP_LOG_ERR(TAG, "%s() getframe chn %d: %d x %d.\n", __func__, index, chn[index].fs_chn_attr.picWidth, chn[index].fs_chn_attr.picHeight);
	
	sprintf(thread_name, "OSD%d-%s", index, __func__);
	prctl(PR_SET_NAME, (unsigned long)thread_name);

	
	void *pfrm = malloc(chn[index].fs_chn_attr.picWidth * chn[index].fs_chn_attr.picHeight * 3 /2);
	if(pfrm == NULL){
		printf("malloc yuv buffer failed.\n");
		return NULL;
	}
	memset(pfrm, 0, chn[index].fs_chn_attr.picWidth * chn[index].fs_chn_attr.picHeight * 3 /2);

	ret = IMP_OSD_ShowRgn(prHander[0], 0, 1); 
	
	while(1){
		IMP_FrameSource_SnapFrame(chnNum, PIX_FMT_NV12, chn[index].fs_chn_attr.picWidth, chn[index].fs_chn_attr.picHeight, pfrm, frame);
		
		rAttrData.picData.pData = pfrm;
		IMP_OSD_UpdateRgnAttrData(prHander[0], &rAttrData);
		
		i++;
	}
	
	free(pfrm);
    pip_thread_id = (pthread_t)NULL;
	pthread_exit(NULL);
	
	return (void *)-1;
}


int sample_change_dual_sensor_mode()
{
	int ret,i,index;
	
	IMPCell osdcell = {DEV_ID_OSD, grpNum, 0};
	
	pthread_attr_t attr;

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

	/* Step.3 Encoder init */
	for (i = 0; i < FS_CHN_NUM; i++) {
		if (jointmode_en == 1 && i == 3){
			printf("jointmode:%d, ch%d continue, line %d.\n", jointmode_en, i, __LINE__);
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

	/* Step.4 OSD init */
	if (IMP_OSD_CreateGroup(grpNum) < 0) {
		IMP_LOG_ERR(TAG, "IMP_OSD_CreateGroup(%d) error !\n", grpNum);
		return -1;
	}

	prHander = sample_osd_init_pip(grpNum, chn[0].fs_chn_attr.picWidth, chn[0].fs_chn_attr.picHeight, chn[3].fs_chn_attr.picWidth, chn[3].fs_chn_attr.picHeight);
	if (prHander <= 0) {
		IMP_LOG_ERR(TAG, "OSD init failed\n");
		return -1;
	}
	
	/* Step.5 Bind */
	for (i = 0; i < FS_CHN_NUM; i++) {
		if (jointmode_en == 1 && i == 3){
			IMP_LOG_DBG(TAG, "jointmode:%d, ch%d continue, line %d.\n", jointmode_en, i, __LINE__);
			continue;			
		}
		
		if (i == 0){
			ret = IMP_System_Bind(&chn[0].framesource_chn, &osdcell);
			if (ret < 0) {
				IMP_LOG_ERR(TAG, "Bind FrameSource channel0 and OSD failed\n");
				return -1;
			}

			ret = IMP_System_Bind(&osdcell, &chn[0].imp_encoder);
			if (ret < 0) {
				IMP_LOG_ERR(TAG, "Bind OSD and Encoder failed\n");
				return -1;
			}

			if(!jointmode_en){
				index = 3;
				if (pip_thread_id == (pthread_t)NULL) {
					pthread_attr_init(&attr);
					pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
					ret = pthread_create(&pip_thread_id, &attr, &pip_update_thread, (void *)index);
					pthread_attr_destroy(&attr);
				}
			}
		}else{
			if (chn[i].enable) {
				ret = IMP_System_Bind(&chn[i].framesource_chn, &chn[i].imp_encoder);
				if (ret < 0) {
					IMP_LOG_ERR(TAG, "Bind FrameSource channel%d and Encoder failed\n",i);
					return -1;
				}
			}
		}
	}
	
	/* Step.6 Stream On */
	ret = sample_framesource_streamon();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "ImpStreamOn failed\n");
		return -1;
	}

	/* Step.6-0 Get stream */
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

	
	sleep(5);

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
	
		if (i == 0){
			/* Step.b UnBind */
			ret = IMP_System_UnBind(&osdcell, &chn[0].imp_encoder);
			if (ret < 0) {
				IMP_LOG_ERR(TAG, "Bind OSD and Encoder failed\n");
				return -1;
			}

			ret = IMP_System_UnBind(&chn[0].framesource_chn, &osdcell);
			if (ret < 0) {
				IMP_LOG_ERR(TAG, "UnBind FrameSource and OSD failed\n");
				return -1;
			}
		}else {
			if (chn[i].enable) {
				ret = IMP_System_UnBind(&chn[i].framesource_chn, &chn[i].imp_encoder);
				if (ret < 0) {
					IMP_LOG_ERR(TAG, "UnBind FrameSource channel%d and Encoder failed\n",i);
					return -1;
				}
			}
		}
	}

	/* Step.c OSD exit */
	ret = sample_osd_exit_pip(prHander,grpNum);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "OSD exit failed\n");
		return -1;
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

	return 0;
}


int main(int argc, char *argv[])
{
	int count = 1;
	chn[0].enable = 1;
	chn[1].enable = 0;
	chn[2].enable = 0;
	chn[3].enable = 1;
	chn[4].enable = 0;
	chn[5].enable = 0;
    
	if (argc >= 2) {
        byGetFd = atoi(argv[1]);
    }
	
	while(count--)
	{
		/*JOINT mode*/
		jointmode_en = 0;
		dualmode.joint_mode = IMPISP_NOT_JOINT;
		chn[0].fs_chn_attr.crop.enable = 0;
		chn[0].fs_chn_attr.crop.top = 0;
		chn[0].fs_chn_attr.crop.left = 0;
		chn[0].fs_chn_attr.crop.width = 1920;
		chn[0].fs_chn_attr.crop.height = 1080;
		chn[0].fs_chn_attr.scaler.enable = 1;
		chn[0].fs_chn_attr.scaler.outwidth = 1920;
		chn[0].fs_chn_attr.scaler.outheight = 1080;
		chn[0].fs_chn_attr.picWidth = 1920;
		chn[0].fs_chn_attr.picHeight = 1080;
		

		chn[3].fs_chn_attr.crop.enable = 0;
		chn[3].fs_chn_attr.crop.top = 0;
		chn[3].fs_chn_attr.crop.left = 0;
		chn[3].fs_chn_attr.crop.width = 1920;
		chn[3].fs_chn_attr.crop.height = 1080;
		chn[3].fs_chn_attr.scaler.enable = 1;
		chn[3].fs_chn_attr.scaler.outwidth = 640;
		chn[3].fs_chn_attr.scaler.outheight = 360;
		chn[3].fs_chn_attr.picWidth = 640;
		chn[3].fs_chn_attr.picHeight = 360;
		sample_change_joint_mode(&dualmode);
		sample_change_dual_sensor_mode();
	
	}

	IMP_System_Exit();
	
	return 0;
}
