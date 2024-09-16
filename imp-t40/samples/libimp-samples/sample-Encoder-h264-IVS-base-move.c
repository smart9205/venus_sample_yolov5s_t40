/*
 * sample-Encoder-h264-IVS-move.c
 *
 * Copyright (C) 2016 Ingenic Semiconductor Co.,Ltd
 */
#include <string.h>
#include <imp/imp_log.h>
#include <imp/imp_common.h>
#include <imp/imp_system.h>
#include <imp/imp_framesource.h>
#include <imp/imp_encoder.h>
#include <imp/imp_ivs.h>
#include <imp/imp_ivs_base_move.h>
#include "sample-common.h"
#define TAG "Sample-Encoder-h264-IVS-base-move"
#define SAD_MODE_SIZE 8  //When sadMode is 0, it means the size of the detection area is 8*8, and this macro is defined as 8;
#define DEBUG_PRINT
extern struct chn_conf chn[];
/**
 * @defgroup IMP_IVS
 * @ingroup imp
 * @brief IVS intelligent analysis common interface API(The following debugging content can be viewed under Ingenic-SDK-T××/include_cn/imp/imp_ivs_base_move.h)
 * @section concept 1  related concept
 * IMP IVS calls the instantiated IMPIVSInterface throgh IVS Common Interface API to embed intelligent analysis algorithms into the SDK to analyze frame images in the SDK.
 * @subsection IMPIVSInterface 1.1 IMPIVSInterface
 * IMPIVSInterface is a generic algorithm interface,the specific algorithm implements this interface and passes it to IMP IVS to achieve the purpose of running the specific algorithm in the SDK.
 * A channel has and is only the carrier of a single algorithm instance. The general algorithm interface of the specific implementation must be passed to the specific channel in order to run the algorithm in the SDK
 * The IMPIVSInterface member param is an argument to the member function init。
 * IMP_IVS will externally lock the frame passed to the member function ProcessAsync parameter.
 * ProcessAsync must call IMP_IVS_ReleaseData to release the frame after it finished using frame to avoid deadlocks.
 * @section ivs_usage 2  usage guide
 * For a specific implementation of the function, see Sample file
 *
 * STEP.1 Initialize the system, you can call the sample_system_init() function in the sample directly.
 * The entire application can only initialize the system once, and if it has been initialized before, it does not need to be initialized here.
 * @code
 * ret = IMP_ISP_Open(); //Open the ISP module
 * ret = IMP_ISP_EnableTuning();	// Enable flipping, debug images
 *	ret = IMP_ISP_SetCameraInputMode(&mode) //If there are multiple sensors (maximum support of three cameras), please set to multiple cameras mode (if you are using single camera, ignore this)
 *	ret = IMP_ISP_AddSensor(IMPVI_MAIN, &sensor_info[*]) //Add sensor, Prior to this operation, the sensor driver had been added to the kernel (IMPVI_MAIN is the main camera, IMPVI_SEC is the second camera, IMPVI_THR is the third camera)
 *	ret = IMP_ISP_EnableSensor(IMPVI_MAIN, &sensor_info[*])	//Enable the sensor, now the sensor starts to output the image (IMPVI_MAIN is the main camera, IMPVI_SEC is the second camera, IMPVI_THR is the third camera)
 * @endcode
 *
 * STEP.2 Initialize framesource
 * If the framesource channel used by the algorithm has been created, the created channel can be directly used.
 * If the framesource channel used by the algorithm is not created, You can call the sample_framesource_init (IVS_FS_CHN, &fs_chn_attr) to create it.
 * @code
 * ret = IMP_FrameSource_CreateChn(chn[i].index, &chn[i].fs_chn_attr);	//Create channel
 * ret = IMP_FrameSource_SetChnAttr(chn[i].index, &chn[i].fs_chn_attr);	//Set channel properties
 * @endcode
 *
 * STEP.3 Encoder init  Encoding initialization. 
 * @code
 * ret = IMP_Encoder_CreateGroup(chn[i].index);	//Create a coding group
 * ret = sample_encoder_init();		//Encoding initialization
 * @endcode
 *
 * STEP.4 Create IVS specific algorithm channel group.
 * Multiple algorithms can share one channel group or use diffenrent channel groups separately, as described in sample_ivs_move_init()
 * @code
 * int sample_ivs_move_init(int grp_num)
 * {
 *  	int ret = 0;
 *
 *		ret = IMP_IVS_CreateGroup(grp_num);
 *		if (ret < 0) {
 *			IMP_LOG_ERR(TAG, "IMP_IVS_CreateGroup(%d) failed\n", grp_num);
 *			return -1;
 *		}
 *		return 0;
 * }
 * @endcode
 *
 * STEP.5 Bind algorithm channel groups and framesource channel groups
 * @code
 *	bind framesource channel.1-output.1 to ivs group
 *	fs.0 ----------------> encoder.0(main stream)
 *	fs.1 ----------------> ivs----> encoder.1(second stream)
 *
 *	IMPCell framesource_cell = {DEV_ID_FS, IVS_FS_CHN, 0};
 *	IMPCell ivs_cell = {DEV_ID_IVS, 0, 0};
 *	ret = IMP_System_Bind(&framesource_cell, &ivs_cell);
 *	if (ret < 0) {
 *		IMP_LOG_ERR(TAG, "Bind FrameSource channel%d and ivs0 failed\n", IVS_FS_CHN);
 *		return -1;
 *	}
 *	ret = IMP_System_Bind(&ivs_cell, &chn[i].imp_encoder);
 *	if (ret < 0) {
 *		IMP_LOG_ERR(TAG, "Bind FrameSource channel%d and Encoder failed\n",i);
 *		return -1;
 *	}
 * @endcode
 *
 * STEP.6 Start framesource。
 * @code
 *	IMP_FrameSource_SetFrameDepth(0, 0);	//Set the maximum image depth, this API is used to set the number of video image frames cached by a channel. 
 *  When user sets up caching of multi-frame video images, user can obtain a certain amount of continuous image data.
 *  If depth is set to 0, it means that the system does not need to cache images for the channel, so the user cannot obtain the image data of the channel. The system does not cache images for channels by default.
 *	ret = sample_framesource_streamon(IVS_FS_CHN);
 *	if (ret < 0) {
 *		IMP_LOG_ERR(TAG, "ImpStreamOn failed\n");
 *		return -1;
 *	}
 * @endcode
 *
 * STEP.7 Start the algorithm. It is recommended that the algorithm channel number and algorithm number be consistent so that you can directly correspond to which algorithm is currently operating.
 * @code
 *	ret = sample_ivs_move_start(0, 0, &inteface);
 *	if (ret < 0) {
 *		IMP_LOG_ERR(TAG, "sample_ivs_move_start(0, 0) failed\n");
 *		return -1;
 *	}
 * *interface = IMP_IVS_CreateBaseMoveInterface(&param);	//IMP_IVS_CreateBaseMoveInterface　Algorithm interface
 *  ret = IMP_IVS_CreateChn(chn_num, *interface);	//Create channel
 *  ret = IMP_IVS_RegisterChn(grp_num, chn_num);	//Register channel
 *  ret = IMP_IVS_StartRecvPic(chn_num);	//Start receiving pictures
 * @endcode
 *
 * STEP.8 Get the algorithm results
 * Polling results, acquisition results, and release results must correspond exactly, with no breaks in between;
 * Only if the Polling result is returned correctly, the obtained result will be updated, otherwise the obtained result cannot be predicted.
 * @code
 *	for (i = 0; i < NR_FRAMES_TO_IVS; i++) {
 *		ret = IMP_IVS_PollingResult(0, IMP_IVS_DEFAULT_TIMEOUTMS);
 *		if (ret < 0) {
 *			IMP_LOG_ERR(TAG, "IMP_IVS_PollingResult(%d, %d) failed\n", 0, IMP_IVS_DEFAULT_TIMEOUTMS);
 *			return -1;
 *		}
 *		ret = IMP_IVS_GetResult(0, (void **)&result);
 *		if (ret < 0) {
 *			IMP_LOG_ERR(TAG, "IMP_IVS_GetResult(%d) failed\n", 0);
 *			return -1;
 *		}
 *		IMP_LOG_INFO(TAG, "frame[%d], result->ret=%d\n", i, result->ret);
 *
 *		ret = IMP_IVS_ReleaseResult(0, (void *)result);
 *		if (ret < 0) {
 *			IMP_LOG_ERR(TAG, "IMP_IVS_ReleaseResult(%d) failed\n", 0);
 *			return -1;
 *		}
 *	}
 * @endcode
 *
 * STEP.9  get h264 stream
 * @code
 * Get the H264 video stream and generate an H264 video file
 * @encode
 *
 * STEP.10~17 Release resources, use them in the order corresponding to the sample code.
 * @code
 *　sample_ivs_move_get_result_stop(ivs_tid);
 *  sample_ivs_move_stop(2, inteface);
 *  sample_framesource_streamoff();
 *  IMP_System_UnBind(&framesource_cell, &ivs_cell);
 *  sample_ivs_move_exit(0);
 *  sample_framesource_exit(IVS_FS_CHN);
 *  sample_system_exit();
 * @endcode
 * @{
 */
static int sample_ivs_move_init(int grp_num)
{
	int ret = 0;
	ret = IMP_IVS_CreateGroup(grp_num);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_CreateGroup(%d) failed\n", grp_num);
		return -1;
	}
	return 0;
}
static int sample_ivs_move_exit(int grp_num)
{
	int ret = 0;
	ret = IMP_IVS_DestroyGroup(grp_num);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_DestroyGroup(%d) failed\n", grp_num);
		return -1;
	}
	return 0;
}
static int sample_ivs_move_start(int grp_num, int chn_num, IMPIVSInterface **interface)
{
	int ret = 0;
	IMP_IVS_BaseMoveParam param;
	memset(&param, 0, sizeof(IMP_IVS_BaseMoveParam));
	param.skipFrameCnt = 3;
	param.referenceNum = 4;
	param.sadMode = 0;
	param.sense = 3;
	param.frameInfo.width = FIRST_SENSOR_WIDTH_SECOND;
	param.frameInfo.height = FIRST_SENSOR_HEIGHT_SECOND;
	*interface = IMP_IVS_CreateBaseMoveInterface(&param);
	if (*interface == NULL) {
		IMP_LOG_ERR(TAG, "IMP_IVS_CreateGroup(%d) failed\n", grp_num);
		return -1;
	}
	ret = IMP_IVS_CreateChn(chn_num, *interface);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_CreateChn(%d) failed\n", chn_num);
		return -1;
	}
	ret = IMP_IVS_RegisterChn(grp_num, chn_num);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_RegisterChn(%d, %d) failed\n", grp_num, chn_num);
		return -1;
	}
	ret = IMP_IVS_StartRecvPic(chn_num);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_StartRecvPic(%d) failed\n", chn_num);
		return -1;
	}
	return 0;
}
static int sample_ivs_move_stop(int chn_num, IMPIVSInterface *interface)
{
	int ret = 0;
	ret = IMP_IVS_StopRecvPic(chn_num);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_StopRecvPic(%d) failed\n", chn_num);
		return -1;
	}
	sleep(1);
	ret = IMP_IVS_UnRegisterChn(chn_num);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_UnRegisterChn(%d) failed\n", chn_num);
		return -1;
	}
	ret = IMP_IVS_DestroyChn(chn_num);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_DestroyChn(%d) failed\n", chn_num);
		return -1;
	}
	IMP_IVS_DestroyBaseMoveInterface(interface);
	return 0;
}
#if 0
static int sample_ivs_set_sense(int chn_num, int sensor)
{
	int ret = 0;
	IMP_IVS_MoveParam param;
	int i = 0;
	ret = IMP_IVS_GetParam(chn_num, &param);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_GetParam(%d) failed\n", chn_num);
		return -1;
	}
	for( i = 0 ; i < param.roiRectCnt ; i++){
	  param.sense[i] = sensor;
	}
	ret = IMP_IVS_SetParam(chn_num, &param);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_SetParam(%d) failed\n", chn_num);
		return -1;
	}
	return 0;
}
#endif
static void *sample_ivs_move_get_result_process(void *arg)
{
	int i = 0, j = 0, ret = 0;
	int chn_num = (int)arg;
	IMP_IVS_BaseMoveOutput *result = NULL;
	for (i = 0; i < NR_FRAMES_TO_SAVE; i++) {
		ret = IMP_IVS_PollingResult(chn_num, IMP_IVS_DEFAULT_TIMEOUTMS);
		if (ret < 0) {
			IMP_LOG_ERR(TAG, "IMP_IVS_PollingResult(%d, %d) failed\n", chn_num, IMP_IVS_DEFAULT_TIMEOUTMS);
			return (void *)-1;
		}
		ret = IMP_IVS_GetResult(chn_num, (void **)&result);
		if (ret < 0) {
			IMP_LOG_ERR(TAG, "IMP_IVS_GetResult(%d) failed\n", chn_num);
			return (void *)-1;
		}
#ifdef DEBUG_PRINT
		for(j = 0; j < result->datalen; j ++) {
			printf("%4d ",*(result->data + j));
			if(j%(FIRST_SENSOR_WIDTH_SECOND/SAD_MODE_SIZE) == 0) printf("\n");
		}
#endif
		ret = IMP_IVS_ReleaseResult(chn_num, (void *)result);
		if (ret < 0) {
			IMP_LOG_ERR(TAG, "IMP_IVS_ReleaseResult(%d) failed\n", chn_num);
			return (void *)-1;
		}
#if 0
		if (i % 20 == 0) {
			ret = sample_ivs_set_sense(chn_num, i % 5);
			if (ret < 0) {
				IMP_LOG_ERR(TAG, "sample_ivs_set_sense(%d, %d) failed\n", chn_num, i % 5);
				return (void *)-1;
			}
		}
#endif
	}
	return (void *)0;
}
static int sample_ivs_move_get_result_start(int chn_num, pthread_t *ptid)
{
	if (pthread_create(ptid, NULL, sample_ivs_move_get_result_process, (void *)chn_num) < 0) {
		IMP_LOG_ERR(TAG, "create sample_ivs_move_get_result_process failed\n");
		return -1;
	}
	return 0;
}
static int sample_ivs_move_get_result_stop(pthread_t tid)
{
	pthread_join(tid, NULL);
	return 0;
}
int main(int argc, char *argv[])
{
	int i, ret;
	pthread_t ivs_tid;
	IMPIVSInterface *inteface = NULL;
	chn[0].enable = 0;
	chn[1].enable = 1;
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
		if (chn[i].enable) {
			ret = IMP_Encoder_CreateGroup(chn[i].index);
			if (ret < 0) {
				IMP_LOG_ERR(TAG, "IMP_Encoder_CreateGroup(%d) error !\n", i);
				return -1;
			}
		}
	}
	ret = sample_encoder_init();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "Encoder init failed\n");
		return -1;
	}
	/* Step.4 ivs init */
	ret = sample_ivs_move_init(0);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_ivs_move_init(0) failed\n");
		return -1;
	}
    /* step.5 bind */
	/* bind framesource channel.1-output.1 to ivs group */
	/**
	 * fs.0 ----------------> encoder.0(main stream)
	 * fs.1 ----------------> ivs----> encoder.1(second stream)
	 */
	IMPCell ivs_cell = {DEV_ID_IVS, 0, 0};
	IMPCell fs_for_ivs_cell = {DEV_ID_FS, 2, 2};
    for (i = 0; i < FS_CHN_NUM; i++) {
        if (IVS_CHN_ID == i) {
            if (chn[i].enable) {
                ret = IMP_System_Bind(&chn[i].framesource_chn, &ivs_cell);
                if (ret < 0) {
                    IMP_LOG_ERR(TAG, "Bind FrameSource channel.1 output.1 and ivs0 failed\n");
                    return -1;
                }
                ret = IMP_System_Bind(&ivs_cell, &chn[i].imp_encoder);
                if (ret < 0) {
                    IMP_LOG_ERR(TAG, "Bind FrameSource channel%d and Encoder failed\n",i);
                    return -1;
                }
            }
        } else {
            if (chn[i].enable) {
                ret = IMP_System_Bind(&chn[i].framesource_chn, &chn[i].imp_encoder);
                if (ret < 0) {
                    IMP_LOG_ERR(TAG, "Bind FrameSource channel%d and Encoder failed\n",i);
                    return -1;
                }
            }
        }
    }
    /* Step.6 framesource Stream On */
	ret = sample_framesource_streamon();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "ImpStreamOn failed\n");
		return -1;
	}
	/* Step.7 ivs move start */
	ret = sample_ivs_move_start(0, 2, &inteface);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_ivs_move_start(0, 0) failed\n");
		return -1;
	}
	/* Step.8 start to get ivs move result */
	ret = sample_ivs_move_get_result_start(2, &ivs_tid);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_ivs_move_get_result_start failed\n");
		return -1;
	}
	/* Step.9 get h264 stream */
	ret = sample_get_video_stream();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "Get H264 stream failed\n");
		return -1;
	}
	/* Exit sequence as follow */
	/* Step.10 stop to get ivs move result */
	ret = sample_ivs_move_get_result_stop(ivs_tid);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_ivs_move_get_result_stop failed\n");
		return -1;
	}
	/* Step.11 ivs move stop */
	ret = sample_ivs_move_stop(2, inteface);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_ivs_move_stop(0) failed\n");
		return -1;
	}
	/* Step.12 Stream Off */
	ret = sample_framesource_streamoff();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "FrameSource StreamOff failed\n");
		return -1;
	}
	/* Step.13 UnBind */
	ret = IMP_System_UnBind(&chn[IVS_CHN_ID].framesource_chn, &ivs_cell);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "UnBind FrameSource channel%d and ivs0 failed\n", IVS_CHN_ID);
		return -1;
	}
	for (i = 0; i < FS_CHN_NUM; i++) {
		if (chn[i].enable) {
			if(IVS_CHN_ID == i) {
				ret = IMP_System_UnBind(&ivs_cell, &chn[i].imp_encoder);
				if (ret < 0) {
					IMP_LOG_ERR(TAG, "UnBind FrameSource channel%d and Encoder failed\n",i);
					return -1;
				}
			}else{
			ret = IMP_System_UnBind(&chn[i].framesource_chn, &chn[i].imp_encoder);
			if (ret < 0) {
				IMP_LOG_ERR(TAG, "UnBind FrameSource channel%d and Encoder failed\n",i);
				return -1;
			}
			}
		}
	}
	/* Step.14 ivs exit */
	ret = sample_ivs_move_exit(0);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "ivs mode exit failed\n");
		return -1;
	}
	/* Step.15 Encoder exit */
	ret = sample_encoder_exit();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "Encoder exit failed\n");
		return -1;
	}
	/* Step.16 FrameSource exit */
	ret = sample_framesource_exit();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "FrameSource exit failed\n");
		return -1;
	}
	/* Step.17 System exit */
	ret = sample_system_exit();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_system_exit() failed\n");
		return -1;
	}
	return 0;
}
