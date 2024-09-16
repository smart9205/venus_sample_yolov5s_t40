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
#define TAG "Sample-IVS-unbind-move"
extern struct chn_conf chn[];

/**
 * @defgroup IMP_IVS
 * @ingroup imp
 * @brief IVS intelligent analysis common interface API(The following debugging content can be viewed under Ingenic-SDK-T××/include_cn/imp/imp_ivs_move.h)
 * @section concept 1 related concept
 * IMP IVS Call the instantiated IMPIVSInterface through the IVS Common Interface API to embed an intelligent analysis algorithm into the SDK to analyze the frame image in the SDK.
 * @subsection IMPIVSInterface 1.1 IMPIVSInterface
 * IMPIVSInterface It is a general algorithm interface, and the specific algorithm implements this interface and transmits it to IMP IVS to achieve the purpose of running specific algorithms in the SDK.
 * IMPIVSInterface The member param is an argument to the member function init.
 * @section ivs_usage 2 usage
 * For a specific implementation of the function, see Sample file
 *
 * STEP.1 Initialize the system, you can call the sample_system_init() function in the sample directly.
 * The entire application can only initialize the system once, and if it has been initialized before, it does not need to be initialized here.
 * @code
 * ret = IMP_ISP_Open(); //Open isp module
 * ret = IMP_ISP_EnableTuning();	// Enable flipping, debug images
 *	ret = IMP_ISP_SetCameraInputMode(&mode) //If there are multiple sensors (maximum support of three cameras), set the mode to multiple cameras (please ignore if you're single camera)
 *	ret = IMP_ISP_AddSensor(IMPVI_MAIN, &sensor_info[*]) //Add sensor, before this operation the sensor driver has been added to the kernel (IMPVI_MAIN is main camera, IMPVI_SEC is second camera, IMPVI_THR is third camera)
 * @endcode
 *
 * STEP.2 Initialize the framesource
 * If the framesource channel used by the algorithm has been created, you can directly use the created channel.
 * If the framesource channel used by the algorithm is not created, sample_framesource_init (IVS_FS_CHN, &fs_chn_attr) in the column can be called to create it.
 * @code
 * ret = IMP_FrameSource_CreateChn(chn[i].index, &chn[i].fs_chn_attr);	//Create channel
 * ret = IMP_FrameSource_SetChnAttr(chn[i].index, &chn[i].fs_chn_attr);	//Set channel attribute
 * @endcode
 *
 * STEP.3 Start framesource.
 * @code
 *	IMP_FrameSource_SetFrameDepth(0, 0);	//Set the maximum image depth, this API is used to set the number of video image frames cached by a channel.
 *  When a user sets up caching of multi-frame video images, the user can obtain a certain amount of continuous image data.
 *  If you specify depth as 0, it means that the system does not need to cache images for this channel, 
 *  Therefore, the user does not get the channel image data system. The image is not cached by the channel, that is, the depth defaults to 0
 *	ret = sample_framesource_streamon(IVS_FS_CHN);
 *	if (ret < 0) {
 *		IMP_LOG_ERR(TAG, "ImpStreamOn failed\n");
 *		return -1;
 *	}
 * @endcode
 *
 * STEP.4 Get the algorithm results
 * Polling results, acquisition results, and release results must correspond strictly, without interruptions in between;
 * Only if the Polling result is returned correctly, the obtained result will be updated, otherwise the obtained result cannot be predicted.
 *
 * STEP.6~9 Release resources，use them in the order corresponding to the sample code.
 * @code
 *  sample_ivs_move_stop(2, inteface);
 *  sample_framesource_streamoff();
 *  sample_framesource_exit(IVS_FS_CHN);
 *  sample_system_exit();
 * @endcode
 */
static int sample_ivs_move_start(int grp_num, int chn_num, IMPIVSInterface **interface)
{
	int ret = 0;
	IMP_IVS_MoveParam param;
	int i = 0, j = 0;
	memset(&param, 0, sizeof(IMP_IVS_MoveParam));
	param.skipFrameCnt = 1;
	param.frameInfo.width = FIRST_SENSOR_WIDTH_SECOND;
	param.frameInfo.height = FIRST_SENSOR_HEIGHT_SECOND;
	param.roiRectCnt = 1;
	for(i=0; i<param.roiRectCnt; i++){
	  param.sense[i] = 4;
	}
	/* printf("param.sense=%d, param.skipFrameCnt=%d, param.frameInfo.width=%d, param.frameInfo.height=%d\n", param.sense, param.skipFrameCnt, param.frameInfo.width, param.frameInfo.height); */
	for (j = 0; j < 2; j++) {
		for (i = 0; i < 2; i++) {
		  if((i==0)&&(j==0)){
			param.roiRect[j * 2 + i].p0.x = i * param.frameInfo.width /* / 2 */;
			param.roiRect[j * 2 + i].p0.y = j * param.frameInfo.height /* / 2 */;
			param.roiRect[j * 2 + i].p1.x = (i + 1) * param.frameInfo.width /* / 2 */ - 1;
			param.roiRect[j * 2 + i].p1.y = (j + 1) * param.frameInfo.height /* / 2 */ - 1;
			printf("(%d,%d) = ((%d,%d)-(%d,%d))\n", i, j, param.roiRect[j * 2 + i].p0.x, param.roiRect[j * 2 + i].p0.y,param.roiRect[j * 2 + i].p1.x, param.roiRect[j * 2 + i].p1.y);
		  }
		  else
		    {
		      	param.roiRect[j * 2 + i].p0.x = param.roiRect[0].p0.x;
			param.roiRect[j * 2 + i].p0.y = param.roiRect[0].p0.y;
			param.roiRect[j * 2 + i].p1.x = param.roiRect[0].p1.x;;
			param.roiRect[j * 2 + i].p1.y = param.roiRect[0].p1.y;;
			printf("(%d,%d) = ((%d,%d)-(%d,%d))\n", i, j, param.roiRect[j * 2 + i].p0.x, param.roiRect[j * 2 + i].p0.y,param.roiRect[j * 2 + i].p1.x, param.roiRect[j * 2 + i].p1.y);
		    }
		}
	}
	*interface = IMP_IVS_CreateMoveInterface(&param);
	if (*interface == NULL) {
		IMP_LOG_ERR(TAG, "IMP_IVS_CreateGroup(%d) failed\n", grp_num);
		return -1;
	}
	return 0;
}
static int sample_ivs_move_stop(int chn_num, IMPIVSInterface *interface)
{
	IMP_IVS_DestroyMoveInterface(interface);
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
	ret = sample_ivs_move_start(0, 2, &interface);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_ivs_move_start(0, 0) failed\n");
		return -1;
	}
	if(interface->init && ((ret = interface->init(interface)) < 0)) {
		IMP_LOG_ERR(TAG, "interface->init failed, ret=%d\n", ret);
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
		if (interface->preProcessSync && ((ret = interface->preProcessSync(interface, &frame)) < 0)) {
			IMP_LOG_ERR(TAG, "interface->preProcessSync failed,ret=%d\n", ret);
			return -1;
		}
		if (interface->processAsync && ((ret = interface->processAsync(interface, &frame)) < 0)) {
			IMP_LOG_ERR(TAG, "interface->processAsync failed,ret=%d\n", ret);
			return -1;
		}
		if (interface->getResult && ((ret = interface->getResult(interface, (void **)&result)) < 0)) {
			IMP_LOG_ERR(TAG, "interface->getResult failed,ret=%d\n", ret);
			return -1;
		}
		IMP_LOG_INFO(TAG, "frame[%d], result->retRoi(%d,%d,%d,%d)\n", i, result->retRoi[0], result->retRoi[1], result->retRoi[2], result->retRoi[3]);
		//release moveresult
		if (interface->releaseResult && ((ret = interface->releaseResult(interface, (void *)result)) < 0)) {
		IMP_LOG_ERR(TAG, "interface->releaseResult failed ret=%d\n", ret);
			return -1;
		}
	}
	if(interface->exit < 0) {
		IMP_LOG_ERR(TAG, "interface->init failed, ret=%d\n", ret);
		return -1;
	}
	free(g_sub_nv12_buf_move);
	/* Step.6 ivs move stop */
	ret = sample_ivs_move_stop(2, interface);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_ivs_move_stop(0) failed\n");
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
