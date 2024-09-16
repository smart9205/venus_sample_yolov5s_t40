#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>

#include "sample-common.h"

int main(int argc, char *argv[])
{
	int ret = 0;
	int i = 0, frmSize = 0;
	int Src_width = 1920, Src_height = 1080;
    int Enc_width = 2560, Enc_height = 1440;
	int encNum = 50;
	int out = -1;
	char path[32];
	FILE *inFile = NULL;
	uint8_t *src_buf = NULL;
	void *h = NULL;
	IMPEncoderYuvOut stream;
	IMPEncoderYuvIn info;

	memset(&info, 0, sizeof(IMPEncoderYuvIn));

	info.type = IMP_ENC_TYPE_HEVC;
	info.mode = IMP_ENC_RC_MODE_CAPPED_QUALITY;
	//info.type = IMP_ENC_TYPE_JPEG;
	//info.mode = IMP_ENC_RC_MODE_FIXQP;
	info.frameRate = 25;
	info.gopLength = 25;
	info.targetBitrate = 4096;
	info.maxBitrate = 4096 * 4 / 3;
	info.initQp = 25;
	info.minQp = 15;
	info.maxQp = 48;
	info.maxPictureSize = info.maxBitrate;

	ret = IMP_System_Init();
	if (ret < 0){
		printf("IMP_System_Init failed\n");
		return -1;
	}

	inFile = fopen("1920x1080.nv12", "rb");
	if (inFile == NULL) {
		printf("fopen src file:%s failed\n", "1920x1080.nv12");
		return -1;
	}
	if (info.type == IMP_ENC_TYPE_HEVC)
		strcpy(path, "out.h265");
	else if (info.type == IMP_ENC_TYPE_AVC)
		strcpy(path, "out.h264");

	if (info.type != IMP_ENC_TYPE_JPEG) {
		out = open(path, O_RDWR | O_CREAT | O_TRUNC, 0644);
		if (out < 0) {
			printf("open out file:%s failed\n", path);
			return -1;
		}
	}
	ret = IMP_Encoder_YuvInit(&h, Enc_width, Enc_height, (void *)&info);
	if ((ret < 0) || (h == NULL)) {
		printf("IMP_Encoder_YuvInit failed\n");
		return -1;
	}

	frmSize = Src_width * Src_height * 3 / 2;
	src_buf = (uint8_t*)malloc(frmSize);
	if(src_buf == NULL) {
		printf("malloc src_buf failed\n");
		return -1;
	}

	stream.outAddr = (uint8_t *)malloc(frmSize);
	if(stream.outAddr == NULL) {
		printf("steamAddr malloc failed\n");
		return -1;
	}

	IMPProcessYuvInfo frame;
	memset(&frame, 0, sizeof(IMPProcessYuvInfo));
	
	frame.srcframe_width = Src_width;
	frame.srcframe_height = Src_height;
	frame.srcframe_virAddr = src_buf;

	frame.crop_enable = 1;
	frame.crop_left = 0;
	frame.crop_top = 0;
	frame.crop_width = 1280;
	frame.crop_height = 720;

	frame.scaler_enable = 1;
	frame.scaler_width = Enc_width;
	frame.scaler_height = Enc_height;

	frame.dstframe_width = Enc_width;
	frame.dstframe_height = Enc_height;
	frame.dstframe_virAddr = (uint32_t)IMP_Encoder_VbmAlloc(frmSize, 256);
	frame.dstframe_phyAddr = (uint32_t)IMP_Encoder_VbmV2P((intptr_t)frame.dstframe_virAddr);

	if (frame.resize_tmp_buf == NULL){
		int max_width = Src_width > frame.scaler_width ? Src_width : frame.scaler_width;
		int max_height = Src_height > frame.scaler_height ? Src_height : frame.scaler_height;
		if((frame.resize_tmp_buf = (uint16_t*)malloc(sizeof(uint16_t)*(max_width * 10 + max_height * 12 + 2592))) == NULL) {
			printf("malloc resize_tmp_buf failed\n");
			return -1;
		}
	}

	for (i = 0; i < encNum; i++) {
		if (info.type == IMP_ENC_TYPE_JPEG) {
			sprintf(path, "out_%d.jpeg", i);
			out = open(path, O_RDWR | O_CREAT | O_TRUNC, 0644);
			if (out < 0) {
				printf("open out file:%s failed\n", path);
				return -1;
			}
		}
		if(fread(src_buf, frmSize, 1, inFile) != frmSize) {
			fseek(inFile, 0, SEEK_SET);
			fread(src_buf, 1, frmSize, inFile);
		}
		ret = IMP_Encoder_YuvprocessEncode(h, frame, &stream);
		if (ret < 0) {
			printf("IMP_Encoder_YuvprocessEncode failed\n");
			return -1;
		}
		printf("\r%d encode success", i);
		fflush(stdout);
		if (stream.outLen != write(out, stream.outAddr, stream.outLen)) {
			printf("stream write failed\n");
			return -1;
		}
		if (info.type == IMP_ENC_TYPE_JPEG)
			close(out);
	}
	puts("");

	free(src_buf);
	free(stream.outAddr);
	free(frame.resize_tmp_buf);
	IMP_Encoder_VbmFree(frame.dstframe_virAddr);
	IMP_Encoder_YuvExit(h);
	if (info.type != IMP_ENC_TYPE_JPEG)
		close(out);
	fclose(inFile);
	IMP_System_Exit();

	return 0;
}
