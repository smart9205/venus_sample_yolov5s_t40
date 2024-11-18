TOPDIR ?= ./2.29/
IMP_DIR ?= ./imp-t40
CROSS_COMPILE:=mips-linux-gnu-

CC = $(CROSS_COMPILE)gcc
CPP = $(CROSS_COMPILE)g++
STRIP = $(CROSS_COMPILE)strip
libtype ?= muclibc
build_type ?= release
#build_type ?= profile
#build_type ?= debug
#build_type ?= nmem

CFLAGS := -std=c++11 -mfp64 -mnan=2008 -mabs=2008 -Wall -EL -O3 -march=mips32r2 -flax-vector-conversions -lpthread -lrt -ldl -lm
CXXFLAGS := -std=c++11 -mfp64 -mnan=2008 -mabs=2008 -Wall -EL -O3 -march=mips32r2 -flax-vector-conversions -lpthread -lrt -ldl -lm
INCLUDES := -I$(TOPDIR)/include
INCLUDES += -I$(IMP_DIR)/include
INCLUDES += -I$(IMP_DIR)/samples/libimp-samples

ifeq ($(libtype), muclibc)
	CXXFLAGS += -muclibc
endif

ifeq ($(build_type), release)
    ifeq ($(libtype), muclibc)
        LIBS := -L$(TOPDIR)/lib/uclibc -lvenus
    else
        LIBS := -L$(TOPDIR)/lib/glibc -lvenus
    endif

    ifeq ($(libtype), muclibc)
        TARGET = venus_yolov5s_bin_uclibc_release
    else
        TARGET = venus_yolov5s_bin_glibc_release
    endif

else ifeq ($(build_type), profile)
    CXXFLAGS += -DVENUS_PROFILE
    ifeq ($(libtype), muclibc)
        LIBS := -L$(TOPDIR)/lib/uclibc/ -lvenus.p
    else
        LIBS := -L$(TOPDIR)/lib/glibc/ -lvenus.p
    endif

    ifeq ($(libtype), muclibc)
        TARGET = venus_yolov5s_bin_uclibc_profile
    else
        TARGET = venus_yolov5s_bin_glibc_prolfile
    endif

else ifeq ($(build_type), debug)
    CXXFLAGS += -DVENUS_DEBUG
    ifeq ($(libtype), muclibc)
        LIBS := -L$(TOPDIR)/lib/uclibc/  -lvenus.d
    else
        LIBS := -L$(TOPDIR)/lib/glibc/  -lvenus.d
    endif

    ifeq ($(libtype), muclibc)
        TARGET = venus_yolov5s_bin_uclibc_debug
    else
        TARGET = venus_yolov5s_bin_glibc_debug
    endif

else ifeq ($(build_type), nmem)
    ifeq ($(libtype), muclibc)
        LIBS := -L$(TOPDIR)/lib/uclibc/ -lvenus.m
    else
        LIBS := -L$(TOPDIR)/lib/glibc/ -lvenus.m 
    endif

    ifeq ($(libtype), muclibc)
        TARGET = venus_yolov5s_bin_uclibc_nmem
    else
        TARGET = venus_yolov5s_bin_glibc_nmem
    endif

endif

LIBS += ${IMP_DIR}/lib/uclibc/libalog.a ${IMP_DIR}/lib/uclibc/libimp.a

OBJS := inference_nv12.o sample-Magik.o ${IMP_DIR}/samples/libimp-samples/sample-common.o
# OBJS := inference.o



%.o:%.cpp
	$(CPP) $(INCLUDES) $(CXXFLAGS) -o $@ -c $^
%.o:%.c
	$(CC) $(INCLUDES) $(CFLAGS) -o $@ -c $^

$(TARGET):$(OBJS)
	$(CPP) $(CXXFLAGS) $(OBJS) -o $@ $(INCLUDES) $(LIBS)

all:$(TARGET)

release:
	mkdir execuate_file
	cp $(TOPDIR)/lib/uclibc/libvenus.so execuate_file/
	cp $(TARGET) execuate_file/
	#cp venus_yolov5s_bin_uclibc_release execuate_file/
	cp yolov5s_t40_magik.bin execuate_file/

.PHONY: clean
clean: 
	rm -f $(TARGET) $(OBJS)
	rm execuate_file -r
