LOCAL_PATH:= $(call my-dir)
 
include $(CLEAR_VARS)
LOCAL_SRC_FILES := uart-test.cpp
LOCAL_SHARED_LIBRARIES := liblog libm libc libprocessgroup libcutils libutils
LOCAL_CFLAGS := -Werror
LOCAL_MODULE := uart-test
LOCAL_C_INCLUDES := $(LOCAL_PATH)/
include $(BUILD_EXECUTABLE)
