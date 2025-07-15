LOCAL_PATH:= $(call my-dir)
 
include $(CLEAR_VARS)
LOCAL_SRC_FILES := libuart.cpp utils.cpp
LOCAL_CFLAGS := -Werror -DDEBUG=1
LOCAL_MODULE := libuart
LOCAL_C_INCLUDES := $(LOCAL_PATH)/
include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := libbquart.cpp
LOCAL_SHARED_LIBRARIES := liblog libm libc libprocessgroup libcutils libutils libuart
LOCAL_CFLAGS := -Werror -DDEBUG=1
LOCAL_MODULE := libbquart
LOCAL_C_INCLUDES := $(LOCAL_PATH)/
include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := uart-test.cpp
LOCAL_SHARED_LIBRARIES := liblog libm libc libprocessgroup libcutils libutils libuart
LOCAL_CFLAGS := -Werror -DDEBUG=1
LOCAL_CPPFLAGS += -fexceptions
LOCAL_MODULE := uart-test
LOCAL_C_INCLUDES := $(LOCAL_PATH)/
include $(BUILD_EXECUTABLE)
