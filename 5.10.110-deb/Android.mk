LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE := diaggrabpro
LOCAL_SRC_FILES := diaggrabpro.c ftp_client.c mdm.c
LOCAL_MODULE_TAGS := debug
include $(BUILD_EXECUTABLE) 
