ifeq ($(TARGET_DATARMNET_EXT_ENABLE), true)
ifneq ($(TARGET_BOARD_AUTO),true)
ifneq ($(TARGET_BOARD_PLATFORM),qssi)

RMNET_MEM_DLKM_PLATFORMS_LIST := pineapple
RMNET_MEM_DLKM_PLATFORMS_LIST += blair
RMNET_MEM_DLKM_PLATFORMS_LIST += monaco
RMNET_MEM_DLKM_PLATFORMS_LIST += pitti
RMNET_MEM_DLKM_PLATFORMS_LIST += volcano

ifeq ($(call is-board-platform-in-list, $(RMNET_MEM_DLKM_PLATFORMS_LIST)),true)
LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

#Enabling BAZEL
LOCAL_MODULE_DDK_BUILD := true

LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
LOCAL_CLANG :=true
LOCAL_MODULE := rmnet_mem.ko
LOCAL_SRC_FILES := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)

BOARD_COMMON_DIR ?= device/qcom/common

DLKM_DIR := $(TOP)/$(BOARD_COMMON_DIR)/dlkm

include $(DLKM_DIR)/Build_external_kernelmodule.mk

endif #End of check for target
endif #End of Check for qssi target
endif #End of check for AUTO Target
endif #End of Check for datarmnet
