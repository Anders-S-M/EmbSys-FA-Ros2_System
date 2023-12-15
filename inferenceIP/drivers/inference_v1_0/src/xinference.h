// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef XINFERENCE_H
#define XINFERENCE_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif
#include "xinference_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
    u16 DeviceId;
    u32 Control_BaseAddress;
} XInference_Config;
#endif

typedef struct {
    u64 Control_BaseAddress;
    u32 IsReady;
} XInference;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XInference_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XInference_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XInference_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XInference_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
int XInference_Initialize(XInference *InstancePtr, u16 DeviceId);
XInference_Config* XInference_LookupConfig(u16 DeviceId);
int XInference_CfgInitialize(XInference *InstancePtr, XInference_Config *ConfigPtr);
#else
int XInference_Initialize(XInference *InstancePtr, const char* InstanceName);
int XInference_Release(XInference *InstancePtr);
#endif

void XInference_Start(XInference *InstancePtr);
u32 XInference_IsDone(XInference *InstancePtr);
u32 XInference_IsIdle(XInference *InstancePtr);
u32 XInference_IsReady(XInference *InstancePtr);
void XInference_EnableAutoRestart(XInference *InstancePtr);
void XInference_DisableAutoRestart(XInference *InstancePtr);
u32 XInference_Get_return(XInference *InstancePtr);

u32 XInference_Get_in_r_BaseAddress(XInference *InstancePtr);
u32 XInference_Get_in_r_HighAddress(XInference *InstancePtr);
u32 XInference_Get_in_r_TotalBytes(XInference *InstancePtr);
u32 XInference_Get_in_r_BitWidth(XInference *InstancePtr);
u32 XInference_Get_in_r_Depth(XInference *InstancePtr);
u32 XInference_Write_in_r_Words(XInference *InstancePtr, int offset, word_type *data, int length);
u32 XInference_Read_in_r_Words(XInference *InstancePtr, int offset, word_type *data, int length);
u32 XInference_Write_in_r_Bytes(XInference *InstancePtr, int offset, char *data, int length);
u32 XInference_Read_in_r_Bytes(XInference *InstancePtr, int offset, char *data, int length);

void XInference_InterruptGlobalEnable(XInference *InstancePtr);
void XInference_InterruptGlobalDisable(XInference *InstancePtr);
void XInference_InterruptEnable(XInference *InstancePtr, u32 Mask);
void XInference_InterruptDisable(XInference *InstancePtr, u32 Mask);
void XInference_InterruptClear(XInference *InstancePtr, u32 Mask);
u32 XInference_InterruptGetEnabled(XInference *InstancePtr);
u32 XInference_InterruptGetStatus(XInference *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
