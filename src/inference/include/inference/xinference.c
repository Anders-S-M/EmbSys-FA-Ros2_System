// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "xinference.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XInference_CfgInitialize(XInference *InstancePtr, XInference_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XInference_Start(XInference *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XInference_ReadReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_AP_CTRL) & 0x80;
    XInference_WriteReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_AP_CTRL, Data | 0x01);
}

u32 XInference_IsDone(XInference *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XInference_ReadReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XInference_IsIdle(XInference *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XInference_ReadReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XInference_IsReady(XInference *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XInference_ReadReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XInference_EnableAutoRestart(XInference *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XInference_WriteReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_AP_CTRL, 0x80);
}

void XInference_DisableAutoRestart(XInference *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XInference_WriteReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_AP_CTRL, 0);
}

u32 XInference_Get_return(XInference *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XInference_ReadReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_AP_RETURN);
    return Data;
}
u32 XInference_Get_in_r_BaseAddress(XInference *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Control_BaseAddress + XINFERENCE_CONTROL_ADDR_IN_R_BASE);
}

u32 XInference_Get_in_r_HighAddress(XInference *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Control_BaseAddress + XINFERENCE_CONTROL_ADDR_IN_R_HIGH);
}

u32 XInference_Get_in_r_TotalBytes(XInference *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XINFERENCE_CONTROL_ADDR_IN_R_HIGH - XINFERENCE_CONTROL_ADDR_IN_R_BASE + 1);
}

u32 XInference_Get_in_r_BitWidth(XInference *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XINFERENCE_CONTROL_WIDTH_IN_R;
}

u32 XInference_Get_in_r_Depth(XInference *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XINFERENCE_CONTROL_DEPTH_IN_R;
}

u32 XInference_Write_in_r_Words(XInference *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XINFERENCE_CONTROL_ADDR_IN_R_HIGH - XINFERENCE_CONTROL_ADDR_IN_R_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(int *)(InstancePtr->Control_BaseAddress + XINFERENCE_CONTROL_ADDR_IN_R_BASE + (offset + i)*4) = *(data + i);
    }
    return length;
}

u32 XInference_Read_in_r_Words(XInference *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XINFERENCE_CONTROL_ADDR_IN_R_HIGH - XINFERENCE_CONTROL_ADDR_IN_R_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(int *)(InstancePtr->Control_BaseAddress + XINFERENCE_CONTROL_ADDR_IN_R_BASE + (offset + i)*4);
    }
    return length;
}

u32 XInference_Write_in_r_Bytes(XInference *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XINFERENCE_CONTROL_ADDR_IN_R_HIGH - XINFERENCE_CONTROL_ADDR_IN_R_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(char *)(InstancePtr->Control_BaseAddress + XINFERENCE_CONTROL_ADDR_IN_R_BASE + offset + i) = *(data + i);
    }
    return length;
}

u32 XInference_Read_in_r_Bytes(XInference *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XINFERENCE_CONTROL_ADDR_IN_R_HIGH - XINFERENCE_CONTROL_ADDR_IN_R_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(char *)(InstancePtr->Control_BaseAddress + XINFERENCE_CONTROL_ADDR_IN_R_BASE + offset + i);
    }
    return length;
}

void XInference_InterruptGlobalEnable(XInference *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XInference_WriteReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_GIE, 1);
}

void XInference_InterruptGlobalDisable(XInference *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XInference_WriteReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_GIE, 0);
}

void XInference_InterruptEnable(XInference *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XInference_ReadReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_IER);
    XInference_WriteReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_IER, Register | Mask);
}

void XInference_InterruptDisable(XInference *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XInference_ReadReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_IER);
    XInference_WriteReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_IER, Register & (~Mask));
}

void XInference_InterruptClear(XInference *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XInference_WriteReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_ISR, Mask);
}

u32 XInference_InterruptGetEnabled(XInference *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XInference_ReadReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_IER);
}

u32 XInference_InterruptGetStatus(XInference *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XInference_ReadReg(InstancePtr->Control_BaseAddress, XINFERENCE_CONTROL_ADDR_ISR);
}

