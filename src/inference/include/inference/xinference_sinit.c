// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#include "xparameters.h"
#include "xinference.h"

extern XInference_Config XInference_ConfigTable[];

XInference_Config *XInference_LookupConfig(u16 DeviceId) {
	XInference_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XINFERENCE_NUM_INSTANCES; Index++) {
		if (XInference_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XInference_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XInference_Initialize(XInference *InstancePtr, u16 DeviceId) {
	XInference_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XInference_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XInference_CfgInitialize(InstancePtr, ConfigPtr);
}

#endif

