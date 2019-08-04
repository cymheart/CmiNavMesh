#pragma once

#include "../core/SolidSpanGroup.h"      

#ifdef __cplusplus
extern "C" {
#endif

	extern __declspec(dllexport) SolidSpanGroup* CreateSolidSpanGroup(VoxelSpace* voxSpace);
	extern __declspec(dllexport) void DisposeSolidSpanGroup(SolidSpanGroup* solidSpanGroup);
	extern __declspec(dllexport) SolidSpanList* GetSolidSpanGrids(SolidSpanGroup* solidSpanGroup);
	extern __declspec(dllexport) int GetGridCount(SolidSpanGroup* solidSpanGroup);

#ifdef __cplusplus
}
#endif