#pragma once

#include "../core/VoxelSpace.h"      

#ifdef __cplusplus
extern "C" {
#endif

	extern __declspec(dllexport) VoxelSpace* CreateVoxelSpace();
	extern __declspec(dllexport) void DisposeVoxelSpace(VoxelSpace* voxelSpace);

	extern __declspec(dllexport) void SetCellSize(VoxelSpace* voxelSpace, float cellSize, float cellHeight);
	extern __declspec(dllexport) float GetCellSize(VoxelSpace* voxelSpace);
	extern __declspec(dllexport) float GetCellHeight(VoxelSpace* voxelSpace);
	extern __declspec(dllexport) void CreateSpaceGrids(VoxelSpace* voxelSpace);
	extern __declspec(dllexport) void CreateVoxels(VoxelSpace* voxelSpace, SolidSpanGroup* solidSpanGroup);
	extern __declspec(dllexport) void FreeSolidSpanGridsMemory(VoxelSpace* voxelSpace);

	extern __declspec(dllexport) void TransModelVertexs(
		VoxelSpace* voxelSpace,
		SimpleVector3 vert0,
		SimpleVector3 vert1,
		SimpleVector3 vert2);

#ifdef __cplusplus
}
#endif