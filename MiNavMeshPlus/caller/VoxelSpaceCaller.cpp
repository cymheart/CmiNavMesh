#include "../caller/VoxelSpaceCaller.h"


VoxelSpace* CreateVoxelSpace()
{
	return new VoxelSpace();
}

void DisposeVoxelSpace(VoxelSpace* voxelSpace)
{
	if (voxelSpace != NULL)
	{
		delete voxelSpace;
		voxelSpace = NULL;
	}
}

void SetCellSize(VoxelSpace* voxelSpace, float cellSize, float cellHeight)
{
	voxelSpace->SetCellSize(cellSize, cellHeight);
}

float GetCellSize(VoxelSpace* voxelSpace)
{
	return voxelSpace->GetCellSize();
}

float GetCellHeight(VoxelSpace* voxelSpace)
{
	return voxelSpace->GetCellHeight();
}

void CreateSpaceGrids(VoxelSpace* voxelSpace)
{
	voxelSpace->CreateSpaceGrids();
}

void CreateVoxels(VoxelSpace* voxelSpace, SolidSpanGroup* solidSpanGroup)
{
	voxelSpace->CreateVoxels(solidSpanGroup);
}

void FreeSolidSpanGridsMemory(VoxelSpace* voxelSpace)
{
	voxelSpace->FreeSolidSpanGridsMemory();
}

void TransModelVertexs(
	VoxelSpace* voxelSpace, 
	SimpleVector3 vert0,
	SimpleVector3 vert1,
	SimpleVector3 vert2)
{
	SimpleVector3 verts[3] = { vert0, vert1, vert2 };
	voxelSpace->TransModelVertexs(verts);
}