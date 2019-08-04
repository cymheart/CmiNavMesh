#pragma once
#include"VoxelSpace.h"

/// <summary>	
/// ÊµÐÄ¿ç¾à×é	
/// </summary>
class MinNavDLL_API SolidSpanGroup
{
private:
	VoxelSpace* voxSpace;
	SolidSpanList* solidSpanGrids;
	int gridCount = 0;

public:

	SolidSpanGroup(VoxelSpace* voxSpace);
	~SolidSpanGroup();

	SolidSpanList* GetSolidSpanGrids()
	{
		return solidSpanGrids;
	}

	int GetGridCount()
	{
		return gridCount;
	}
	
protected:
	void AppendVoxBox(
		int floorCellIdxX, int floorCellIdxZ,
		int heightCellStartIdx, int heightCellEndIdx);

	void AppendVoxBoxToSpanHeightList(
		SolidSpanList* solidSpanList,
		int heightCellStartIdx, int heightCellEndIdx);

	friend class VoxelTriangle;
};