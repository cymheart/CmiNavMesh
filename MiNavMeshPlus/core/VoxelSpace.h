#pragma once
#include"MiNavTypes.h"
#include<list>
using namespace std;
struct SolidSpanList;
struct SolidSpan;
class SolidSpanGroup;


class MinNavDLL_API VoxelSpace
{
protected:
	float cellSize = 0.1;
	float cellHeight = 0.1;
	float invCellSize;
	float invCellHeight;

	SolidSpanList* solidSpanGrids;
	SolidSpanList* spaceSpanGrids;
	float* cellxList;
	float* cellzList;

	list<SolidSpan*> solidSpansList;
	int freeSolidSpanCount = 0;
	const int preCreateSolidSpanCount = 100000;

	//
	list<TriVertsInfo*> triVertInfoList;
	int freeVertsCount = 0;
	const int preCreateVertsCount = 10000;


	MiNavAABB spaceAABB = {1,  -1 };
	int xstartCell, xendCell;
	int zstartCell, zendCell;
	int cellxCount;
	int cellzCount;

	int gridCount;

public:
	VoxelSpace();
	~VoxelSpace();

	void SetCellSize(float cellSize, float cellHeight)
	{
		this->cellSize = cellSize;
		this->cellHeight = cellHeight;
		invCellSize = 1 / cellSize;
		invCellHeight = 1 / cellHeight;
	}


	float GetCellSize()
	{
		return cellSize;
	}

	float GetCellHeight()
	{
		return cellHeight;
	}

	void CreateSpaceGrids();
	void CreateVoxels(SolidSpanGroup* solidSpanGroup);
	void FreeSolidSpanGridsMemory();
	void TransModelVertexs(SimpleVector3* triFaceVertex);

protected:
	int GetFloorGridIdx(int x, int z);
	SolidSpan* GetSoildSpan();

private:

	TriVertsInfo* GetTriInfo();
	void CalFloorGridIdxRange();
	void CreateSoildSpanSpaceGrids();

	friend class VoxelTriangle;
	friend class SolidSpanGroup;;
};