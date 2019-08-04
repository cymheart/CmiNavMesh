#include"VoxelSpace.h"
#include"VoxelTriangle.h"
#include"SolidSpanGroup.h"

VoxelSpace::VoxelSpace()
{
	invCellSize = 1 / cellSize;
	invCellHeight = 1 / cellHeight;
}

VoxelSpace::~VoxelSpace()
{
	FreeSolidSpanGridsMemory();
}
	
void VoxelSpace::CreateSpaceGrids()	
{		
	CalFloorGridIdxRange();		
	CreateSoildSpanSpaceGrids();	
}

	void VoxelSpace::CreateVoxels(SolidSpanGroup* solidSpanGroup)
	{
		VoxelTriangle voxTri = VoxelTriangle(this, solidSpanGroup);
		TriVertsInfo* info;
		int count;

		list<TriVertsInfo*>::iterator i = triVertInfoList.begin();
		for (; i != triVertInfoList.end(); i++)
		{
			info = (TriVertsInfo*)*i;

			if (info == triVertInfoList.back())
				count = preCreateVertsCount - freeVertsCount;
			else
				count = preCreateVertsCount;

			for (int i = 0; i < count; i++)
			{
				voxTri.CreateVoxels(info + i);
			}
		}
	}

	void VoxelSpace::FreeSolidSpanGridsMemory()
	{
		list<SolidSpan*>::iterator i = solidSpansList.begin();
		for (; i != solidSpansList.end(); i++)
		{
			free(*i);
		}
		solidSpansList.clear();

		list<TriVertsInfo*>::iterator i2 = triVertInfoList.begin();
		for (; i2 != triVertInfoList.end(); i2++)
		{
			free(*i2);
		}
		triVertInfoList.clear();

		free(solidSpanGrids);
		solidSpanGrids = nullptr;
	}

	int VoxelSpace::GetFloorGridIdx(int x, int z)
	{
		return (z - zstartCell) * cellxCount + x - xstartCell;
	}

	SolidSpan* VoxelSpace::GetSoildSpan()
	{
		if (freeSolidSpanCount == 0)
		{
			SolidSpan* solidSpans = (SolidSpan*)malloc(sizeof(SolidSpan) * preCreateSolidSpanCount);
			memset(solidSpans, 0, sizeof(SolidSpan) * preCreateSolidSpanCount);
			solidSpansList.push_back(solidSpans);
			freeSolidSpanCount = preCreateSolidSpanCount;
		}

		SolidSpan* solidSpanPtr = solidSpansList.back();
		solidSpanPtr += preCreateSolidSpanCount - freeSolidSpanCount;
		freeSolidSpanCount--;
		return solidSpanPtr;
	}

	void VoxelSpace::TransModelVertexs(SimpleVector3* triFaceVertex)
	{
		TriVertsInfo* triInfo = GetTriInfo();

		MiNavAABB* aabb = &(triInfo->aabb);

		triInfo->vert0 = triFaceVertex[0];
		aabb->maxX = triInfo->vert0.x; aabb->minX = triInfo->vert0.x;
		aabb->maxZ = triInfo->vert0.z; aabb->minZ = triInfo->vert0.z;
		aabb->maxY = triInfo->vert0.y; aabb->minY = triInfo->vert0.y;


		triInfo->vert1 = triFaceVertex[1];
		if (triInfo->vert1.x > aabb->maxX) { aabb->maxX = triInfo->vert1.x; }
		if (triInfo->vert1.x < aabb->minX) { aabb->minX = triInfo->vert1.x; }
		if (triInfo->vert1.z > aabb->maxZ) { aabb->maxZ = triInfo->vert1.z; }
		if (triInfo->vert1.z < aabb->minZ) { aabb->minZ = triInfo->vert1.z; }

		triInfo->vert2 = triFaceVertex[2];
		if (triInfo->vert2.x > aabb->maxX) { aabb->maxX = triInfo->vert2.x; }
		if (triInfo->vert2.x < aabb->minX) { aabb->minX = triInfo->vert2.x; }
		if (triInfo->vert2.z > aabb->maxZ) { aabb->maxZ = triInfo->vert2.z; }
		if (triInfo->vert2.z < aabb->minZ) { aabb->minZ = triInfo->vert2.z; }

		if (spaceAABB.maxX < spaceAABB.minX)
		{
			spaceAABB.maxX = aabb->maxX;
			spaceAABB.minX = aabb->minX;
			spaceAABB.maxZ = aabb->maxZ;
			spaceAABB.minZ = aabb->minZ;
		}
		else
		{
			if (aabb->maxX > spaceAABB.maxX) { spaceAABB.maxX = aabb->maxX; }
			if (aabb->minX < spaceAABB.minX) { spaceAABB.minX = aabb->minX; }
			if (aabb->maxZ > spaceAABB.maxZ) { spaceAABB.maxZ = aabb->maxZ; }
			if (aabb->minZ < spaceAABB.minZ) { spaceAABB.minZ = aabb->minZ; }
		}

	}

	TriVertsInfo* VoxelSpace::GetTriInfo()
	{
		if (freeVertsCount == 0)
		{
			TriVertsInfo* vertsInfo = (TriVertsInfo*)malloc(sizeof(TriVertsInfo) * preCreateVertsCount);
			triVertInfoList.push_back(vertsInfo);
			freeVertsCount = preCreateVertsCount;
		}

		TriVertsInfo* triVertsInfoPtr = triVertInfoList.back();
		triVertsInfoPtr += preCreateVertsCount - freeVertsCount;
		freeVertsCount--;
		return triVertsInfoPtr;
	}

	void VoxelSpace::CalFloorGridIdxRange()
	{
		//xstartCell
		float n = spaceAABB.minX * invCellSize;
		xstartCell = (int)floor(n);

		//xendCell
		n = spaceAABB.maxX * invCellSize;
		xendCell = (int)ceil(n);
		if (xstartCell == xendCell) xendCell++;

		//zstartCell
		n = spaceAABB.minZ * invCellSize;
		zstartCell = (int)floor(n);

		//zendCell
		n = spaceAABB.maxZ * invCellSize;
		zendCell = (int)ceil(n);
		if (zstartCell == zendCell) zendCell++;

		//
		cellxCount = xendCell - xstartCell;
		cellzCount = zendCell - zstartCell;
	}

	void VoxelSpace::CreateSoildSpanSpaceGrids()
	{
		gridCount = cellxCount * cellzCount;
		int size = sizeof(SolidSpanList) * gridCount;
		solidSpanGrids = (SolidSpanList*)malloc(size);
		memset(solidSpanGrids, 0, size);

		//
		cellxList = new float[cellxCount + 1];
		cellzList = new float[cellzCount + 1];


		for (int i = zstartCell; i <= zendCell; i++)
		{
			cellzList[i - zstartCell] = i * cellSize;
		}

		for (int i = xstartCell; i <= xendCell; i++)
		{
			cellxList[i - xstartCell] = i * cellSize;
		}
	}