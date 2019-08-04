#pragma once
#include"VoxelSpace.h"
#include<vector>
using namespace std;

class VoxelTriangle
{
private:
	VoxelSpace* voxSpace;
	const float esp = 0.001f;

	SimpleVector3 vertexs[4];
	SimpleVector3 floorCellRect[4];

	float m[3], n[3];
	float a[3], b[3];
	float a2[3], b2[3];
	float a3[3], b3[3];

	float cellProjYpos[30];
	int cellProjPtsCount = 0;

	int vertCellX[3];
	int vertCellZ[3];

	SimpleVector3 triFaceNormal;
	SimpleVector3 floorGridNormal = SimpleVector3(0,1,0);
	bool isHorPlane = false;
	bool isInSingleCell = false;

	SolidSpanGroup* solidSpanGroup;

	MiNavAABB aabb;
	int xstartCell, xendCell;
	int zstartCell, zendCell;

	float* cellxList;
	float* cellzList;

	vector<CellLineRange> xcolZRangeList = vector<CellLineRange>(2000);
	vector<CellLineRange> zrowXRangeList = vector<CellLineRange>(2000);
	vector<LineParam> zrowXYPlaneLineParamList = vector<LineParam>(2000);
	vector<LineParam> xrowZYPlaneLineParamList = vector<LineParam>(2000);

public:
	VoxelTriangle(VoxelSpace* voxSpace, SolidSpanGroup* solidSpanGroup);
	~VoxelTriangle();
	void CreateVoxels(TriVertsInfo* info);

private:

	// 计算三角面法线
	void CalTriFaceNormal();

	// 计算三角面在floorGrid的格子范围
	void CalFloorGridIdxRange();
	
	// 计算三角面顶点所在单元格
	void CalTriVertsAtCells();
	void CreateVertexsProjFloorSidesParams();
	void CreateVertexsProjZYPlaneSidesParams();
	void CreateVertexsProjXYPlaneSidesParams();
	void CreateFloorCellLines();
	void CreateXYPlaneLinesParams();
	void CreateZYPlaneLinesParams();

	// 生成地面所有网格投影到TriFace上的体素Box
	void CreateFloorGridProjTriFaceVoxBox();

	// 生成地面所有网格投影到TriFace上的体素Box
	void CreateFloorGridProjTriFaceVoxBoxForHorPlane();

	void CreateFloorGridProjTriFaceVoxBoxForInSingleCell();

	// 获取单元格与投影三角形的覆盖关系
	MiNavOverlapRelation GetOverlapRelation(int cellx, int cellz);
	
	// 获取单元格与投影三角形的覆盖关系，针对水平平面
	MiNavOverlapRelation GetOverlapRelationForHorPlane(int cellx, int cellz);

	// 生成投影到TriFace上的pts
	void CreateProjectionToTriFacePts(SimpleVector3* pts, int count);

	SimpleVector3 SolveCrossPoint(SimpleVector3 p, SimpleVector3 n, SimpleVector3 o, SimpleVector3 m);

	// 根据投影Rect生成体素box
	void CreateVoxBoxToList(int cellx, int cellz);



};