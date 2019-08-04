#pragma once

#define MinNavDLL_EXPORTS

#ifdef MinNavDLL_EXPORTS
#define MinNavDLL_API __declspec(dllexport)
#else
#define MinNavDLL_API __declspec(dllimport)
#endif       

enum MiNavOverlapRelation
{
	/// <summary>
	/// 不重叠
	/// </summary>
	NotOverlay = -1,

	/// <summary>
	/// 完全重叠
	/// </summary>
	FullOverlap,

	/// <summary>
	/// 部分重叠
	/// </summary>
	PartOverlay,

};

struct MiNavAABB
{
	float minX, maxX;
	float minZ, maxZ;
	float minY, maxY;
};

struct SimpleVector3
{
	float x = 0, y = 0, z = 0;
	
	SimpleVector3()
	{

	}

	SimpleVector3(float x, float y, float z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}
};

struct CellLineRange
{
	float start, end;
};

struct LineParam
{
	float m, b;
	float offsety;
	float ystart, yend;	
};

struct TriVertsInfo
{
	SimpleVector3 vert0;
	SimpleVector3 vert1;
	SimpleVector3 vert2;
	MiNavAABB aabb;
};