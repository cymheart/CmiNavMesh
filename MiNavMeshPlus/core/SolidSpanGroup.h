#pragma once
#include"VoxelSpace.h"

struct SolidSpan
{
	float ystartPos;
	float yendPos;

	int ystartCellIdx;
	int yendCellIdx;

	SolidSpan* prev;
	SolidSpan* next;
};

struct SolidSpanList
{

	int floorCellIdxX;
	int floorCellIdxZ;
	SolidSpan* first;
	SolidSpan* last;

	void AddFirst(SolidSpan* node)
	{
		if (first != nullptr)
		{
			first->prev = node;
			node->next = first;
			first = node;
		}
		else
		{
			first = node;
			last = node;
		}
	}

	void AddLast(SolidSpan* node)
	{
		if (last != nullptr)
		{
			last->next = node;
			node->prev = last;
			last = node;
		}
		else
		{
			first = node;
			last = node;
		}
	}

	void AddAfter(SolidSpan* after, SolidSpan* node)
	{
		if (after != nullptr)
		{
			SolidSpan* tmp = after->next;
			after->next = node;
			node->prev = after;
			node->next = tmp;

			if (tmp != nullptr)
			{
				tmp->prev = node;
			}
			else
			{
				last = node;
			}
		}
		else
		{
			AddFirst(node);
		}
	}


	void Remove(SolidSpan* node)
	{
		SolidSpan* prev = node->prev;
		SolidSpan* next = node->next;

		if (prev != nullptr)
			prev->next = next;
		else
			first = next;

		if (next != nullptr)
			next->prev = prev;
		else
			last = prev;
	}
};



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