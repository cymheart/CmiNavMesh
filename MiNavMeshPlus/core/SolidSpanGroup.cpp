#include"SolidSpanGroup.h"

SolidSpanGroup::SolidSpanGroup(VoxelSpace* voxSpace)
{
	this->voxSpace = voxSpace;
	solidSpanGrids = voxSpace->solidSpanGrids;
	gridCount = voxSpace->gridCount;
}

SolidSpanGroup::~SolidSpanGroup()
{

}

//test
void SolidSpanGroup::AppendVoxBox(
	int floorCellIdxX, int floorCellIdxZ,
	int heightCellStartIdx, int heightCellEndIdx)
{
	int idx = voxSpace->GetFloorGridIdx(floorCellIdxX, floorCellIdxZ);
	SolidSpanList* solidSpanList = &(solidSpanGrids[idx]);

	SolidSpanList tmp = *solidSpanList;

	if (solidSpanList->first == nullptr)
	{
		solidSpanList->floorCellIdxX = floorCellIdxX;
		solidSpanList->floorCellIdxZ = floorCellIdxZ;
	}

	AppendVoxBoxToSpanHeightList(solidSpanList, heightCellStartIdx, heightCellEndIdx);
}


void SolidSpanGroup::AppendVoxBoxToSpanHeightList(
	SolidSpanList* solidSpanList,
	int heightCellStartIdx, int heightCellEndIdx)
{
	int voxStartIdx = heightCellStartIdx;
	int voxEndIdx = heightCellEndIdx;
	float yPosStart = voxStartIdx * voxSpace->cellHeight;
	float yPosEnd = voxEndIdx * voxSpace->cellHeight;

	SolidSpan* startNode = nullptr;
	SolidSpan* endNode = nullptr;

	SolidSpan* node = solidSpanList->first;
	for (; node != nullptr; node = node->next)
	{
		if (startNode == nullptr)
		{
			if (node->ystartCellIdx > voxStartIdx)
			{
				startNode = node;
			}
			else if (voxStartIdx >= node->ystartCellIdx &&
				voxStartIdx <= node->yendCellIdx)
			{
				yPosStart = node->ystartPos;
				voxStartIdx = node->ystartCellIdx;
				startNode = node;
			}
		}

		if (endNode == nullptr)
		{
			if (node->ystartCellIdx > voxEndIdx)
			{
				endNode = node->prev;
				break;
			}
			else if (voxEndIdx >= node->ystartCellIdx &&
				voxEndIdx <= node->yendCellIdx)
			{
				yPosEnd = node->yendPos;
				voxEndIdx = node->yendCellIdx;
				endNode = node;
				break;
			}
		}
	}

	if (startNode != nullptr && node == nullptr)
		endNode = solidSpanList->last;


	SolidSpan* voxSpan = voxSpace->GetSoildSpan();
	voxSpan->ystartPos = yPosStart;
	voxSpan->yendPos = yPosEnd;
	voxSpan->ystartCellIdx = voxStartIdx;
	voxSpan->yendCellIdx = voxEndIdx;

	if (endNode != nullptr && endNode->next == startNode)
	{
		solidSpanList->AddAfter(endNode, voxSpan);
		return;
	}

	if (startNode != nullptr && endNode == nullptr)
	{
		solidSpanList->AddFirst(voxSpan);
		return;
	}

	if (startNode == nullptr && endNode == nullptr)
	{
		if (node == solidSpanList->first)
			solidSpanList->AddFirst(voxSpan);
		else
			solidSpanList->AddLast(voxSpan);
	}
	else
	{
		SolidSpan* prevNode = startNode->prev;
		SolidSpan* mnode = startNode;
		SolidSpan* tmpNode;
		bool flag = true;

		while (flag)
		{
			if (mnode == endNode)
				flag = false;

			tmpNode = mnode->next;
			solidSpanList->Remove(mnode);
			mnode = tmpNode;
		}

		if (prevNode == nullptr)
			solidSpanList->AddFirst(voxSpan);
		else
			solidSpanList->AddAfter(prevNode, voxSpan);
	}
}