#include "../caller/SolidSpanGroupCaller.h"


SolidSpanGroup* CreateSolidSpanGroup(VoxelSpace* voxSpace)
{
	return new SolidSpanGroup(voxSpace);
}

void DisposeSolidSpanGroup(SolidSpanGroup* solidSpanGroup)
{
	if (solidSpanGroup != NULL)
	{
		delete solidSpanGroup;
		solidSpanGroup = NULL;
	}
}

SolidSpanList* GetSolidSpanGrids(SolidSpanGroup* solidSpanGroup)
{
	return solidSpanGroup->GetSolidSpanGrids();
}

int GetGridCount(SolidSpanGroup* solidSpanGroup)
{
	return solidSpanGroup->GetGridCount();
}