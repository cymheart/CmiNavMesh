#include <iostream>
#include<fstream>
#include"core/VoxelSpace.h"
#include"core/SolidSpanGroup.h"
#include<windows.h>

void CreateVerts(char* txt, SimpleVector3* svGroup);
int ParseNumber(char* txt, int startpos, int endpos, float* num);
int SkipSpace(char* txt, int startpos, int endpos);

int main()
{
	VoxelSpace voxSpace;

	SimpleVector3 sv;
	SimpleVector3 svGroup[3];
	char d[256];
	const char *filename = "j:\\verts.txt";

	std::ifstream inf(filename);
	if (!inf.is_open())
	{
		std::cout << "Error opening file";
	}
	while (!inf.eof())
	{
		inf.getline(d, 200);	
		CreateVerts(d, svGroup);

		if (svGroup[0].x > -0.0001 && svGroup[0].x <0.0001 &&
			svGroup[0].z > -0.0001 && svGroup[0].z <0.0001)
		{
			continue;
		}

		voxSpace.TransModelVertexs(svGroup);
	}
	system("pause");
	system("pause");
	DWORD start = ::GetTickCount();

	voxSpace.CreateSpaceGrids();
	SolidSpanGroup* solidSpanGroup = new SolidSpanGroup(&voxSpace);
	voxSpace.CreateVoxels(solidSpanGroup);

	cout <<"花费时间:"<< ::GetTickCount() - start <<"ms"<< endl;
	system("pause");
}

void CreateVerts(char* txt, SimpleVector3* svGroup)
{
	size_t endpos = strlen(txt) - 1;

	int startpos = 0;
	float num[3];
	startpos = ParseNumber(txt, startpos, endpos, &num[0]);
	startpos = ParseNumber(txt, startpos, endpos, &num[1]);
	startpos = ParseNumber(txt, startpos, endpos, &num[2]);
	SimpleVector3 sv0 = { num[0], num[1], num[2] };

	startpos = ParseNumber(txt, startpos, endpos, &num[0]);
	startpos = ParseNumber(txt, startpos, endpos, &num[1]);
	startpos = ParseNumber(txt, startpos, endpos, &num[2]);
	SimpleVector3 sv1 = { num[0], num[1], num[2] };

	startpos = ParseNumber(txt, startpos, endpos, &num[0]);
	startpos = ParseNumber(txt, startpos, endpos, &num[1]);
	startpos = ParseNumber(txt, startpos, endpos, &num[2]);
	SimpleVector3 sv2 = { num[0], num[1], num[2] };

	svGroup[0] = sv0;
	svGroup[1] = sv1;
	svGroup[2] = sv2;
}

int SkipSpace(char* txt, int startpos, int endpos)
{
	int i = startpos;
	while (*(txt + i) == ' ')
	{
		i++;
		if (i > endpos)
			break;
	}

	return i;
}

int ParseNumber(char* txt, int startpos, int endpos, float* num)
{
	int i = SkipSpace(txt, startpos, endpos);
	
	int j = i;
	while (*(txt + j) != ' ' && *(txt + j) != '\0')
		j++;

	char ss[100];
	strncpy_s(ss, txt + i, j - i);
	*num = atof(ss);

	return j;
}