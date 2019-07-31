using LinearAlgebra.VectorAlgebra;
using Mathd;
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

namespace MINAV
{
    public class VoxelTriangle
    {
        VoxelSpace voxSpace;
        readonly float esp = 0.002f;

        SimpleVector3[] vertexs = new SimpleVector3[4];

        SimpleVector3[] floorCellRect = new SimpleVector3[4]
      {
            new SimpleVector3(0, 0, 0),
            new SimpleVector3(0, 0, 0),
            new SimpleVector3(0, 0, 0),
            new SimpleVector3(0, 0, 0)
      };

        float[] m = new float[3];
        float[] n = new float[3];
        float[] a = new float[3];
        float[] b = new float[3];
        float[] a2 = new float[3];
        float[] b2 = new float[3];
        float[] a3 = new float[3];
        float[] b3 = new float[3];

        float[] cellProjYpos = new float[30];
        int cellProjPtsCount = 0;

        int[] vertCellX = new int[3];
        int[] vertCellZ = new int[3];

        SimpleVector3 triFaceNormal;
        SimpleVector3 floorGridNormal = new SimpleVector3(0,1,0);
        bool isHorPlane = false;
        bool isInSingleCell = false;

        SolidSpanGroup solidSpanGroup;

        MiNavAABB aabb = new MiNavAABB();
        int xstartCell, xendCell;
        int zstartCell, zendCell;

        List<CellLineRange> xcolZRangeList = new List<CellLineRange>(2000);
        List<CellLineRange> zrowXRangeList = new List<CellLineRange>(2000);
        List<LineParam> zrowXYPlaneLineParamList = new List<LineParam>(2000);
        List<LineParam> xrowZYPlaneLineParamList = new List<LineParam>(2000);

        public VoxelTriangle(VoxelSpace voxSpace, SolidSpanGroup solidSpanGroup)
        {
            this.voxSpace = voxSpace;
            this.solidSpanGroup = solidSpanGroup;
        }


        public unsafe void CreateVoxels(TriVertsInfo* info)
        {
            vertexs[0] = info->vert0;
            vertexs[1] = info->vert1;
            vertexs[2] = info->vert2;
            vertexs[3] = info->vert0;
            aabb = info->aabb;

            CalTriVertsAtCells();
            if (isInSingleCell)
            {
                CreateFloorGridProjTriFaceVoxBoxForInSingleCell();
                return;
            }

            //
            CalTriFaceNormal();
            CalFloorGridIdxRange();  

            if(xstartCell == -25430 && xendCell == -25419 &&
                zstartCell == 25479 && zendCell == 25490)
            {
                int a;
                a = 3;
            }



            CreateVertexsProjFloorSidesParams();
            CreateFloorCellLines();

            if (!isHorPlane)
            {
                CreateVertexsProjZYPlaneSidesParams();
                CreateVertexsProjXYPlaneSidesParams();                
                CreateXYPlaneLinesParams();
                CreateZYPlaneLinesParams();
                CreateFloorGridProjTriFaceVoxBox();
            }
            else
            {
                CreateFloorGridProjTriFaceVoxBoxForHorPlane();
            }
        }

        

        /// <summary>
        /// 计算三角面法线
        /// </summary>
        void CalTriFaceNormal()
        {
            SimpleVector3 vec1 = new SimpleVector3()
            {
                x = vertexs[1].x - vertexs[0].x,
                y = vertexs[1].y - vertexs[0].y,
                z = vertexs[1].z - vertexs[0].z,
            };

            SimpleVector3 vec2 = new SimpleVector3()
            {
                x = vertexs[2].x - vertexs[0].x,
                y = vertexs[2].y - vertexs[0].y,
                z = vertexs[2].z - vertexs[0].z,
            };

            float x = vec1.y * vec2.z - vec2.y * vec1.z;
            float y = vec1.z * vec2.x - vec2.z * vec1.x;
            float z = vec1.x * vec2.y - vec2.x * vec1.y;
            triFaceNormal = new SimpleVector3(x, y, z);

            //
            isHorPlane = false;
            if (triFaceNormal.x > -esp && triFaceNormal.x < esp && 
                triFaceNormal.z > -esp && triFaceNormal.z < esp)
            {
                isHorPlane = true;
            }
        }

       
        /// <summary>
        /// 计算三角面在floorGrid的格子范围
        /// </summary>
        void CalFloorGridIdxRange()
        {
            //xstartCell
            float n = aabb.minX * voxSpace.invCellSize;
            xstartCell = (int)Math.Floor(n);

            //xendCell
            n = aabb.maxX * voxSpace.invCellSize;
            xendCell = (int)Math.Ceiling(n);
            if (xstartCell == xendCell) xendCell++;

            //zstartCell
            n = aabb.minZ * voxSpace.invCellSize;
            zstartCell = (int)Math.Floor(n);

            //zendCell
            n = aabb.maxZ * voxSpace.invCellSize;
            zendCell = (int)Math.Ceiling(n);
            if (zstartCell == zendCell) zendCell++;

        }

      
        /// <summary>
        /// 计算三角面顶点所在单元格
        /// </summary>
        void CalTriVertsAtCells()
        {
            for (int i = 0; i < 3; i++)
            {
                float cell = vertexs[i].x * voxSpace.invCellSize;
                vertCellX[i] = (int)Math.Floor(cell);
                cell = vertexs[i].z * voxSpace.invCellSize;
                vertCellZ[i] = (int)Math.Floor(cell);
            }

            //
            isInSingleCell = false;
            if (vertCellX[0] == vertCellX[1] && vertCellX[0] == vertCellX[2] &&
                vertCellZ[0] == vertCellZ[1] && vertCellZ[0] == vertCellZ[2])
            {
                isInSingleCell = true;
            }
        }

        void CreateVertexsProjFloorSidesParams()
        {
            float tmpM;
            float tmpN;
            SimpleVector3 vec = new SimpleVector3();
            for (int i = 0; i < 3; i++)
            {
                vec.x = vertexs[i + 1].x - vertexs[i].x;
                vec.z = vertexs[i + 1].z - vertexs[i].z;

                if (vec.z > -esp && vec.z < esp)
                {
                    m[i] = 0;

                    //
                    b[i] = vertexs[i].z;
                    a[i] = 99999;
                }
                else if (vec.x != 0)
                {
                    tmpM = vec.z / vec.x;
                    tmpN = vertexs[i].z - tmpM * vertexs[i].x;
                    m[i] = 1 / tmpM;
                    n[i] = -tmpN;

                    //
                    b[i] = tmpN;
                    a[i] = tmpM;
                }
                else
                {
                    m[i] = 99999;
                    n[i] = vertexs[i].x;

                    a[i] = 0;
                }
            }  
        }


        void CreateVertexsProjZYPlaneSidesParams()
        {
            float tmpM;
            float tmpN;
            SimpleVector3 vec = new SimpleVector3();
            for (int i = 0; i < 3; i++)
            {
                vec.y = vertexs[i + 1].y - vertexs[i].y;
                vec.z = vertexs[i + 1].z - vertexs[i].z;

                if (vec.y > -esp && vec.y < esp)
                {
                    b2[i] = vertexs[i].y;
                    a2[i] = 99999;
                }
                else if (vec.z != 0)
                {
                    tmpM = vec.y / vec.z;
                    tmpN = vertexs[i].y - tmpM * vertexs[i].z;
                    b2[i] = tmpN;
                    a2[i] = tmpM;
                }
                else
                {
                    a2[i] = 0;
                }
            }
        }

        void CreateVertexsProjXYPlaneSidesParams()
        {
            float tmpM;
            float tmpN;
            SimpleVector3 vec = new SimpleVector3();
            for (int i = 0; i < 3; i++)
            {
                vec.y = vertexs[i + 1].y - vertexs[i].y;
                vec.x = vertexs[i + 1].x - vertexs[i].x;

                if (vec.y > -esp && vec.y < esp)
                {
                    b3[i] = vertexs[i].y;
                    a3[i] = 99999;
                }
                else if (vec.x != 0)
                {
                    tmpM = vec.y / vec.x;
                    tmpN = vertexs[i].y - tmpM * vertexs[i].x;
                    b3[i] = tmpN;
                    a3[i] = tmpM;
                }
                else
                {
                    a3[i] = 0;
                }
            }
        }

        void CreateFloorCellLines()
        {
            zrowXRangeList.Clear();
            xcolZRangeList.Clear();

            float cellSize = voxSpace.cellSize;    
            float z = zstartCell * cellSize - cellSize;
            float x;
            float min, max;
            CellLineRange cellLineRange;

            for (int j = zstartCell; j <= zendCell; j++)
            {
                z += cellSize;
                min = 999999; max = -999999;

                for (int i=0; i<3; i++)
                {
                    if (m[i] == 0)
                        continue;

                    if (!(z >= vertexs[i].z - esp && z <= vertexs[i + 1].z + esp) &&
                        !(z >= vertexs[i + 1].z - esp && z <= vertexs[i].z + esp))
                        continue;

                    if (m[i] != 99999)
                        x = (z + n[i]) * m[i];
                    else
                        x = n[i];

                    if (x < min)
                        min = x;
                    if (x > max)
                        max = x;
                }

                cellLineRange = new CellLineRange() { start = min, end = max };
                zrowXRangeList.Add(cellLineRange);
            }


            //
            x = xstartCell * cellSize - cellSize;
            for (int j = xstartCell; j <= xendCell; j++)
            {
                x += cellSize;
                min = 999999; max = -999999;

                for (int i = 0; i < 3; i++)
                {
                    if (a[i] == 0)
                        continue;

                    if (!(x >= vertexs[i].x - esp && x <= vertexs[i + 1].x + esp) &&
                        !(x >= vertexs[i + 1].x - esp && x <= vertexs[i].x + esp))
                        continue;

                    if (a[i] != 99999)
                        z = a[i] * x + b[i];
                    else
                        z = b[i];

                    if (z < min)
                        min = z;
                    if (z > max)
                        max = z;
                }

                cellLineRange = new CellLineRange() { start = min, end = max };
                xcolZRangeList.Add(cellLineRange);
            }

        }

        void CreateXYPlaneLinesParams()
        {
            zrowXYPlaneLineParamList.Clear();

            float y;
            int invPlaneType = 0;
            float cellSize = voxSpace.cellSize;
            float z = zstartCell * cellSize - cellSize;
            float min, max;
            float m1, n;
            LineParam lineParam;
            float _ystart = 0, _yend = 0;

            for (int j = zstartCell; j <= zendCell; j++)
            {
                z += cellSize;
                min = 999999; max = -999999;

                for (int i = 0; i < 3; i++)
                {
                    if (a2[i] == 0)
                        continue;

                    if (!(z >= vertexs[i].z - esp && z <= vertexs[i + 1].z + esp) &&
                        !(z >= vertexs[i + 1].z - esp && z <= vertexs[i].z + esp))
                        continue;

                    if (a2[i] != 99999)
                        y = a2[i] * z + b2[i];
                    else
                        y = b2[i];

                    if (y < min)
                        min = y;
                    if (y > max)
                        max = y;
                }



                if (invPlaneType == 0)
                {
                    if (min < max - esp)
                    {
                        SimpleVector3 orgStart = new SimpleVector3(zrowXRangeList[j - zstartCell].start, 0, z);
                        orgStart = SolveCrossPoint(orgStart, floorGridNormal, vertexs[0], triFaceNormal);

                        if (Math.Abs(orgStart.y - min) < Math.Abs(orgStart.y - max))
                            invPlaneType = 1;
                        else
                            invPlaneType = -1;
                    }
                    else if(min >= max - esp && min <= max + esp)
                    {
                        _ystart = min;
                        _yend = max;
                    }
                    else
                    {
                        _ystart = 0;
                        _yend = 0;
                    }
                }

                if (invPlaneType == 1)
                {
                    _ystart = min;
                    _yend = max;
                }
                else if (invPlaneType == -1)
                {
                    _ystart = max;
                    _yend = min;
                }

                CellLineRange xa = zrowXRangeList[j - zstartCell];
                if (xa.end > xa.start){
                    m1 = (_yend - _ystart) / (xa.end - xa.start);
                    n = _ystart - m1 * xa.start;
                }
                else
                {
                    m1 = 99999;
                    n = 0;
                }

                lineParam = new LineParam() { m = m1, b = n, ystart = _ystart, yend = _yend };
                zrowXYPlaneLineParamList.Add(lineParam);
            }
        }

        void CreateZYPlaneLinesParams()
        {
            xrowZYPlaneLineParamList.Clear();

            float y;
            int invPlaneType = 0;
            float cellSize = voxSpace.cellSize;
            float x = xstartCell * cellSize - cellSize;
            float min, max;
            float m1, n;
            LineParam lineParam;
            float _ystart = 0, _yend = 0;

            for (int j = xstartCell; j <= xendCell; j++)
            {
                x += cellSize;
                min = 999999; max = -999999;

                for (int i = 0; i < 3; i++)
                {
                    if (a3[i] == 0)
                        continue;

                    if (!(x >= vertexs[i].x - esp && x <= vertexs[i + 1].x + esp) &&
                        !(x >= vertexs[i + 1].x - esp && x <= vertexs[i].x + esp))
                        continue;

                    if (a3[i] != 99999)
                        y = a3[i] * x + b3[i];
                    else
                        y = b3[i];

                    if (y < min)
                        min = y;
                    if (y > max)
                        max = y;
                }


                if (invPlaneType == 0)
                {
                    if (min < max - esp)
                    {
                        SimpleVector3 orgStart = new SimpleVector3(x, 0, xcolZRangeList[j - xstartCell].start);
                        orgStart = SolveCrossPoint(orgStart, floorGridNormal, vertexs[0], triFaceNormal);

                        if(Math.Abs(orgStart.y - min) < Math.Abs(orgStart.y - max))
                            invPlaneType = 1;
                        else
                            invPlaneType = -1;
                    }
                    else if (min >= max - esp && min <= max + esp)
                    {
                        _ystart = min;
                        _yend = max;
                    }
                    else
                    {
                        _ystart = 0;
                        _yend = 0;
                    }
                }

                if (invPlaneType == 1)
                {
                    _ystart = min;
                    _yend = max;
                }
                else if (invPlaneType == -1)
                {
                    _ystart = max;
                    _yend = min;
                }

                CellLineRange xa = xcolZRangeList[j - xstartCell];
                if (xa.end > xa.start)
                {
                    m1 = (_yend - _ystart) / (xa.end - xa.start);
                    n = _ystart - m1 * xa.start;
                }
                else
                {
                    m1 = 99999;
                    n = 0;
                }

                lineParam = new LineParam() { m = m1, b = n, ystart = _ystart, yend = _yend };
                xrowZYPlaneLineParamList.Add(lineParam);
            }
        }

        /// <summary>
        /// 生成地面所有网格投影到TriFace上的体素Box
        /// </summary>
        void CreateFloorGridProjTriFaceVoxBox()
        {
            float cellSize = voxSpace.cellSize;
            float xStart = xstartCell * cellSize;
            float zStart = zstartCell * cellSize;
            floorCellRect[0].x = xStart - cellSize;
            floorCellRect[1].x = xStart - cellSize;
            floorCellRect[2].x = xStart;
            floorCellRect[3].x = xStart;

            for (int x = xstartCell; x < xendCell; x++)
            {
                floorCellRect[0].x += cellSize;
                floorCellRect[1].x += cellSize;
                floorCellRect[2].x += cellSize;
                floorCellRect[3].x += cellSize;

                floorCellRect[0].z = zStart - cellSize;
                floorCellRect[1].z = zStart;
                floorCellRect[2].z = zStart;
                floorCellRect[3].z = zStart - cellSize;

                for (int z = zstartCell; z < zendCell; z++)
                {
                    floorCellRect[0].z += cellSize;
                    floorCellRect[1].z += cellSize;
                    floorCellRect[2].z += cellSize;
                    floorCellRect[3].z += cellSize;

                    int idx = voxSpace.GetFloorGridIdx(x, z);

                    if (idx == 243609)
                    {

                        if (GetOverlapRelation(x, z) == MiNavOverlapRelation.NotOverlay)
                            continue;

                        CreateVoxBoxToList(x, z);
                    }

                }
            }
        }

        /// <summary>
        /// 生成地面所有网格投影到TriFace上的体素Box
        /// </summary>
        void CreateFloorGridProjTriFaceVoxBoxForHorPlane()
        {
            float cellSize = voxSpace.cellSize;
            float xStart = xstartCell * cellSize;
            float zStart = zstartCell * cellSize;
            floorCellRect[0].x = xStart - cellSize;
            floorCellRect[1].x = xStart - cellSize;
            floorCellRect[2].x = xStart;
            floorCellRect[3].x = xStart;

            for (int x = xstartCell; x < xendCell; x++)
            {
                floorCellRect[0].x += cellSize;
                floorCellRect[1].x += cellSize;
                floorCellRect[2].x += cellSize;
                floorCellRect[3].x += cellSize;

                floorCellRect[0].z = zStart - cellSize;
                floorCellRect[1].z = zStart;
                floorCellRect[2].z = zStart;
                floorCellRect[3].z = zStart - cellSize;

                for (int z = zstartCell; z < zendCell; z++)
                {
                    floorCellRect[0].z += cellSize;
                    floorCellRect[1].z += cellSize;
                    floorCellRect[2].z += cellSize;
                    floorCellRect[3].z += cellSize;

                    if (GetOverlapRelationForHorPlane(x, z) == MiNavOverlapRelation.NotOverlay)
                        continue;

                    int start = (int)Math.Floor(vertexs[0].y * voxSpace.invCellHeight);
                    int end = start + 1;
                    solidSpanGroup.AppendVoxBox(x, z, start, end);
                }
            }
        }

        void CreateFloorGridProjTriFaceVoxBoxForInSingleCell()
        {
            cellProjYpos[0] = vertexs[0].y;
            cellProjYpos[1] = vertexs[1].y;
            cellProjYpos[2] = vertexs[2].y;
            cellProjPtsCount = 3;

            CreateVoxBoxToList(vertCellX[0], vertCellZ[0]);
        }

        /// <summary>
        /// 获取单元格与投影三角形的覆盖关系
        /// </summary>
        /// <returns></returns>
        MiNavOverlapRelation GetOverlapRelation(int cellx, int cellz)
        {
            int idx = cellz - zstartCell;
            CellLineRange xa = zrowXRangeList[idx];
            CellLineRange xb = zrowXRangeList[idx + 1];
          
            idx = cellx - xstartCell;
            CellLineRange za = xcolZRangeList[idx];
            CellLineRange zb = xcolZRangeList[idx + 1];
           

            if (((floorCellRect[2].x < xb.start && floorCellRect[3].x < xa.start) || 
                (floorCellRect[1].x > xb.end && floorCellRect[0].x > xa.end)) &&
                ((floorCellRect[0].z > za.end && floorCellRect[3].z > zb.end) ||
                (floorCellRect[1].z < za.start && floorCellRect[2].z < zb.start)))
            {
                return MiNavOverlapRelation.NotOverlay;
            }


            LineParam lineParamA, lineParamB;

            if (floorCellRect[0].x >= xa.start - esp && floorCellRect[0].x<= xa.end + esp &&
               floorCellRect[3].x >= xa.start - esp && floorCellRect[3].x <= xa.end + esp &&
               floorCellRect[1].x >= xb.start - esp && floorCellRect[1].x <= xb.end + esp &&
               floorCellRect[2].x >= xb.start - esp && floorCellRect[2].x <= xb.end + esp)
            {
                cellProjPtsCount = 0;
                idx = cellz - zstartCell;
                lineParamA = zrowXYPlaneLineParamList[idx];
                lineParamB = zrowXYPlaneLineParamList[idx + 1];

                cellProjYpos[cellProjPtsCount++] = floorCellRect[0].x * lineParamA.m + lineParamA.b;
                cellProjYpos[cellProjPtsCount++] = floorCellRect[3].x * lineParamA.m + lineParamA.b;
                cellProjYpos[cellProjPtsCount++] = floorCellRect[1].x * lineParamB.m + lineParamB.b;
                cellProjYpos[cellProjPtsCount++] = floorCellRect[2].x * lineParamB.m + lineParamB.b;

                return MiNavOverlapRelation.FullOverlap;
            }

            //
            idx = cellz - zstartCell;
            lineParamA = zrowXYPlaneLineParamList[idx];
            lineParamB = zrowXYPlaneLineParamList[idx + 1];
            cellProjPtsCount = 0;

            if (floorCellRect[0].x >= xa.start - esp && floorCellRect[0].x <= xa.end + esp && lineParamA.m != 99999)
            {
                cellProjYpos[cellProjPtsCount++] = floorCellRect[0].x * lineParamA.m + lineParamA.b;
            }
            if (floorCellRect[3].x >= xa.start - esp && floorCellRect[3].x <= xa.end + esp && lineParamA.m != 99999)
            {
                cellProjYpos[cellProjPtsCount++] = floorCellRect[3].x * lineParamA.m + lineParamA.b;
            }
            if (floorCellRect[1].x >= xb.start - esp && floorCellRect[1].x <= xb.end + esp && lineParamB.m != 99999)
            {    
                cellProjYpos[cellProjPtsCount++] = floorCellRect[1].x * lineParamB.m + lineParamB.b;
            }
            if (floorCellRect[2].x >= xb.start - esp && floorCellRect[2].x <= xb.end + esp && lineParamB.m != 99999)
            {
                cellProjYpos[cellProjPtsCount++] = floorCellRect[2].x * lineParamB.m + lineParamB.b;
            }


            if (xa.start >= floorCellRect[0].x - esp && xa.start <= floorCellRect[3].x + esp)
            {
                cellProjYpos[cellProjPtsCount++] = lineParamA.ystart;
            }
            if (xa.end >= floorCellRect[0].x - esp && xa.end <= floorCellRect[3].x + esp)
            {
                cellProjYpos[cellProjPtsCount++] = lineParamA.yend;
            }
            if (xb.start >= floorCellRect[1].x - esp && xb.start <= floorCellRect[2].x + esp)
            {
                cellProjYpos[cellProjPtsCount++] = lineParamB.ystart;
            }
            if (xb.end >= floorCellRect[1].x - esp && xb.end <= floorCellRect[2].x + esp)
            {
                cellProjYpos[cellProjPtsCount++] = lineParamB.yend;
            }


            //
            idx = cellx - xstartCell;
            lineParamA = xrowZYPlaneLineParamList[idx];
            lineParamB = xrowZYPlaneLineParamList[idx + 1];
            if (za.start >= floorCellRect[0].z - esp && za.start <= floorCellRect[1].z + esp)
            {
                cellProjYpos[cellProjPtsCount++] = lineParamA.ystart;
            }
            if (za.end >= floorCellRect[0].z - esp && za.end <= floorCellRect[1].z + esp)
            {
                cellProjYpos[cellProjPtsCount++] = lineParamA.yend;
            }           
            if (zb.start >= floorCellRect[3].z - esp && zb.start <= floorCellRect[2].z + esp)
            {
                cellProjYpos[cellProjPtsCount++] = lineParamB.ystart;
            }
            if (zb.end >= floorCellRect[3].z - esp && zb.end <= floorCellRect[2].z + esp)
            {
                cellProjYpos[cellProjPtsCount++] = lineParamB.yend;
            }

            for (int i = 0; i < 3; i++)
            {
                if (vertCellX[i] == cellx && vertCellZ[i] == cellz)
                {
                    cellProjYpos[cellProjPtsCount++] = vertexs[i].y;
                    break;
                }
            }

            return MiNavOverlapRelation.PartOverlay;
        }


        /// <summary>
        /// 获取单元格与投影三角形的覆盖关系，针对水平平面
        /// </summary>
        /// <returns></returns>
        MiNavOverlapRelation GetOverlapRelationForHorPlane(int cellx, int cellz)
        {
            int idx = cellz - zstartCell;
            CellLineRange xa = zrowXRangeList[idx];
            CellLineRange xb = zrowXRangeList[idx + 1];

            idx = cellx - xstartCell;
            CellLineRange za = xcolZRangeList[idx];
            CellLineRange zb = xcolZRangeList[idx + 1];


            if (((floorCellRect[2].x < xb.start && floorCellRect[3].x < xa.start) ||
                (floorCellRect[1].x > xb.end && floorCellRect[0].x > xa.end)) &&
                ((floorCellRect[0].z > za.end && floorCellRect[3].z > zb.end) ||
                (floorCellRect[1].z < za.start && floorCellRect[2].z < zb.start)))
            {
                return MiNavOverlapRelation.NotOverlay;
            }

            if (floorCellRect[0].x >= xa.start - esp && floorCellRect[0].x <= xa.end + esp &&
               floorCellRect[3].x >= xa.start - esp && floorCellRect[3].x <= xa.end + esp &&
               floorCellRect[1].x >= xb.start - esp && floorCellRect[1].x <= xb.end + esp &&
               floorCellRect[2].x >= xb.start - esp && floorCellRect[2].x <= xb.end + esp)
            {
                return MiNavOverlapRelation.FullOverlap;
            }

            return MiNavOverlapRelation.PartOverlay;
        }


        /// <summary>
        /// 生成投影到TriFace上的pts
        /// </summary>
        /// <param name="rect"></param>
        /// <returns></returns>
        void CreateProjectionToTriFacePts(SimpleVector3[] pts, int count)
        {
            SimpleVector3 pt;
            for (int i = 0; i < count; i++)
            {
                pt = SolveCrossPoint(pts[i], floorGridNormal, vertexs[0], triFaceNormal);
                cellProjYpos[cellProjPtsCount++] = pt.y;
            }
        }


        /// <summary>
        ///  求解射线(p, n)与平面(o, m)的交点d
        ///  一条向量直线: d = p + t * n  1式（n为射线方向，p为起始点， t为长度， d为目标点）
        ///  平面的点积方程表示:   (d - o).m = 0    2式
        ///  (d为平面上任意一点， o为平面上已知的一个点， d-o表示由o点指向d点的向量， m为此平面朝向向量，
        ///  这两个向量互相垂直时，向量点积为0)
        ///  联立1式和2式方程， 可解出射线和平面相交时，t的量值
        ///  (p + t*n - o).m = 0
        ///  p.m + t*n.m - o.m = 0
        ///  t*n.m = o.m - p.m
        ///  t = (o - p).m / n.m
        ///  t代入1式可求出d
        /// </summary>
        public SimpleVector3 SolveCrossPoint(SimpleVector3 p, SimpleVector3 n, SimpleVector3 o, SimpleVector3 m)
        {
            //float value = Vector3d.Dot(n, m);
            //float t = Vector3d.Dot(o - p, m) / value;
            float a = (o.x - p.x) * m.x + (o.y - p.y) * m.y + (o.z - p.z) * m.z;
            float b = n.x * m.x + n.y * m.y + n.z * m.z;
            float t = a / b;

            SimpleVector3 pt = new SimpleVector3();
            pt.y = p.y + n.y * t;
            pt.x = p.x;
            pt.z = p.z;

            //pt.x = p.x + n.x * t;
            //pt.y = p.y + n.y * t;
           // pt.z = p.z + n.z * t;
            return pt;
        }

        /// <summary>
        /// 根据投影Rect生成体素box
        /// </summary>
        /// <param name="cellProjectionRect"></param>
        /// <param name="floorGridCenter">在地板单元格的中心位置坐标</param>
        void CreateVoxBoxToList(int cellx, int cellz)
        {

            float minY = cellProjYpos[0], maxY = cellProjYpos[0];
            for (int i = 1; i < cellProjPtsCount; i++)
            {
                if (cellProjYpos[i] > maxY)
                    maxY = cellProjYpos[i];
                else if (cellProjYpos[i] < minY)
                    minY = cellProjYpos[i];
            }

            //
            float n = minY * voxSpace.invCellHeight;   
            int start = (int)Math.Floor(n);

            //yendCell
            n = maxY * voxSpace.invCellHeight;
            int end = (int)Math.Ceiling(n);
            if (start == end) { end++; }

            solidSpanGroup.AppendVoxBox(cellx, cellz, start, end);
        }

    }
}
