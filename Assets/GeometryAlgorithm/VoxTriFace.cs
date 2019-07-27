using LinearAlgebra.VectorAlgebra;
using Mathd;
using System;
using System.Collections.Generic;

namespace Geometry_Algorithm
{
    public struct SimpleVector3
    {
        public float x, y, z;
        public SimpleVector3(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }

    public struct CellLineRange
    {
        public float start, end;
    }

    public class VoxTriFace
    {
        VoxSpace voxSpace;
        readonly float esp = 0.00001f;

        SimpleVector3[] vertexs = new SimpleVector3[4]
        {
            new SimpleVector3(0,0,0),
            new SimpleVector3(0,0,0),
            new SimpleVector3(0,0,0),
            new SimpleVector3(0,0,0)
        };

        SimpleVector3[] floorCellRect = new SimpleVector3[4]
      {
            new SimpleVector3(0, 0, 0),
            new SimpleVector3(0, 0, 0),
            new SimpleVector3(0, 0, 0),
            new SimpleVector3(0, 0, 0)
      };

        SimpleVector3[] realFloorCellRect = new SimpleVector3[4]
   {
            new SimpleVector3(0, 0, 0),
            new SimpleVector3(0, 0, 0),
            new SimpleVector3(0, 0, 0),
            new SimpleVector3(0, 0, 0)
   };

        SimpleVector3[] cellProjPoints = new SimpleVector3[10];
   
        float[] m = new float[3];
        float[] n = new float[3];

        float[] a = new float[3];
        float[] b = new float[3];

        float[] a2 = new float[3];
        float[] b2 = new float[3];

        SimpleVector3[] crossPt = new SimpleVector3[10];
        int crossPtCount = 0;

        int[] vertCellX = new int[3];
        int[] vertCellZ = new int[3];

        float invRandomExNum = 1 / 100000;
        SimpleVector3 triFaceNormal;
        SimpleVector3 floorGridNormal = new SimpleVector3(0,1,0);
        DirCmpInfo faceDirType = DirCmpInfo.Same;

        public List<VoxBox> voxBoxList = new List<VoxBox>(1000);
        AABB aabb = new AABB() {minX = 99999f, maxX = 0f, minZ = 99999f, maxZ = 0f };
        int xstartCell, xendCell;
        int zstartCell, zendCell;

        List<CellLineRange> xcolZRangeList = new List<CellLineRange>(2000);
        List<CellLineRange> zrowXRangeList = new List<CellLineRange>(2000);
        List<CellLineRange> zcolYRangeList = new List<CellLineRange>(2000);

        public int totalCount = 0;

        public VoxTriFace(VoxSpace voxSpace)
        {
            SetVoxSpace(voxSpace);
        }

        public void SetVoxSpace(VoxSpace voxSpace)
        {
            this.voxSpace = voxSpace;
        }

        public void Clear()
        {
            crossPtCount = 0;
            totalCount = 0;
            voxBoxList.Clear();
            aabb = new AABB() { minX = 99999f, maxX = 0f, minZ = 99999f, maxZ = 0f };
        }

        public void TransTriFaceWorldVertexToVoxSpace(Vector[] triFaceWorldVertex)
        {
            _TransTriFaceWorldVertexToVoxSpace(triFaceWorldVertex);
            CalFloorGridIdxRange();
            CreateProjToFloorTriFaceVertexs();
            CreateVertexsProjFloorSidesParams();
            CreateVertexsProjZYPlaneSidesParams();
            CreateCellLines();

            //
            CreateFloorGridProjTriFaceVoxBox();   
        }

        /// <summary>
        /// 转换三角面的世界顶点到体素空间
        /// </summary>
        /// <param name="triFaceWorldVertex"></param>
        void _TransTriFaceWorldVertexToVoxSpace(Vector[] triFaceVertex)
        {
            for(int i=0;i < triFaceVertex.Length; i++)
            {
                vertexs[i].x = (float)triFaceVertex[i].Elements[0];
                vertexs[i].y = (float)triFaceVertex[i].Elements[1];
                vertexs[i].z = (float)triFaceVertex[i].Elements[2];

                if (vertexs[i].x > aabb.maxX) { aabb.maxX = vertexs[i].x; }
                if (vertexs[i].x < aabb.minX) { aabb.minX = vertexs[i].x; }
                if (vertexs[i].z > aabb.maxZ) { aabb.maxZ = vertexs[i].z; }
                if (vertexs[i].z < aabb.minZ) { aabb.minZ = vertexs[i].z; }
                if (vertexs[i].y > aabb.maxY) { aabb.maxY = vertexs[i].y; }
                if (vertexs[i].y < aabb.minY) { aabb.minY = vertexs[i].y; }        
            }

            CalTriFaceNormal();

            if(faceDirType == DirCmpInfo.Vertical)
            {
                Random ran = new Random();
                float pos;
                
                for (int i=0; i < vertexs.Length; i++)
                {
                    pos = ran.Next(10, 60) * invRandomExNum;
                    vertexs[i].x += pos;
                    vertexs[i].y += pos;
                    vertexs[i].z += pos;
                }

                CalTriFaceNormal();
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
        /// 生成三角面到地面的投影顶点
        /// </summary>
        void CreateProjToFloorTriFaceVertexs()
        {
            for (int i = 0; i < 3; i++)
            {
                float cell = vertexs[i].x * voxSpace.invCellSize;
                vertCellX[i] = (int)Math.Floor(cell);
                cell = vertexs[i].z * voxSpace.invCellSize;
                vertCellZ[i] = (int)Math.Floor(cell);
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

        void CreateCellLines()
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

                    if (!(z >= vertexs[i].z && z <= vertexs[i + 1].z) &&
                        !(z >= vertexs[i + 1].z && z <= vertexs[i].z))
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

                    if (!(x >= vertexs[i].x && x <= vertexs[i + 1].x) &&
                        !(x >= vertexs[i + 1].x && x <= vertexs[i].x))
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

            //
            float y;
            z = zstartCell * cellSize - cellSize;
            for (int j = zstartCell; j <= zendCell; j++)
            {
                z += cellSize;
                min = 999999; max = -999999;

                for (int i = 0; i < 3; i++)
                {
                    if (a2[i] == 0)
                        continue;

                    if (!(z >= vertexs[i].z && z <= vertexs[i + 1].z) &&
                        !(z >= vertexs[i + 1].z && z <= vertexs[i].z))
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

                cellLineRange = new CellLineRange() { start = min, end = max };
                zcolYRangeList.Add(cellLineRange);
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

                    CreateFloorGridCellProjTriFaceVoxBox(x, z);
                }
            }
        }

        /// <summary>
        /// 生成指定地面网格单元格投影到TriFace上的体素Box
        /// </summary>
        /// <param name="cellx"></param>
        /// <param name="cellz"></param>
        void CreateFloorGridCellProjTriFaceVoxBox(int cellx, int cellz)
        {
            if (cellx == 4 && cellz == 6)
            {
                int a;
                a = 3;
            }
            totalCount++;

            OverlapRelation relation = GetOverlapRelation(cellx, cellz);

            if (relation == OverlapRelation.NotOverlay)
                return;

            if (relation == OverlapRelation.FullOverlap)
            {
              //  CreateProjectionToTriFacePts(floorCellRect, 4);
               // CreateVoxBoxToList(cellProjPoints, 4, cellx, cellz);
            }
            else
            {
               // CreateProjectionToTriFacePts(crossPt, crossPtCount);
               // CreateVoxBoxToList(cellProjPoints, crossPtCount, cellx, cellz);
            }
        }

        /// <summary>
        /// 获取单元格与投影三角形的覆盖关系
        /// </summary>
        /// <returns></returns>
        OverlapRelation GetOverlapRelation(int cellx, int cellz)
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
                return OverlapRelation.NotOverlay;
            }

            if(floorCellRect[0].x >= xa.start && floorCellRect[0].x<= xa.end &&
               floorCellRect[3].x >= xa.start && floorCellRect[3].x <= xa.end &&
               floorCellRect[1].x >= xb.start && floorCellRect[1].x <= xb.end &&
               floorCellRect[2].x >= xb.start && floorCellRect[2].x <= xb.end)
            {
                return OverlapRelation.FullOverlap;
            }

            //
            crossPtCount = 0;
            if (floorCellRect[0].x >= xa.start && floorCellRect[0].x <= xa.end)
                crossPt[crossPtCount++] = floorCellRect[0];
            if (floorCellRect[3].x >= xa.start && floorCellRect[3].x <= xa.end)
                crossPt[crossPtCount++] = floorCellRect[3];
            if (floorCellRect[1].x >= xb.start && floorCellRect[1].x <= xb.end)
                crossPt[crossPtCount++] = floorCellRect[1];
            if (floorCellRect[2].x >= xb.start && floorCellRect[2].x <= xb.end)
                crossPt[crossPtCount++] = floorCellRect[2];

            if (xa.start >= floorCellRect[0].x && xa.start <= floorCellRect[3].x)
                crossPt[crossPtCount++] = new SimpleVector3(xa.start, 0, cellz * voxSpace.cellSize);
            if (xa.end >= floorCellRect[0].x && xa.end <= floorCellRect[3].x)
                crossPt[crossPtCount++] = new SimpleVector3(xa.end, 0, cellz * voxSpace.cellSize);
            if (xb.start >= floorCellRect[1].x && xb.start <= floorCellRect[2].x)
                crossPt[crossPtCount++] = new SimpleVector3(xb.start, 0, (cellz+1) * voxSpace.cellSize);
            if (xb.end >= floorCellRect[1].x && xb.end <= floorCellRect[2].x)
                crossPt[crossPtCount++] = new SimpleVector3(xb.end, 0, (cellz+1) * voxSpace.cellSize);

            if (za.start >= floorCellRect[0].z && za.start <= floorCellRect[1].z)
                crossPt[crossPtCount++] = new SimpleVector3(cellx * voxSpace.cellSize, 0, za.start);
            if (za.end >= floorCellRect[0].z && za.end <= floorCellRect[1].z)
                crossPt[crossPtCount++] = new SimpleVector3(cellx * voxSpace.cellSize, 0, za.end);
            if (zb.start >= floorCellRect[3].z && zb.start <= floorCellRect[2].z)
                crossPt[crossPtCount++] = new SimpleVector3((cellx+1) * voxSpace.cellSize, 0, zb.start);
            if (zb.end >= floorCellRect[3].z && zb.end <= floorCellRect[2].z)
                crossPt[crossPtCount++] = new SimpleVector3((cellx+1) * voxSpace.cellSize, 0, zb.end);

            for (int i = 0; i < 3; i++)
            {
                if (vertCellX[i] == cellx && vertCellZ[i] == cellz)
                {
                    crossPt[crossPtCount++] = vertexs[i];
                    break;
                }
            }


            return OverlapRelation.PartOverlay;
        }

        /// <summary>
        /// 生成投影到TriFace上的pts
        /// </summary>
        /// <param name="rect"></param>
        /// <returns></returns>
        void CreateProjectionToTriFacePts(SimpleVector3[] pts, int count)
        {
            for(int i=0; i<count; i++)
                cellProjPoints[i] = SolveCrossPoint(pts[i], floorGridNormal, vertexs[0], triFaceNormal);
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
        void CreateVoxBoxToList(SimpleVector3[] projectionPts, int count, int cellx, int cellz)
        {
            float minY = projectionPts[0].y, maxY = projectionPts[0].y;
            for (int i = 1; i < count; i++)
            {
                if (projectionPts[i].y > maxY)
                    maxY = projectionPts[i].y;
                else if (projectionPts[i].y < minY)
                    minY = projectionPts[i].y;
            }

            //
            float n = minY * voxSpace.invCellHeight;
            int start = (int)Math.Floor(n);

            //yendCell
            n = maxY * voxSpace.invCellHeight;
            int end = (int)(Math.Ceiling(n));
            if (start == end) { end++; }

           // VoxBox voxBox = new VoxBox(cellx.ToString() +" " + cellz.ToString(), voxSpace, cellx, cellz, start, end);
           // voxBoxList.Add(voxBox);
        }

    }
}
