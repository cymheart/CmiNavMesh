using LinearAlgebra;
using LinearAlgebra.VectorAlgebra;
using Mathd;
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Geometry_Algorithm
{
    public class VoxSpace
    {
        public Vector3d boundSize;
        public float cellSize = 0.5f;
        public float cellHeight = 0.01f;
        public float invCellSize = 1 / 0.5f;
        public float invCellHeight = 1 / 0.01f;

        /// <summary>
        /// 求点pa从坐标系A转换到另一个坐标系B后点的坐标位置pb，转换原理:
        /// 1.先把A坐标系的基轴向量转换为B坐标系中单位向量AToB_BasicAxis
        /// 2.再把pa各分量乘以AToB_BasicAxis各个对应基轴，获得pa在坐标系B中的向量pab_x, pab_y, pab_z 
        /// 3.获取以B坐标系作为参照， A坐标系原点相对B原点的向量vab
        /// 4.vab + pab_x + pab_y + pab_z 即为 pb
        /// 5.分解开来: 
        /// pb.x = pab_x.x + pab_y.x + pab_z.x + vab.x
        /// pb.y = pab_x.y + pab_y.y + pab_z.y + vab.y
        /// pb.z = pab_x.z + pab_y.z + pab_z.z + vab.z
        /// 
        /// 组成变换矩阵C形式:(pa为行向量，变换矩阵C左乘pa得到pb)
        ///  pab_x.x   pab_y.x   pab_z.x
        ///  pab_x.y   pab_y.y   pab_z.y
        ///  pab_x.z   pab_y.z   pab_z.z
        ///  vab.x     vab.y     vab.z        
        /// </summary>
        public Matrix worldToVoxSpace = Matrix.Eye(4);
        public Matrix voxSpaceToWorld = Matrix.Eye(4);

        public unsafe SolidSpanList* solidSpanGrids;

        LinkedList<IntPtr> solidSpansList = new LinkedList<IntPtr>();
        LinkedListNode<IntPtr> curtSolidSpansNode;
        int freeSolidSpanCount;
        const int preCreateSolidSpanCount = 100000;

        //
        LinkedList<IntPtr> vertsList = new LinkedList<IntPtr>();
        LinkedListNode<IntPtr> curtVertsNode;
        int freeVertsCount;
        const int preCreateVertsCount = 10000 * 3;

        LinkedList<IntPtr> triAABBList = new LinkedList<IntPtr>();
        LinkedListNode<IntPtr> curtTriAABBNode;
        int freeTriAABBCount;
        const int preCreateTriAABBCount = 10000;


        AABB spaceAABB = new AABB();
        int xstartCell, xendCell;
        int zstartCell, zendCell;
        int cellxCount;
        int cellzCount;
        public VoxSpace()
        {
            freeSolidSpanCount = preCreateSolidSpanCount;
        }

        public void CreateSpaceGrids()
        {
            CalFloorGridIdxRange();
            CreateSoildSpanSpaceGrids(cellxCount, cellzCount);
        }

        public int GetFloorGridIdx(int x, int z)
        {
            return (z - zstartCell) * cellzCount + x - xstartCell;
        }


        void CreateSoildSpanSpaceGrids(int cellxCount, int cellzCount)
        {
            unsafe
            {
                solidSpanGrids = (SolidSpanList*)Marshal.AllocHGlobal(sizeof(SolidSpanList) * cellxCount * cellzCount);
            }
        }


        unsafe public SolidSpan* GetSoildSpan()
        {
            if (freeSolidSpanCount == 0)
            {
                SolidSpan* solidSpans = (SolidSpan*)Marshal.AllocHGlobal(sizeof(SolidSpan) * preCreateSolidSpanCount);
                solidSpansList.AddLast((IntPtr)solidSpans);
                curtSolidSpansNode = solidSpansList.Last;
                freeSolidSpanCount = preCreateSolidSpanCount;
            }

            SolidSpan* solidSpanPtr = (SolidSpan*)curtSolidSpansNode.Value;
            solidSpanPtr += preCreateSolidSpanCount - freeSolidSpanCount;
            freeSolidSpanCount--;
            return solidSpanPtr;
        }

        unsafe SimpleVector3* GetVertexs3()
        {
            if (freeVertsCount == 0)
            {
                SimpleVector3* verts = (SimpleVector3*)Marshal.AllocHGlobal(sizeof(SimpleVector3) * preCreateVertsCount);
                vertsList.AddLast((IntPtr)verts);
                curtVertsNode = vertsList.Last;
                freeVertsCount = preCreateVertsCount;
            }

            SimpleVector3* vertPtr = (SimpleVector3*)curtVertsNode.Value;
            vertPtr += preCreateSolidSpanCount - freeSolidSpanCount;
            freeSolidSpanCount -= 3;
            return vertPtr;
        }

        unsafe AABB* GetTriAABB()
        {
            if (freeVertsCount == 0)
            {
                AABB* aabbs = (AABB*)Marshal.AllocHGlobal(sizeof(AABB) * preCreateTriAABBCount);
                triAABBList.AddLast((IntPtr)aabbs);
                curtTriAABBNode = triAABBList.Last;
                freeTriAABBCount = preCreateTriAABBCount;
            }

            AABB* triAABBPtr = (AABB*)curtTriAABBNode.Value;
            triAABBPtr += preCreateTriAABBCount - freeTriAABBCount;
            freeTriAABBCount--;
            return triAABBPtr;
        }


        public void TransModelVertexs(Vector[] triFaceVertex)
        {
            unsafe
            {
                SimpleVector3* vertexs = GetVertexs3();
                AABB* aabb = GetTriAABB();

                vertexs[0].x = (float)triFaceVertex[0].Elements[0];
                vertexs[0].y = (float)triFaceVertex[0].Elements[1];
                vertexs[0].z = (float)triFaceVertex[0].Elements[2];

                aabb->maxX = vertexs[0].x; aabb->minX = vertexs[0].x;
                aabb->maxZ = vertexs[0].z; aabb->minZ = vertexs[0].z;
                aabb->maxY = vertexs[0].y; aabb->minY = vertexs[0].y;

                for (int i = 1; i < triFaceVertex.Length; i++)
                {
                    vertexs[i].x = (float)triFaceVertex[i].Elements[0];
                    vertexs[i].y = (float)triFaceVertex[i].Elements[1];
                    vertexs[i].z = (float)triFaceVertex[i].Elements[2];

                    if (vertexs[i].x > aabb->maxX) { aabb->maxX = vertexs[i].x; }
                    if (vertexs[i].x < aabb->minX) { aabb->minX = vertexs[i].x; }
                    if (vertexs[i].z > aabb->maxZ) { aabb->maxZ = vertexs[i].z; }
                    if (vertexs[i].z < aabb->minZ) { aabb->minZ = vertexs[i].z; }
                }

                vertexs[3].x = vertexs[0].x;
                vertexs[3].y = vertexs[0].y;
                vertexs[3].z = vertexs[0].z;

                if (aabb->maxX > spaceAABB.maxX) { spaceAABB.maxX = aabb->maxX; }
                if (aabb->minX < spaceAABB.minX) { spaceAABB.minX = aabb->minX; }
                if (aabb->maxZ > spaceAABB.maxZ) { spaceAABB.maxZ = aabb->maxZ; }
                if (aabb->minZ < spaceAABB.minZ) { spaceAABB.minZ = aabb->minZ; }
            }

        }


        /// <summary>
        /// 计算floorGrid的格子范围
        /// </summary>
        void CalFloorGridIdxRange()
        {
            //xstartCell
            float n = spaceAABB.minX * invCellSize;
            xstartCell = (int)Math.Floor(n);

            //xendCell
            n = spaceAABB.maxX * invCellSize;
            xendCell = (int)Math.Ceiling(n);
            if (xstartCell == xendCell) xendCell++;

            //zstartCell
            n = spaceAABB.minZ * invCellSize;
            zstartCell = (int)Math.Floor(n);

            //zendCell
            n = spaceAABB.maxZ * invCellSize;
            zendCell = (int)Math.Ceiling(n);
            if (zstartCell == zendCell) zendCell++;

            //
            cellxCount = xendCell - xstartCell;
            cellzCount = zendCell - zstartCell;
        }

        /// <summary>
        /// 根据提供的FloorGridCell标号获取这个Cell在VoxSpace中的四个角的Rect坐标
        /// </summary>
        /// <param name="xIdxCell"></param>
        /// <param name="zIdxCell"></param>
        /// <returns></returns>
        public SimpleVector3[] GetFloorGridCellRect(int xIdxCell, int zIdxCell)
        {
            float xStart = xIdxCell * cellSize;
            float xEnd = xStart + cellSize;

            float zStart = zIdxCell * cellSize;
            float zEnd = zStart + cellSize;

            SimpleVector3[] rect = new SimpleVector3[]
            {
                new SimpleVector3(xStart, 0, zStart),
                new SimpleVector3(xStart,0, zEnd),
                new SimpleVector3(xEnd,0, zEnd),
                new SimpleVector3(xEnd,0, zStart),
            };

            return rect;
        }


        /// <summary>
        /// 获取地面单元格的中心点坐标
        /// </summary>
        /// <param name="xIdxCell"></param>
        /// <param name="zIdxCell"></param>
        /// <returns></returns>
        public SimpleVector3 GetFloorGridCellRectCenterPos(int xIdxCell, int zIdxCell)
        {
            float xStart = xIdxCell * cellSize;
            float zStart = zIdxCell * cellSize;
            return new SimpleVector3((xStart + cellSize + xStart) / 2, 0, (zStart + cellSize + zStart) / 2);
        }


        /// <summary>
        /// 根据给定的高度位置范围值,获取高度cell的编号范围
        /// </summary>
        /// <param name="minHeightPos"></param>
        /// <param name="maxHeightPos"></param>
        /// <returns></returns>
        public int[] GetWallGridCellIdxRange(float minHeightPos, float maxHeightPos)
        {
            int end;
            float n = minHeightPos * invCellHeight;
            int start = (int)Math.Floor(n);

            //yendCell
            n = maxHeightPos * invCellHeight;
            end = (int)(Math.Ceiling(n));
            if (start == end) { end++; }

            return new int[] { start, end };
        }


        /// <summary>
        /// 根据高度Cell的编号范围，获取高度的位置范围
        /// </summary>
        /// <param name="cellStartIdx"></param>
        /// <param name="cellEndIdx"></param>
        /// <returns></returns>
        public float[] GetWallGridCellPosRange(int cellStartIdx, int cellEndIdx)
        {
            float ystart = cellStartIdx * cellHeight;
            float yend = cellEndIdx * cellHeight;
            return new float[] { ystart, yend };
        }

      

       

    }
}
