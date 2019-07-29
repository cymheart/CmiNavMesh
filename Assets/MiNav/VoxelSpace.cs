using LinearAlgebra;
using LinearAlgebra.VectorAlgebra;
using Mathd;
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace MINAV
{
    public class VoxelSpace
    {
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
        int freeSolidSpanCount;
        const int preCreateSolidSpanCount = 100000;

        //
        LinkedList<IntPtr> triVertInfoList = new LinkedList<IntPtr>();
        int freeVertsCount;
        const int preCreateVertsCount = 10000;


        MiNavAABB spaceAABB = new MiNavAABB();
        int xstartCell, xendCell;
        int zstartCell, zendCell;
        int cellxCount;
        int cellzCount;
        public VoxelSpace()
        {
            freeSolidSpanCount = preCreateSolidSpanCount;
        }

        public void CreateSpaceGrids()
        {
            CalFloorGridIdxRange();
            CreateSoildSpanSpaceGrids(cellxCount, cellzCount);
        }


        public void CreateVoxels(SolidSpanGroup solidSpanGroup)
        {
            unsafe
            {
                VoxelTriangle voxTri = new VoxelTriangle(this, solidSpanGroup);
                TriVertsInfo* info;
                int count;
                LinkedListNode<IntPtr> node = triVertInfoList.First;
                for (; node != null; node = node.Next)
                {
                    info = (TriVertsInfo*)node.Value;

                    if (node == triVertInfoList.Last)
                        count = preCreateVertsCount - freeVertsCount;
                    else
                        count = preCreateVertsCount;

                    for (int i = 0; i < count; i++)
                    {
                        voxTri.CreateVoxels(info + i);
                    }
                }
            }
        }

        public int GetFloorGridIdx(int x, int z)
        {
            return (z - zstartCell) * cellzCount + x - xstartCell;
        }


        unsafe void CreateSoildSpanSpaceGrids(int cellxCount, int cellzCount)
        {
            solidSpanGrids = (SolidSpanList*)Marshal.AllocHGlobal(sizeof(SolidSpanList) * cellxCount * cellzCount);
        }


        unsafe public SolidSpan* GetSoildSpan()
        {
            if (freeSolidSpanCount == 0)
            {
                SolidSpan* solidSpans = (SolidSpan*)Marshal.AllocHGlobal(sizeof(SolidSpan) * preCreateSolidSpanCount);
                solidSpansList.AddLast((IntPtr)solidSpans);
                freeSolidSpanCount = preCreateSolidSpanCount;
            }

            SolidSpan* solidSpanPtr = (SolidSpan*)solidSpansList.Last.Value;
            solidSpanPtr += preCreateSolidSpanCount - freeSolidSpanCount;
            freeSolidSpanCount--;
            return solidSpanPtr;
        }

        unsafe TriVertsInfo* GetTriInfo()
        {
            if (freeVertsCount == 0)
            {
                TriVertsInfo* vertsInfo = (TriVertsInfo*)Marshal.AllocHGlobal(sizeof(TriVertsInfo) * preCreateVertsCount);
                triVertInfoList.AddLast((IntPtr)vertsInfo);
                freeVertsCount = preCreateVertsCount;
            }

            TriVertsInfo* triVertsInfoPtr = (TriVertsInfo*)triVertInfoList.Last.Value;
            triVertsInfoPtr += preCreateVertsCount - freeVertsCount;
            freeSolidSpanCount--;
            return triVertsInfoPtr;
        }


        public void TransModelVertexs(Vector[] triFaceVertex)
        {
            unsafe
            {
                TriVertsInfo* triInfo = GetTriInfo();


                MiNavAABB* aabb = &(triInfo->aabb);

                triInfo->vert0.x = (float)triFaceVertex[0].Elements[0];
                triInfo->vert0.y = (float)triFaceVertex[0].Elements[1];
                triInfo->vert0.z = (float)triFaceVertex[0].Elements[2];
                aabb->maxX = triInfo->vert0.x; aabb->minX = triInfo->vert0.x;
                aabb->maxZ = triInfo->vert0.z; aabb->minZ = triInfo->vert0.z;
                aabb->maxY = triInfo->vert0.y; aabb->minY = triInfo->vert0.y;


                triInfo->vert1.x = (float)triFaceVertex[1].Elements[0];
                triInfo->vert1.y = (float)triFaceVertex[1].Elements[1];
                triInfo->vert1.z = (float)triFaceVertex[1].Elements[2];
                if (triInfo->vert1.x > aabb->maxX) { aabb->maxX = triInfo->vert1.x; }
                if (triInfo->vert1.x < aabb->minX) { aabb->minX = triInfo->vert1.x; }
                if (triInfo->vert1.z > aabb->maxZ) { aabb->maxZ = triInfo->vert1.z; }
                if (triInfo->vert1.z < aabb->minZ) { aabb->minZ = triInfo->vert1.z; }

                triInfo->vert2.x = (float)triFaceVertex[2].Elements[0];
                triInfo->vert2.y = (float)triFaceVertex[2].Elements[1];
                triInfo->vert2.z = (float)triFaceVertex[2].Elements[2];
                if (triInfo->vert2.x > aabb->maxX) { aabb->maxX = triInfo->vert2.x; }
                if (triInfo->vert2.x < aabb->minX) { aabb->minX = triInfo->vert2.x; }
                if (triInfo->vert2.z > aabb->maxZ) { aabb->maxZ = triInfo->vert2.z; }
                if (triInfo->vert2.z < aabb->minZ) { aabb->minZ = triInfo->vert2.z; }

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
