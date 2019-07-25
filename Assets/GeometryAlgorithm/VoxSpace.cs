using LinearAlgebra;
using Mathd;
using System;

namespace Geometry_Algorithm
{
    public class VoxSpace
    {
        public Vector3d boundSize;
        public float cellSize = 0.1f;
        public float cellHeight = 0.02f;
        public float invCellSize = 1 / 0.1f;
        public float invCellHeight = 1 / 0.02f;

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


        /// <summary>
        /// 根据提供的FloorGridCell标号获取这个Cell在VoxSpace中的四个角的Rect坐标
        /// </summary>
        /// <param name="xIdxCell"></param>
        /// <param name="zIdxCell"></param>
        /// <returns></returns>
        public Vector3d[] GetFloorGridCellRect(int xIdxCell, int zIdxCell)
        {
            double xStart = xIdxCell * cellSize;
            double xEnd = xStart + cellSize;

            double zStart = zIdxCell * cellSize;
            double zEnd = zStart + cellSize;

            Vector3d[] rect = new Vector3d[]
            {
                new Vector3d(xStart, 0, zStart),
                new Vector3d(xStart,0, zEnd),
                new Vector3d(xEnd,0, zEnd),
                new Vector3d(xEnd,0, zStart),
            };

            return rect;
        }


        /// <summary>
        /// 获取地面单元格的中心点坐标
        /// </summary>
        /// <param name="xIdxCell"></param>
        /// <param name="zIdxCell"></param>
        /// <returns></returns>
        public Vector3d GetFloorGridCellRectCenterPos(int xIdxCell, int zIdxCell)
        {
            double xStart = xIdxCell * cellSize;
            double zStart = zIdxCell * cellSize;
            return new Vector3d((xStart + cellSize + xStart) / 2, 0, (zStart + cellSize + zStart) / 2);
        }


        /// <summary>
        /// 根据给定的高度位置范围值,获取高度cell的编号范围
        /// </summary>
        /// <param name="minHeightPos"></param>
        /// <param name="maxHeightPos"></param>
        /// <returns></returns>
        public int[] GetWallGridCellIdxRange(double minHeightPos, double maxHeightPos)
        {
            int end;
            double n = minHeightPos * invCellHeight;
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
        public double[] GetWallGridCellPosRange(int cellStartIdx, int cellEndIdx)
        {
            double ystart = cellStartIdx * cellHeight;
            double yend = cellEndIdx * cellHeight;
            return new double[] { ystart, yend };
        }
    }
}
