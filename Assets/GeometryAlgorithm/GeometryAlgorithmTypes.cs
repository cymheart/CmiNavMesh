
namespace Geometry_Algorithm
{
    /// <summary>
    /// 多边形中边的类型
    /// </summary>
    public enum PloySideType
    {
        /// <summary>
        /// 所有边
        /// </summary>
        Allside,

        /// <summary>
        /// 外边
        /// </summary>
        Outside,

        /// <summary>
        /// 内边
        /// </summary>
        Inside,
    }

    public enum PointType
    {
        /// <summary>
        /// 中间点
        /// </summary>
        MiddlePoint,

        /// <summary>
        /// 端点
        /// </summary>
        EndPoint
    }

    public enum BooleanType
    {
        /// <summary>
        /// 交集
        /// </summary>
        Intersection,

        /// <summary>
        /// 并集
        /// </summary>
        Union,

        /// <summary>
        /// 差集
        /// </summary>
        Sub
    }

    public enum DirCmpInfo
    {
        /// <summary>
        /// 互相垂直
        /// </summary>
        Vertical = 0,

        /// <summary>
        /// 朝向基本相同
        /// </summary>
        Same = 1,

        /// <summary>
        /// 朝向基本不同
        /// </summary>
        Different = -1,

    }


    public enum OverlapRelation
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

    }
    public struct AABB
    {
        public double minX, maxX;
        public double minZ, maxZ;
        public double minY, maxY;
    }


    public struct Range
    {
        public double min, max;
    }
}
