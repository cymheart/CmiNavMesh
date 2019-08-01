
namespace MINAV
{
    public enum MiNavOverlapRelation
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

    public struct MiNavAABB
    {
        public float minX, maxX;
        public float minZ, maxZ;
        public float minY, maxY;
    }

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

    public struct LineParam
    {
        public float m, b;
        public float ystart, yend;
    }

    public unsafe struct TriVertsInfo
    {
        public SimpleVector3 vert0;
        public SimpleVector3 vert1;
        public SimpleVector3 vert2;
        public MiNavAABB aabb;
    }

  
}

