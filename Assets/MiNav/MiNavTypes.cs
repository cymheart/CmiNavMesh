
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
        public float offsety;
    }

    public unsafe struct TriVertsInfo
    {
        public SimpleVector3 vert0;
        public SimpleVector3 vert1;
        public SimpleVector3 vert2;
        public MiNavAABB aabb;
    }

    /// <summary>
    /// 实心跨距
    /// </summary>
    public unsafe struct SolidSpan
    {
        public float ystartPos;
        public float yendPos;

        public int ystartCellIdx;
        public int yendCellIdx;

        public SolidSpan* prev;
        public SolidSpan* next;
    }

    public unsafe struct SolidSpanList
    {
        public int floorCellIdxX;
        public int floorCellIdxZ;
        public SolidSpan* first;
        public SolidSpan* last;

        public void AddFirst(SolidSpan* node)
        {
            if (first != null)
            {
                first->prev = node;
                node->next = first;
                first = node;
            }
            else
            {
                first = node;
                last = node;
            }
        }

        public void AddLast(SolidSpan* node)
        {
            if (last != null)
            {
                last->next = node;
                node->prev = last;
                last = node;
            }
            else
            {
                first = node;
                last = node;
            }
        }

        public void AddAfter(SolidSpan* after, SolidSpan* node)
        {
            if (after != null)
            {
                SolidSpan* tmp = after->next;
                after->next = node;
                node->prev = after;
                node->next = tmp;

                if (tmp != null)
                {
                    tmp->prev = node;
                }
                else
                {
                    last = node;
                }
            }
            else
            {
                AddFirst(node);
            }
        }

        public void Remove(SolidSpan* node)
        {
            SolidSpan* prev = node->prev;
            SolidSpan* next = node->next;

            if (prev != null)
                prev->next = next;
            else
                first = next;

            if (next != null)
                next->prev = prev;
            else
                last = prev;
        }
    }


    /// <summary>
    /// 空间跨距
    /// </summary>
    public unsafe struct SpaceSpan
    {
        public float startPos;
        public float endPos;

        /// <summary>
        /// 0:可通行
        /// 1:不可通行
        /// </summary>
        public int type;

        /// <summary>
        /// 所属区域编号
        /// </summary>
        public int region;

        /// <summary>
        /// 是否为边缘
        /// </summary>
        public bool isEdge;

        /// <summary>
        /// 指向所属单元spaceSpans
        /// </summary>
        public SpaceSpanList* cellSpaceSpans;

        /// <summary>
        /// 连通区域
        /// </summary>
        public SpaceSpan* connectSpan;

        public SpaceSpan* prev;
        public SpaceSpan* next;
    }

    public unsafe struct SpaceSpanList
    {
        public SimpleVector3 v0;
        public SimpleVector3 v1;
        public SimpleVector3 v2;
        public SimpleVector3 v3;
        public int region;
        public SpaceSpan* first;
        public SpaceSpan* last;

        public void AddFirst(SpaceSpan* node)
        {
            if (first != null)
            {
                first->prev = node;
                node->next = first;
                first = node;
            }
            else
            {
                first = node;
                last = node;
            }
        }

        public void AddLast(SpaceSpan* node)
        {
            if (last != null)
            {
                last->next = node;
                node->prev = last;
                last = node;
            }
            else
            {
                first = node;
                last = node;
            }
        }

        public void AddAfter(SpaceSpan* after, SpaceSpan* node)
        {
            if (after != null)
            {
                SpaceSpan* tmp = after->next;
                after->next = node;
                node->prev = after;
                node->next = tmp;

                if (tmp != null)
                {
                    tmp->prev = node;
                }
                else
                {
                    last = node;
                }
            }
            else
            {
                AddFirst(node);
            }
        }

        public void Remove(SpaceSpan* node)
        {
            SpaceSpan* prev = node->prev;
            SpaceSpan* next = node->next;

            if (prev != null)
                prev->next = next;
            else
                first = next;

            if (next != null)
                next->prev = prev;
            else
                last = prev;
        }
    }

}

