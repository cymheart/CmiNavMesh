using Mathd;
using System;
using System.Collections.Generic;

namespace MINAV
{

    /// <summary>
    /// 空间跨距
    /// </summary>
    public class SpaceSpan
    {
        public double startPos = 0;
        public double endPos = 0;

        /// <summary>
        /// 0:可通行
        /// 1:不可通行
        /// </summary>
        public int type = 0;

        /// <summary>
        /// 所属区域编号
        /// </summary>
        public int region = 0;

        /// <summary>
        /// 是否为边缘
        /// </summary>
        public bool isEdge = false;

        /// <summary>
        /// 指向所属单元spaceSpans
        /// </summary>
        public CellSpaceSpans cellSpaceSpans = null;

        /// <summary>
        /// 连通区域
        /// </summary>
        public SpaceSpan[] connectSpan = 
            new SpaceSpan[] { null, null, null, null };
    }


    public class CellSpaceSpans
    {
        public SimpleVector3[] rect;
        public List<SpaceSpan> spaceSpanList;
        public int region;
    }



    /// <summary>
    /// 空间跨距组
    /// </summary>
    public class SpaceSpanGroup
    {
        /// <summary>
        /// 最小角色行走高度
        /// </summary>
        double minWalkHeight = 1f;

        /// <summary>
        /// 最小跨步高度
        /// </summary>
        double minStepHeight = 0.1f;

        /// <summary>
        /// 角色行走半径
        /// </summary>
        double walkRadius = 0.4f;

        /// <summary>
        /// 角色行走半径占据多少个voxSpanBox
        /// </summary>
        int walkRadiusVoxCount = 0;

        static int[] relativeDirMap = { 2, 3, 0, 1 };

        VoxelSpace voxSpace;
        Dictionary<int, CellSpaceSpans> spaceSpanDict = new Dictionary<int, CellSpaceSpans>();

        public SpaceSpanGroup(VoxelSpace voxSpace)
        {
            this.voxSpace = voxSpace;
            walkRadiusVoxCount = CalWalkRadiusVoxCount();
        }


        int CalWalkRadiusVoxCount()
        {
            double n = walkRadius / voxSpace.cellSize;
            int count = (int)(Math.Ceiling(n));
            return count;
        }

        public void CreateSpaceSpanGroup(SolidSpanGroup solidSpanGroup)
        {
            _CreateSpaceSpanGroup(solidSpanGroup);
            CreateSpansConnectRelation();
        }


        public void _CreateSpaceSpanGroup(SolidSpanGroup solidSpanGroup)
        {
            Dictionary<int, LinkedList<SolidSpan>> soildSpanDict = null; // solidSpanGroup.soildSpanDict;
            LinkedList<SolidSpan> solidSpanList;
            SpaceSpan spaceSpan;
            int[] cellIdxs = null;
            SimpleVector3[] voxRect;
            CellSpaceSpans cellSpaceSpans;
            List<SpaceSpan> spaceSpanList;
            double startpos;
            double endpos;

            foreach (var item in soildSpanDict)
            {
               // cellIdxs = solidSpanGroup.GetCellIdxs(item.Key);
                voxRect = voxSpace.GetFloorGridCellRect(cellIdxs[0], cellIdxs[1]);

                solidSpanList = item.Value;
                var node = solidSpanList.First;

                cellSpaceSpans = new CellSpaceSpans();
                spaceSpanList = new List<SpaceSpan>();
                cellSpaceSpans.spaceSpanList = spaceSpanList;
                cellSpaceSpans.rect = voxRect;

                for (; node != null; node = node.Next)
                {
                    startpos = node.Value.endPos;
                    if (node.Next != null)
                        endpos = node.Next.Value.startPos;
                    else
                        endpos = startpos + 1000000;

                    if (endpos - startpos >= minWalkHeight)
                    {
                        spaceSpan = new SpaceSpan();
                        spaceSpan.startPos = startpos;
                        spaceSpan.endPos = endpos;
                        spaceSpan.cellSpaceSpans = cellSpaceSpans;
                        spaceSpan.connectSpan = new SpaceSpan[] { null, null, null, null };
                        spaceSpanList.Add(spaceSpan);
                    }
                }

                spaceSpanDict[item.Key] = cellSpaceSpans;
            }
        }

        /// <summary>
        /// 生成span间的连接关系
        /// </summary>
        void CreateSpansConnectRelation()
        {
            CellSpaceSpans cellSpaceSpans;
            List<SpaceSpan> spaceSpanList;
            List<SpaceSpan> neiSpaceList;
            SpaceSpan span;
            SpaceSpan neiSpan;
            int[] spanIdx;
            int neiKey;
            bool isObstacleDir = false; //是否为有障碍阻止通行的方向
            double realStepHeight;
            double realWalkHeightA, realWalkHeightB;

            foreach (var item in spaceSpanDict)
            {
                spanIdx = GetCellIdxs(item.Key);
                cellSpaceSpans = item.Value;
                spaceSpanList = cellSpaceSpans.spaceSpanList;

                //neiDirIdx以当前span为中心,按左手顺时针: 0←  1↑  2→  3↓
                for (int neiDirIdx = 0; neiDirIdx < 4; neiDirIdx++)
                {          
                    neiKey = GetNeiKey(spanIdx, neiDirIdx);
                    if (neiKey != -1)
                        neiSpaceList = spaceSpanDict[neiKey].spaceSpanList;
                    else
                        neiSpaceList = null;

                    for (int i = 0; i < spaceSpanList.Count; i++)
                    {
                        span = spaceSpanList[i];
                        isObstacleDir = true;

                        if (span.connectSpan[neiDirIdx] != null)
                            continue;

                        for (int j = 0; neiSpaceList != null && j < neiSpaceList.Count; j++)
                        {
                            neiSpan = neiSpaceList[j];

                            //判断邻接span是否在当前检测span的高度范围内
                            if (neiSpan.startPos > span.endPos ||
                                neiSpan.endPos < span.startPos)
                                continue;


                            realStepHeight = Math.Abs(neiSpan.startPos - span.startPos);

                            if (realStepHeight > minStepHeight)
                            {
                                continue;
                            }
                            else
                            {
                                realWalkHeightA = neiSpan.endPos - span.startPos;
                                realWalkHeightB = span.endPos - neiSpan.startPos;

                                if (realWalkHeightA < minWalkHeight ||
                                    realWalkHeightB < minWalkHeight)
                                {
                                    continue;
                                }
                                else
                                {
                                    isObstacleDir = false;

                                    if (neiSpan.type == 0)
                                    {
                                        span.connectSpan[neiDirIdx] = neiSpan;
                                        neiSpan.connectSpan[relativeDirMap[neiDirIdx]] = span;
                                    }

                                    break;
                                }
                            }
                        }


                        //判断是否为此方向有障碍阻止移动
                        if (isObstacleDir)
                        {
                            SetNotWalkSpansByObstacleDir(spanIdx, span, neiDirIdx);
                        }
                    }
                }
            }
        }


        /// <summary>
        /// 根据障碍方向，和角色行走半径，给当前span上的角色半径范围内不可移动的周边spans作标记
        /// </summary>
        /// <param name="spanIdx"></param>
        /// <param name="span"></param>
        /// <param name="obstacleDir">以当前span为中心,按左手顺时针: 0←  1↑  2→  3↓</param>
        void SetNotWalkSpansByObstacleDir(int[] spanIdx, SpaceSpan span, int obstacleDir)
        {
            int x, y;
            int flag = 1;
            if (obstacleDir == 2 || obstacleDir == 3)
                flag = -1;

            double limitEndPosY = span.startPos + minWalkHeight;

            switch (obstacleDir)
            {
                case 0:
                case 2:
                    {
                        for (int i = 0; i < walkRadiusVoxCount; i++)
                        {
                            x = spanIdx[0] + i * flag;
                            SetNotWalkSpans(x, spanIdx[1], span.startPos, limitEndPosY);

                            for (int j = 0; j <= i + 1; j++)
                            {
                                SetNotWalkSpans(x, spanIdx[1] - j, span.startPos, limitEndPosY);
                                SetNotWalkSpans(x, spanIdx[1] + j, span.startPos, limitEndPosY);
                            }
                        }
                    }
                    break;

                case 1:
                case 3:
                    {
                        for (int i = 0; i < walkRadiusVoxCount; i++)
                        {
                            y = spanIdx[0] + i * flag;
                            SetNotWalkSpans(spanIdx[0], y, span.startPos, limitEndPosY);

                            for (int j = 1; j <= i + 1; j++)
                            {
                                SetNotWalkSpans(spanIdx[0] - j, y, span.startPos, limitEndPosY);
                                SetNotWalkSpans(spanIdx[0] + j, y, span.startPos, limitEndPosY);
                            }
                        }
                    }
                    break;
            }
        }

        /// <summary>
        /// 在高度方向上设置那些和limitStartPosY~limitEndPosY范围重叠的span为不可行走类型
        /// </summary>
        /// <param name="spanCellX"></param>
        /// <param name="spanCellZ"></param>
        /// <param name="limitStartPosY"></param>
        /// <param name="limitEndPosY"></param>
        void SetNotWalkSpans(
            int spanCellX, int spanCellZ, 
            double limitStartPosY, double limitEndPosY)
        {
            int key = GetKey(spanCellX, spanCellZ);

            //获取当前floor cell单元上的span链
            List<SpaceSpan> spaceSpanList = spaceSpanDict[key].spaceSpanList;
            SpaceSpan span, neiSpan;

            for(int i=0; i<spaceSpanList.Count; i++)
            {
                span = spaceSpanList[i];

                if(span.type == 1)
                    continue;

                if (span.startPos > limitEndPosY ||            
                    span.endPos < limitStartPosY)
                    continue;

                span.type = 1;

                for (int j = 0; j < 4; j++)
                {
                    neiSpan = span.connectSpan[j];
                    span.connectSpan[j] = null;
                    neiSpan.connectSpan[relativeDirMap[j]] = null;
                }
            }
        }

        void SplitRegions()
        {
            CellSpaceSpans cellSpaceSpans;
            List<SpaceSpan> spaceSpanList;
            SpaceSpan span;
            int region = 1;

            foreach (var item in spaceSpanDict)
            {
                cellSpaceSpans = item.Value;
                spaceSpanList = cellSpaceSpans.spaceSpanList;

                for (int i = 0; i < spaceSpanList.Count; i++)
                {
                    span = spaceSpanList[i];
                    if (span.region > 0)
                        continue;

                    SplitOneRegion(span, region);
                    region++;
                }
            }
        }

        void SplitOneRegion(SpaceSpan startSpan, int region)
        {
            SpaceSpan centerSpan, connectSpan;
            List<SpaceSpan> tmp;
            List<SpaceSpan> frontSpanlist = new List<SpaceSpan>(200);
            List<SpaceSpan> backSpanlist = new List<SpaceSpan>(200);
            frontSpanlist.Add(startSpan);

            while (frontSpanlist.Count > 0)
            {
                for (int i = 0; i < frontSpanlist.Count; i++)
                {
                    centerSpan = frontSpanlist[i];
                    centerSpan.isEdge = false;

                    for (int j = 0; j < 4; j++)
                    {
                        connectSpan = centerSpan.connectSpan[j];

                        if (connectSpan == null ||
                            connectSpan.region > 0 ||
                            connectSpan.type == 1)
                        {
                            centerSpan.isEdge = true;
                            continue;
                        }

                        if (connectSpan.cellSpaceSpans.region == region)
                        {
                            centerSpan.isEdge = true;
                            continue;
                        }

                        connectSpan.region = region;

                        backSpanlist.Add(connectSpan);
                    }
                }

                tmp = frontSpanlist;
                frontSpanlist = backSpanlist;
                backSpanlist = tmp;
                backSpanlist.Clear();
            }
        }

        int GetRelativeDir(int dir)
        {
            return relativeDirMap[dir];
        }

        int GetNeiKey(int[] spanIdx, int dirIdx)
        {
            switch(dirIdx)
            {
                case 0: return GetKey(spanIdx[0] - 1, spanIdx[1]);
                case 1: return GetKey(spanIdx[0], spanIdx[1] + 1);
                case 2: return GetKey(spanIdx[0] + 1, spanIdx[1]);
                case 3: return GetKey(spanIdx[0], spanIdx[1] - 1);
            }

            return -1;
        }

        public int GetKey(int cellx, int cellz)
        {
            if (cellx < 0 || cellz < 0)
                return -1;

            return (cellx << 14) | cellz;
        }

        public int[] GetCellIdxs(int key)
        {
            int[] idxs = new int[] { key >> 14, 0x3FFF & key };
            return idxs;
        }


    }
}
