using Mathd;
using System.Collections.Generic;

namespace Geometry_Algorithm
{
    /// 交点信息列表
    /// </summary>
    public class CrossPointInfo
    {
        public bool isUsed;
        public PointType type = PointType.MiddlePoint;

        /// <summary>
        /// 交点
        /// </summary>
        public Vector3d pt;

        /// <summary>
        /// 在自身边方向上的距离
        /// </summary>
        public double dist;

        public int inoutType;


        public PolySide selfSide;
        /// <summary>
        /// 交点所在的自身多边形点Idx
        /// </summary>
        public int pointIdx;


        public PolySide crossSide;
        /// <summary>
        /// 交点所在的另一个多边形点Idx
        /// </summary>
        public int crossSidePointIdx;


        public CrossPointInfo Copy()
        {
            
            CrossPointInfo info = new CrossPointInfo();
            info.isUsed = isUsed;
            info.type = type;
            info.pt = pt;
            info.dist = dist;
            info.inoutType = inoutType;
            info.selfSide = selfSide;
            info.pointIdx = pointIdx;
            info.crossSide = crossSide;
            info.crossSidePointIdx = crossSidePointIdx;
            return info;
        }

        public void SetUsed()
        {
            isUsed = true;
            CrossPointInfo info = GetCrossSideCrossPointInfo();
            info.isUsed = true;
        }

        /// <summary>
        /// 获取交边的crossPointInfo
        /// </summary>
        /// <returns></returns>
        public CrossPointInfo GetCrossSideCrossPointInfo()
        {
            if (crossSide == null)
                return null;
            return crossSide.crossPointInfoList[crossSidePointIdx];
        }


        /// <summary>
        /// 获取自身边的前一个CrossPointInfo
        /// </summary>
        /// <returns></returns>
        public CrossPointInfo GetSelfSidePrevCrossPointInfo()
        {
            if (pointIdx > 0)
                return selfSide.crossPointInfoList[pointIdx - 1];

            return null;
        }

        /// <summary>
        /// 获取自身边的下一个CrossPointInfo
        /// </summary>
        /// <returns></returns>
        public CrossPointInfo GetSelfSideNextCrossPointInfo()
        {
            if(pointIdx < selfSide.crossPointInfoList.Count - 1)
                return selfSide.crossPointInfoList[pointIdx + 1];

            return null;
        }


    }

    public class PolySide
    {
        /// <summary>
        /// 边的起始点
        /// </summary>
        public Vector3d startpos;

        /// <summary>
        /// 边的方向
        /// </summary>
        public Vector3d dir;

        /// <summary>
        /// 边的长度
        /// </summary>
        public double step;

        /// <summary>
        /// 边对应的垂直方向
        /// </summary>
        public Vector3d vertDir;

        /// <summary>
        /// 和其它多边形的交点信息列表
        /// </summary>
        public List<CrossPointInfo> crossPointInfoList;

        public PolySide Copy()
        {
            PolySide polySide = new PolySide();
            polySide.startpos = startpos;
            polySide.dir = dir;
            polySide.step = step;
            polySide.vertDir = vertDir;

            if(crossPointInfoList != null)
            {
                polySide.crossPointInfoList = new List<CrossPointInfo>();

                for(int i=0; i < crossPointInfoList.Count; i++)
                {
                    polySide.crossPointInfoList[i] = crossPointInfoList[i].Copy();
                }
            }
            return polySide;
        }
    }

    /// <summary>

    public class Poly
    {
        /// <summary>
        /// 多边形朝向
        /// </summary>
        public Vector3d faceNormal;

        /// <summary>
        /// 边列表, 支持内中空多边形， 
        /// sidesList[0]为外边多边形， 后面的多边形集合都是不同闭合内边集合
        /// </summary>
        public List<PolySide[]> sidesList = new List<PolySide[]>();

        /// <summary>
        /// 顶点列表，和边列表对应
        /// </summary>
        public List<Vector3d[]> vertexsList = new List<Vector3d[]>();

        /// <summary>
        /// 多边形自身的外边投影轴的投影范围
        /// </summary>
        public Range[] projRange;


        public Poly Copy()
        {
            Poly poly = new Poly();
            poly.faceNormal = faceNormal;

            for (int i = 0; i < sidesList.Count; i++)
            {
                PolySide[] polySides = new PolySide[sidesList[i].Length];
                for (int j = 0; j < sidesList[i].Length; j++)
                {
                    polySides[j] = sidesList[i][j].Copy();
                }
                poly.sidesList.Add(polySides);
            }

            for (int i = 0; i < sidesList.Count; i++)
            {
                Vector3d[] verts = new Vector3d[vertexsList[i].Length];
                for (int j = 0; j < sidesList[i].Length; j++)
                {
                    verts[j] = vertexsList[i][j];
                }
                poly.vertexsList.Add(verts);
            }

            return poly;
        }

 
        /// <summary>
        /// 获取一个未使用的交点信息
        /// </summary>
        /// <returns></returns>
        public CrossPointInfo GetOneUnUsedCrossPointInfo()
        {
            PolySide[] sides;
            List<CrossPointInfo> cpInfoList;

            for (int i = 0; i < sidesList.Count; i++)
            {
                sides = sidesList[i];
                for (int j = 0; j < sides.Length; j++)
                {
                    cpInfoList = sides[j].crossPointInfoList;

                    if (cpInfoList == null ||
                        cpInfoList.Count <= 2)
                        continue;

                    for (int k = 1; k < cpInfoList.Count - 1; k++)
                    {
                        if (cpInfoList[k].isUsed == false)
                            return cpInfoList[k];
                    }
                }
            }

            return null;
        }


        /// <summary>
        /// 获取一个未使用的端点信息
        /// 入点还是出点
        /// </summary>
        /// <returns></returns>
        public CrossPointInfo GetOneUnUsedEndPointInfo(int inout)
        {
            PolySide[] sides;
            List<CrossPointInfo> cpInfoList;

            for (int i = 0; i < sidesList.Count; i++)
            {
                sides = sidesList[i];
                for (int j = 0; j < sides.Length; j++)
                {
                    cpInfoList = sides[j].crossPointInfoList;
                    if (cpInfoList[0].isUsed == false && cpInfoList[0].inoutType == inout)
                        return cpInfoList[0];
                    else if (cpInfoList[cpInfoList.Count - 1].isUsed == false && cpInfoList[0].inoutType == inout)
                        return cpInfoList[cpInfoList.Count - 1];
                }
            }

            return null;
        }

    }
}
