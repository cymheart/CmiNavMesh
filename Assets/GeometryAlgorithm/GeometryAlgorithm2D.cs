using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace Geometry_Algorithm
{
    public partial class GeometryAlgorithm
    {
        public float Cross2D(Vector3 v1, Vector3 v2)
        {
            return v1.x * v2.z - v1.z * v2.x;
        }

        public float Dot2D(Vector3 v1, Vector3 v2)
        {
            return v1.x * v2.x + v1.z * v2.z;
        }



        public bool IsInsidePoly2D(Poly poly, Vector3 pt, PloySideType checkSideType = PloySideType.Allside)
        {
            PolySide[] polySides;
            List<PolySide[]> polySidesList = poly.sidesList;
            Vector3 polyFaceNormal = poly.faceNormal;

            //判断从点引出的平行于sides[0]方向的正向射线是否与其它边相交
            Vector3 n = polySidesList[0][0].startpos - pt;
            int crossPtCount = 0;
            Vector3 crossPt;
            PolySide polySide2 = new PolySide();
            polySide2.startpos = pt;
            polySide2.dir = n;
            polySide2.step = 1000000;

            for (int j = 0; j < polySidesList.Count; j++)
            {
                if (checkSideType == PloySideType.Outside) { if (j == 1) break; }
                else if (checkSideType == PloySideType.Inside) { if (j == 0) continue; }

                polySides = polySidesList[j];

                for (int i = 0; i < polySides.Length; i++)
                {
                    crossPtCount += SolvePolySideCrossPoint2D(polySides[i], polySide2, true, out crossPt);
                }
            }

            //判断是偶数个交点，还是奇数个交点
            if (crossPtCount % 2 == 0)
                return false;

            return true;
        }


        /// <summary>
        /// 求解多边形边集的所有交点(此功能不比较边的方向，一般比较边的方向用于点是否在多边形中的判断)
        /// </summary>
        /// <param name="polySidesA">多边形A的边集</param>
        /// <param name="polySidesB">多边形B的边集</param>
        /// <param name="polyFaceNormal">多边形面向法线</param>
        /// <returns></returns>
        public Vector3[] SolvePolySidesCrossPoints2D(PolySide[] polySidesA, PolySide[] polySidesB)
        {
            List<Vector3> pts = new List<Vector3>();
            Vector3 crossPt;
            int crossPtCount;

            for (int i = 0; i < polySidesA.Length; i++)
            {
                for (int j = 0; j < polySidesB.Length; j++)
                {
                    crossPtCount = SolvePolySideCrossPoint2D(polySidesA[i], polySidesB[j], false, out crossPt);
                    if (crossPtCount == 0)
                        continue;

                    pts.Add(crossPt);
                }
            }

            return pts.ToArray();
        }

        /// <summary>
        /// 求解多边形边的交点(此功能不比较边的方向，一般比较边的方向用于点是否在多边形中的判断)
        /// </summary>
        /// <param name="polySide1">多边形的一条边</param>
        /// <param name="polySide2">多边形的一条边</param>
        /// <param name="polyFaceNormal">多边形面向法线</param>
        /// <param name="crossPt">输出射线和边的交点</param>
        /// <returns>返回值为 1: 有1个交点， 0：没有交点</returns>
        public int SolvePolySideCrossPoint2D(PolySide polySide1, PolySide polySide2, out Vector3 crossPt)
        {
            return SolvePolySideCrossPoint2D(polySide1, polySide2, false, out crossPt);
        }

        /// <summary>
        /// 求解多边形边的交点，
        /// 是否比较边的端点（比较边的端点一般用于点是否在多边形中的判断）
        /// </summary>
        /// <param name="polySide1">多边形的一条边</param>
        /// <param name="polySide2">多边形的一条边</param>
        /// <param name="isCmpSideEndPoint">是否比较边的端点（比较边的端点一般用于点是否在多边形中的判断）</param>
        /// <param name="crossPt">两条边的交点</param>
        /// <returns>返回值为 1: 有1个交点， 0：没有交点</returns>
        public int SolvePolySideCrossPoint2D(PolySide polySide1, PolySide polySide2, bool isCmpSideEndPoint, out Vector3 crossPt)
        {
            Vector3 m = new Vector3(polySide1.dir.z, 0, -polySide1.dir.x);
            bool isHavCrossPt = SolveCrossPoint2D(polySide2.startpos, polySide2.dir, polySide1.startpos, m, out crossPt);

            //两条边互相平行
            if (!isHavCrossPt)
                return 0;

            float step;
            Vector3 crossPtToPtVec = crossPt - polySide2.startpos;
            CmpParallelVecDir(crossPtToPtVec, polySide2.dir, out step);
            //交点crossPt不在polySide2边的范围内
            if (step < 0 || step > polySide2.step + esp)
                return 0;

            crossPtToPtVec = crossPt - polySide1.startpos;

            //交点与端点重合
            if (IsZero(crossPtToPtVec))
            {
                if (isCmpSideEndPoint)
                {
                    Vector3 sideEndPt = polySide1.startpos + polySide1.dir * polySide1.step;
                    float v = Cross2D(sideEndPt - polySide2.startpos, polySide2.dir);
                    return v < 0 ? 1 : 0;
                }

                return 1;
            }
            else if (IsEqual(crossPtToPtVec, polySide1.dir * polySide1.step))
            {
                if (isCmpSideEndPoint)
                {
                    float v = Cross2D(polySide1.startpos - polySide2.startpos, polySide2.dir);
                    return v < 0 ? 1 : 0;
                }

                return 1;
            }

            CmpParallelVecDir(crossPtToPtVec, polySide1.dir, out step);
            //交点crossPt不在polySide1边的范围内
            if (step < 0 || step > polySide1.step + esp)
                return 0;

            return 1;
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
        public bool SolveCrossPoint2D(Vector3 p, Vector3 n, Vector3 o, Vector3 m, out Vector3 pt)
        {
            float value = Dot2D(n, m);
            if (IsZero(value))
            {
                pt = Vector3.zero;
                return false;
            }

            float t = Dot2D(o - p, m) / value;
            pt = p + t * n;
            return true;
        }
    }
}
