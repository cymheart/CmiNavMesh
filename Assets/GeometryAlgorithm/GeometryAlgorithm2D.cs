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

            for (int j = 0; j < polySidesList.Count; j++)
            {
                if (checkSideType == PloySideType.Outside) { if (j == 1) break; }
                else if (checkSideType == PloySideType.Inside) { if (j == 0) continue; }

                polySides = polySidesList[j];

                for (int i = 0; i < polySides.Length; i++)
                {
                    polySide2.startpos = pt;
                    polySide2.dir = n;
                    polySide2.step = 1000000;
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
            bool isHavCrossPt = SolveCrossPoint(polySide2.startpos, polySide2.dir, polySide1.startpos, m, out crossPt);

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
    }
}
