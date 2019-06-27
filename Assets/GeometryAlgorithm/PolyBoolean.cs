using System.Collections.Generic;
using Mathd;

namespace Geometry_Algorithm
{
    public class PolyBoolean
    {
        Poly polyA;
        Poly polyB;
        GeometryAlgorithm geoAlgor;
        CrossPointInfoComparer cmp;
        Vector3d offsetValue;

        List<Vector3d[]> resultPolyVertsList = new List<Vector3d[]>();

        public PolyBoolean(GeometryAlgorithm geoAlgor)
        {
            this.geoAlgor = geoAlgor;
            cmp = new CrossPointInfoComparer(this);
        }

        public void Sub(Vector3d[] polyVertsA, Vector3d[] polyVertsB)
        {
            CreatePolys(polyVertsA, polyVertsB);
            CreatePolysCrossPointInfoGroup();
            SubOp();
        }

        public void Intersection(Vector3d[] polyVertsA, Vector3d[] polyVertsB)
        {
            CreatePolys(polyVertsA, polyVertsB);
            CreatePolysCrossPointInfoGroup();
            IntersectionOp();
        }

        public void Union(Vector3d[] polyVertsA, Vector3d[] polyVertsB)
        {
            CreatePolys(polyVertsA, polyVertsB);
            CreatePolysCrossPointInfoGroup();
            UnionOp();
        }


        public void Sub(Poly polyA, Poly polyB)
        {
            this.polyA = polyA.Copy();
            this.polyB = polyB.Copy();
            AdjustPolyPos(this.polyB);
            CreatePolysCrossPointInfoGroup();
            SubOp();
        }

        public void Intersection(Poly polyA, Poly polyB)
        {
            this.polyA = polyA.Copy();
            this.polyB = polyB.Copy();
            AdjustPolyPos(this.polyB);
            CreatePolysCrossPointInfoGroup();
            IntersectionOp();
        }

        public void Union(Poly polyA, Poly polyB)
        {
            this.polyA = polyA.Copy();
            this.polyB = polyB.Copy();
            AdjustPolyPos(this.polyB);
            CreatePolysCrossPointInfoGroup();
            UnionOp();
        }


        void CreatePolys(Vector3d[] polyVertsA, Vector3d[] polyVertsB)
        {
            polyA = geoAlgor.CreatePoly(polyVertsA, Vector3d.zero);
            polyB = geoAlgor.CreatePoly(polyVertsB, Vector3d.zero);
            AdjustPolyPos(polyB);
        }

        void AdjustPolyPos(Poly poly)
        {
            Vector3d dir1 = poly.sidesList[0][0].dir;
            Vector3d dir2 = poly.sidesList[0][1].dir;
            Vector3d dir3 = poly.sidesList[0][2].dir;
            float scale1 = UnityEngine.Random.Range(0.0006f, 0.001f);
            float scale2 = UnityEngine.Random.Range(0.0006f, 0.001f);
            float scale3 = UnityEngine.Random.Range(0.0006f, 0.001f);
            offsetValue = dir1 * scale1 + dir2* scale2 + dir3 * scale3;

            List<PolySide[]> sidesList = poly.sidesList;

            for (int i = 0; i < sidesList.Count; i++)
            {
                PolySide[] sides = sidesList[i];
                for (int j = 0; j < sides.Length; j++)
                {
                    sides[j].startpos += offsetValue;
                }
            }
        }

        void RestorePolyPos(Poly poly)
        {
            List<PolySide[]> sidesList = poly.sidesList;
            for (int i = 0; i < sidesList.Count; i++)
            {
                PolySide[] sides = sidesList[i];
                for (int j = 0; j < sides.Length; j++)
                {
                    sides[j].startpos -= offsetValue;
                }
            }
        }

        public List<Vector3d[]> GetResultPolyVerts()
        {
            return resultPolyVertsList;
        }
   
        void CreatePolysCrossPointInfoGroup()
        {
            CreatePolyEndPointInfo(polyA);
            CreatePolyEndPointInfo(polyB);
            CreatePolysCrossPointInfo();
            SortPolyCrossPointInfo(polyA);
            SortPolyCrossPointInfo(polyB);
            AdjustPolyEndPointInfo(polyA);
            AdjustPolyEndPointInfo(polyB);
            CreatePolysCrossPointsInOutType(polyA, polyB);
            CreatePolysCrossPointsInOutType(polyB, polyA);
        }

        /// <summary>
        /// 生成多边形之间的边相交交点信息
        /// </summary>
        void CreatePolysCrossPointInfo()
        {
            List<PolySide[]> sidesListA = polyA.sidesList;
            List<PolySide[]> sidesListB = polyB.sidesList;
            Vector3d crossPt;
            int crossPtCount;

            for (int i=0; i < sidesListA.Count; i++)
            {
                PolySide[] sidesA = sidesListA[i];
                for (int j = 0; j < sidesA.Length; j++)
                {
                    for (int i2 = 0; i2 < sidesListB.Count; i2++)
                    {
                        PolySide[] sidesB = sidesListB[i2];
                        for (int j2 = 0; j2 < sidesB.Length; j2++)
                        {
                            crossPtCount = geoAlgor.SolvePolySideCrossPoint(sidesA[j], sidesB[j2], polyA.faceNormal, out crossPt);
                            if (crossPtCount == 0)
                                continue;

                            CrossPointInfo crossPtInfoA;
                            CrossPointInfo crossPtInfoB;

                            if (sidesA[j].crossPointInfoList == null)
                                sidesA[j].crossPointInfoList = new List<CrossPointInfo>();
                            
                            if (sidesB[j2].crossPointInfoList == null)
                                sidesB[j2].crossPointInfoList = new List<CrossPointInfo>();

                            crossPtInfoA = new CrossPointInfo();
                            crossPtInfoA.selfSide = sidesA[j];
                            crossPtInfoA.crossSide = sidesB[j2];
                            crossPtInfoA.pt = crossPt;
                            crossPtInfoA.pointIdx = sidesA[j].crossPointInfoList.Count;
                            crossPtInfoA.crossSidePointIdx = sidesB[j2].crossPointInfoList.Count;
                            Vector3d v = crossPt - crossPtInfoA.selfSide.startpos;
                            crossPtInfoA.dist = geoAlgor.GetScale(v, crossPtInfoA.selfSide.dir);
                            sidesA[j].crossPointInfoList.Add(crossPtInfoA);

                            //
                            crossPtInfoB = new CrossPointInfo();
                            crossPtInfoB.selfSide = sidesB[j2];
                            crossPtInfoB.crossSide = sidesA[j];
                            crossPtInfoB.pt = crossPt;
                            crossPtInfoB.pointIdx = sidesB[j2].crossPointInfoList.Count;
                            crossPtInfoB.crossSidePointIdx = sidesA[j].crossPointInfoList.Count;
                            v = crossPt - crossPtInfoB.selfSide.startpos;
                            crossPtInfoB.dist = geoAlgor.GetScale(v, crossPtInfoB.selfSide.dir);
                            sidesB[j2].crossPointInfoList.Add(crossPtInfoB);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// 为多边形生成端点的交点信息
        /// </summary>
        /// <param name="poly"></param>
        void CreatePolyEndPointInfo(Poly poly)
        {
            CrossPointInfo crossPtInfo;
            List<PolySide[]> sidesList = poly.sidesList;
            for (int i = 0; i < sidesList.Count; i++)
            {
                PolySide[] sides = sidesList[i];
                for (int j = 0; j < sides.Length; j++)
                {
                    if (sides[j].crossPointInfoList == null)
                        sides[j].crossPointInfoList = new List<CrossPointInfo>();

                    sides[j].crossPointInfoList = new List<CrossPointInfo>();
                   
                    crossPtInfo = new CrossPointInfo();
                    crossPtInfo.type = PointType.EndPoint;
                    crossPtInfo.selfSide = sides[j];         
                    crossPtInfo.crossSide = null;
                    crossPtInfo.pt = sides[j].startpos;
                    crossPtInfo.pointIdx = 0;
                    crossPtInfo.crossSidePointIdx = 1;
                    crossPtInfo.dist = 0 - 0.0001f;
                    sides[j].crossPointInfoList.Add(crossPtInfo);

                    crossPtInfo = new CrossPointInfo();
                    crossPtInfo.type = PointType.EndPoint;
                    crossPtInfo.selfSide = sides[j];
                    crossPtInfo.crossSide = null;
                    crossPtInfo.pt = sides[j].startpos + sides[j].dir * sides[j].step;
                    crossPtInfo.pointIdx = 1;
                    crossPtInfo.crossSidePointIdx = 0;
                    crossPtInfo.dist = sides[j].step + 0.0001f;
                    sides[j].crossPointInfoList.Add(crossPtInfo);
                }
            }
        }

        void AdjustPolyEndPointInfo(Poly poly)
        {
            CrossPointInfo crossPtInfo;
            List<PolySide[]> sidesList = poly.sidesList;
            for (int i = 0; i < sidesList.Count; i++)
            {
                PolySide[] sides = sidesList[i];
                for (int j = 0; j < sides.Length; j++)
                {
                    crossPtInfo = sides[j].crossPointInfoList[0];
                    if (j == 0) { crossPtInfo.crossSide = sides[sides.Length - 1]; }
                    else { crossPtInfo.crossSide = sides[j - 1]; }
                    crossPtInfo.crossSidePointIdx = crossPtInfo.crossSide.crossPointInfoList.Count - 1;

                    crossPtInfo = sides[j].crossPointInfoList[sides[j].crossPointInfoList.Count - 1];
                    if (j == sides.Length - 1) { crossPtInfo.crossSide = sides[0]; }
                    else { crossPtInfo.crossSide = sides[j + 1]; }
                    crossPtInfo.crossSidePointIdx = 0;
                }
            }
        }

        /// <summary>
        /// 排序多边形边上的交点顺序，按距离起始点
        /// </summary>
        /// <param name="poly"></param>
        void SortPolyCrossPointInfo(Poly poly)
        {
            List<PolySide[]> sidesList = poly.sidesList;
            PolySide[] sides;
            List<CrossPointInfo> crossPtInfoList;
            CrossPointInfo crossPtInfo;

            for (int i=0; i < sidesList.Count; i++)
            {
                sides = sidesList[i];
                for (int j = 0; j < sides.Length; j++)
                {
                    crossPtInfoList = sides[j].crossPointInfoList;
                    if (crossPtInfoList == null)
                        continue;

                    crossPtInfoList.Sort(cmp);    
       
                    for(int k = 0; k < crossPtInfoList.Count; k++)
                    {
                        crossPtInfoList[k].pointIdx = k;
                        crossPtInfo = crossPtInfoList[k].GetCrossSideCrossPointInfo();  

                        if(crossPtInfo != null)
                            crossPtInfo.crossSidePointIdx = k;
                    }
                }
            }
        }


        /// <summary>
        /// 生成多边形交点的进出点类型
        /// </summary>
        void CreatePolysCrossPointsInOutType(Poly polyA, Poly polyB)
        {
            List<PolySide[]> sidesListA = polyA.sidesList;
            int inoutType = 0;
            List<CrossPointInfo> cpinfoList;
            CrossPointInfo cpinfo;

            for (int i = 0; i < sidesListA.Count; i++)
            {
                PolySide[] sidesA = sidesListA[i];

                if (geoAlgor.IsInsidePoly(polyB, sidesA[0].startpos))
                    inoutType = 1;
                else
                    inoutType = 0;

                cpinfoList = sidesA[0].crossPointInfoList;
                cpinfoList[0].inoutType = inoutType;

                for (int j = 0; j < sidesA.Length; j++)
                {
                    cpinfoList = sidesA[j].crossPointInfoList;
                    for (int k = 1; k < cpinfoList.Count; k++)
                    {
                        if (k == cpinfoList.Count - 1)
                        {
                            cpinfoList[k].inoutType = cpinfoList[k - 1].inoutType;
                            cpinfo = cpinfoList[k].GetCrossSideCrossPointInfo();
                            cpinfo.inoutType = cpinfoList[k].inoutType;
                        }
                        else
                        {
                            cpinfoList[k].inoutType = 1 - cpinfoList[k - 1].inoutType;
                        } 
                    }
                }
            }
        }

        class CrossPointInfoComparer : IComparer<CrossPointInfo>
        {
            GeometryAlgorithm geoAlgor;
            public CrossPointInfoComparer(PolyBoolean polyBoolean)
            {
                geoAlgor = polyBoolean.geoAlgor;
            }
            public int Compare(CrossPointInfo left, CrossPointInfo right)
            {
                if (geoAlgor.IsZero(left.dist - right.dist))
                    return 0;
                else if (left.dist > right.dist)
                    return 1;
                else
                    return -1;
            }
        }


        void BooleanOp(BooleanType booleanOp)
        {
            List<Vector3d> newPolyVertList = new List<Vector3d>();
            CrossPointInfo firstPtInfo;
            CrossPointInfo ptInfo, ptInfo2;
            bool isContinue;
            int checkInoutType = 0;


            if (booleanOp == BooleanType.Intersection)
                checkInoutType = 1;
            else if (booleanOp == BooleanType.Union)
                checkInoutType = 0;
            else
                return;

            while (true)
            {
                isContinue = true;
                ptInfo = firstPtInfo = polyA.GetOneUnUsedCrossPointInfo();
                if (firstPtInfo == null)
                    break;

                if (ptInfo.inoutType == 1 - checkInoutType)
                    ptInfo = ptInfo.GetCrossSideCrossPointInfo();

                while (isContinue)
                {
                    while (ptInfo.inoutType == checkInoutType)
                    {
                        ptInfo.SetUsed();
                        newPolyVertList.Add(ptInfo.pt);

                        ptInfo2 = ptInfo.GetSelfSideNextCrossPointInfo();
                        if (ptInfo2 != null)
                        {
                            ptInfo = ptInfo2;
                        }
                        else
                        {
                            ptInfo = ptInfo.GetCrossSideCrossPointInfo();
                            ptInfo = ptInfo.GetSelfSideNextCrossPointInfo();
                        }

                        if (ptInfo == firstPtInfo)
                        {
                            isContinue = false;
                            break;
                        }
                    }

                    if (isContinue)
                    {
                        ptInfo = ptInfo.GetCrossSideCrossPointInfo();
                        if (ptInfo == firstPtInfo)
                            isContinue = false;
                    }
                }

                if (newPolyVertList.Count > 0)
                {
                    resultPolyVertsList.Add(newPolyVertList.ToArray());
                    newPolyVertList.Clear();
                }
            }
        }


        /// <summary>
        /// 差集
        /// </summary>
        void SubOp()
        {
            List<Vector3d> newPolyVertList = new List<Vector3d>();
            CrossPointInfo firstPtInfo;
            CrossPointInfo ptInfo = null, ptInfo2;
            bool wiseFlag;
            bool isPolyANull = false;

            while (true)
            {
                wiseFlag = true;
                firstPtInfo = null;

                if (!isPolyANull)
                    ptInfo = firstPtInfo = polyA.GetOneUnUsedEndPointInfo(0);

                if (firstPtInfo == null)
                {
                    isPolyANull = true;
                    ptInfo = firstPtInfo = polyB.GetOneUnUsedEndPointInfo(0);
                    if (firstPtInfo == null)
                        break;
                }

                while (true)
                {       
                    ptInfo.SetUsed();
                    newPolyVertList.Add(ptInfo.pt);

                    if (wiseFlag)
                    {
                        ptInfo2 = ptInfo.GetSelfSideNextCrossPointInfo();
                        if (ptInfo2 != null){
                            ptInfo = ptInfo2;
                        }else{
                            ptInfo = ptInfo.GetCrossSideCrossPointInfo();
                            ptInfo = ptInfo.GetSelfSideNextCrossPointInfo();
                        }
                    }
                    else
                    {
                        ptInfo2 = ptInfo.GetSelfSidePrevCrossPointInfo();

                        if (ptInfo2 != null){
                            ptInfo = ptInfo2;
                        }else{
                            ptInfo = ptInfo.GetCrossSideCrossPointInfo();
                            ptInfo = ptInfo.GetSelfSidePrevCrossPointInfo();
                        }
                    }

                    if (ptInfo.type != PointType.EndPoint)
                    {
                        ptInfo = ptInfo.GetCrossSideCrossPointInfo();
                        wiseFlag = !wiseFlag;
                    }
                    else
                    {
                        if (ptInfo == firstPtInfo ||
                            ptInfo.GetCrossSideCrossPointInfo() == firstPtInfo)
                            break;
                    }
                }

                if (newPolyVertList.Count > 0)
                {
                    resultPolyVertsList.Add(newPolyVertList.ToArray());
                    newPolyVertList.Clear();
                }
            }

            if (resultPolyVertsList.Count != 0)
                return;

            //
            int inoutA = polyA.sidesList[0][0].crossPointInfoList[0].inoutType;
            int inoutB = polyB.sidesList[0][0].crossPointInfoList[0].inoutType;

            if (inoutA == 0 && inoutB == 0)
            {
                for (int i = 0; i < polyA.vertexsList.Count; i++)
                    resultPolyVertsList.Add(polyA.vertexsList[i]);

                for (int i = 0; i < polyB.vertexsList.Count; i++)
                    resultPolyVertsList.Add(polyB.vertexsList[i]);

                return;
            }

            if (inoutA == 0 && inoutB == 1)
            {
                CreateInoutTypeVertesxToResult(polyA, polyB);
                return;
            }

            if (inoutA == 1 && inoutB == 0)
            {
                CreateInoutTypeVertesxToResult(polyB, polyA);
                return;
            }

        }

        void CreateInoutTypeVertesxToResult(Poly polya, Poly polyb)
        {
            for (int i = 0; i < polya.vertexsList.Count; i++)
                resultPolyVertsList.Add(polya.vertexsList[i]);

            for (int i = 0; i < polyb.vertexsList.Count; i++)
            {
                int len = polyb.vertexsList[i].Length;
                Vector3d[] verts = new Vector3d[len];
                for (int j = 0; j < len; j++)
                    verts[j] = polyb.vertexsList[i][len - i - 1];
                resultPolyVertsList.Add(verts);
            }
        }

   
        /// <summary>
        /// 交集
        /// </summary>
        void IntersectionOp()
        {
            BooleanOp(BooleanType.Intersection);

            if (resultPolyVertsList.Count != 0)
                return;

            int inoutA = polyA.sidesList[0][0].crossPointInfoList[0].inoutType;
            int inoutB = polyB.sidesList[0][0].crossPointInfoList[0].inoutType;

            if (inoutA == 0 && inoutB == 0)
                return;

            if (inoutA == 0 && inoutB == 1)
            {
                for (int i = 0; i < polyB.vertexsList.Count; i++)
                    resultPolyVertsList.Add(polyB.vertexsList[i]);
                return;
            }

            if (inoutA == 1 && inoutB == 0)
            {
                for (int i = 0; i < polyA.vertexsList.Count; i++)
                    resultPolyVertsList.Add(polyA.vertexsList[i]);
                return;
            }
        }

        /// <summary>
        /// 并集
        /// </summary>
        void UnionOp()
        { 
             BooleanOp(BooleanType.Union);

            if (resultPolyVertsList.Count != 0)
                return;

            int inoutA = polyA.sidesList[0][0].crossPointInfoList[0].inoutType;
            int inoutB = polyB.sidesList[0][0].crossPointInfoList[0].inoutType;

            if (inoutA == 0 && inoutB == 0)
            {
                for (int i = 0; i < polyA.vertexsList.Count; i++)
                    resultPolyVertsList.Add(polyA.vertexsList[i]);

                for (int i = 0; i < polyB.vertexsList.Count; i++)
                    resultPolyVertsList.Add(polyB.vertexsList[i]);

                return;
            }

            if (inoutA == 0 && inoutB == 1)
            {
                for (int i = 0; i < polyA.vertexsList.Count; i++)
                    resultPolyVertsList.Add(polyA.vertexsList[i]);
                return;
            }

            if (inoutA == 1 && inoutB == 0)
            {
                for (int i = 0; i < polyB.vertexsList.Count; i++)
                    resultPolyVertsList.Add(polyB.vertexsList[i]);
                return;
            }
        }
    } 
}
