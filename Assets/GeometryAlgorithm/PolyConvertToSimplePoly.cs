using Mathd;
using System;
using System.Collections.Generic;

namespace Geometry_Algorithm
{
    public class PolyConvertToSimplePoly
    {
        GeometryAlgorithm geoAlgor;

        public struct IgnoreSideInfo
        {
            public int ringIdx;
            public int sideIdx1;
            public int sideIdx2;
        }

        public PolyConvertToSimplePoly(GeometryAlgorithm geoAlgor)
        {
            this.geoAlgor = geoAlgor;
        }

        /// <summary>     
        /// 切割为简单多边形
        /// </summary>
        /// <param name="poly"></param>
        /// <returns></returns>
        public Poly ConvertToSimplePoly2D(Poly poly)
        {
            List<IgnoreSideInfo> ignoreSideInfos = new List<IgnoreSideInfo>();
            Vector3d startVertA;
            Vector3d endVertA;
            PolySide sideAB;
            Vector3d[] outsideVertexs;
            Vector3d[] insideVertexs;

            while (true)
            {
                //判断是否存在内环多边形,当不存在内环时,结束算法，返回最后生成的多边形
                if (poly.vertexsList.Count <= 1)
                    return poly;

                outsideVertexs = poly.vertexsList[0];

                for (int i = 1; i < poly.vertexsList.Count; i++)
                {
                    insideVertexs = poly.vertexsList[i];
                    for(int j = 1; j < insideVertexs.Length; j++)
                    {
                        ignoreSideInfos.Clear();
                        startVertA = insideVertexs[j];

                        AddIgnoreSideInfos(poly, i, j, ignoreSideInfos);

                        for (int k = 0; k < outsideVertexs.Length; k++)
                        { 
                            endVertA = outsideVertexs[k];

                            if(ignoreSideInfos.Count == 2)
                                ignoreSideInfos.RemoveAt(1);

                            AddIgnoreSideInfos(poly, 0, k, ignoreSideInfos);

                            if (HavOverlayPoint(poly, endVertA))
                                continue;

                            sideAB = geoAlgor.CreatePolySide(startVertA, endVertA);

                            if (HavCrossPoint(poly, sideAB, ignoreSideInfos.ToArray()))
                                continue;

                            poly = CreatePolyByRemoveRing(poly, i, j, k, sideAB);
                            i = poly.vertexsList.Count;
                            j = insideVertexs.Length;
                            break;
                        }
                    }

                    if (i == 0)
                        break;
                    else if (i == poly.vertexsList.Count - 1)
                        i = -1;
                }
            }
        }

        bool HavCrossPoint(Poly poly, PolySide sideAB, IgnoreSideInfo[] ignoreSideInfos)
        {
            int result;
            Vector3d pt;

            for (int i = 0; i < poly.sidesList.Count; i++)
            {
                PolySide[] sides = poly.sidesList[i];

                for (int j = 0; j < sides.Length; j++)
                {
                    if (InIgnoreSides(i, j, ignoreSideInfos))
                        continue;

                    //求解ab线段和多边形边sides[j]的交点
                    result = geoAlgor.SolvePolySideCrossPoint2D(sides[j], sideAB, out pt);
                    if (result == 1)  //存在交点
                        return true;            
                }
            }

            return false;
        }

        bool HavOverlayPoint(Poly poly, Vector3d vert)
        {
            Vector3d[] vertexs;
            int count = 0;

            for (int i = 0; i < poly.vertexsList.Count; i++)
            {
                vertexs = poly.vertexsList[i];
                for (int j = 0; j < vertexs.Length; j++)
                {
                    if (geoAlgor.IsEqual(vertexs[j], vert) == false)
                        continue;
                    count++;
                    if (count == 2)
                        return true;
                }
            }

            return false;
        }



        /// <summary>
        /// 生成去除一个内环后的多边形
        /// </summary>
        /// <param name="poly">原始多边形</param>
        /// <param name="ringIdx">内环在原始多边形中的编号</param>
        /// <param name="ringVertexIdx">内环分切点编号</param>
        /// <param name="outVertexIdx">外环分切点编号</param>
        /// <param name="endLinkSide">连接线段</param>
        /// <returns></returns>
        Poly CreatePolyByRemoveRing(
            Poly poly, 
            int ringIdx, 
            int ringSplitVertIdx, 
            int outSplitVertIdx,
            PolySide endLinkSide)
        {
            List<Vector3d> resultPolyVertexList = new List<Vector3d>();
            List<PolySide> resultPolySideList = new List<PolySide>();
            Vector3d outVert = poly.vertexsList[0][outSplitVertIdx];
            Vector3d startVert = poly.vertexsList[ringIdx][ringSplitVertIdx];

            for (int i = outSplitVertIdx; i < poly.vertexsList[0].Length; i++)
                resultPolyVertexList.Add(poly.vertexsList[0][i]);
            for (int i = 0; i <= outSplitVertIdx; i++)
                resultPolyVertexList.Add(poly.vertexsList[0][i]);

            for (int i = ringSplitVertIdx; i < poly.vertexsList[ringIdx].Length; i++)
                resultPolyVertexList.Add(poly.vertexsList[ringIdx][i]);
            for (int i = 0; i <= ringSplitVertIdx; i++)
                resultPolyVertexList.Add(poly.vertexsList[ringIdx][i]);

            //   
            for (int i = outSplitVertIdx; i < poly.sidesList[0].Length; i++)
                resultPolySideList.Add(poly.sidesList[0][i]);
            for (int i = 0; i < outSplitVertIdx; i++)
                resultPolySideList.Add(poly.sidesList[0][i]);


            PolySide linkSide = geoAlgor.CreatePolySide(outVert, startVert);
            resultPolySideList.Add(linkSide);

            for (int i = ringSplitVertIdx; i < poly.sidesList[ringIdx].Length; i++)
                resultPolySideList.Add(poly.sidesList[ringIdx][i]);
            for (int i = 0; i < ringSplitVertIdx; i++)
                resultPolySideList.Add(poly.sidesList[ringIdx][i]);

            resultPolySideList.Add(endLinkSide);


            //
            Poly resultPoly = new Poly();
            resultPoly.sidesList.Add(resultPolySideList.ToArray());

            for (int i = 1; i < poly.sidesList.Count; i++)
            {
                if (i == ringIdx)
                    continue;

                resultPoly.sidesList.Add(poly.sidesList[i]);
            }


            //
            resultPoly.vertexsList.Add(resultPolyVertexList.ToArray());

            for (int i = 1; i < poly.vertexsList.Count; i++) 
            {
                if (i == ringIdx)
                    continue;

                resultPoly.vertexsList.Add(poly.vertexsList[i]);
            }

            resultPoly.faceNormal = poly.faceNormal;
            return resultPoly;
        }

        void AddIgnoreSideInfos(Poly poly, int ring, int vertIdx, List<IgnoreSideInfo> ignoreSideInfos)
        {
            IgnoreSideInfo ignoreSideInfo;
            int otherSideIdx;

            if (vertIdx == 0) { otherSideIdx = poly.sidesList[ring].Length - 1; }
            else { otherSideIdx = vertIdx - 1; }

            ignoreSideInfo = new IgnoreSideInfo()
            {
                ringIdx = ring,
                sideIdx1 = vertIdx,
                sideIdx2 = otherSideIdx
            };

            ignoreSideInfos.Add(ignoreSideInfo);
        }


        /// <summary>
        /// 判断指定环边是否在忽略边列表中
        /// </summary>
        /// <param name="ringIdx"></param>
        /// <param name="sideIdx"></param>
        /// <param name="ignoreSideInfos"></param>
        /// <returns></returns>
        bool InIgnoreSides(int ringIdx, int sideIdx, IgnoreSideInfo[] ignoreSideInfos)
        {
            for(int i=0; i<ignoreSideInfos.Length; i++)
            {
                if (ringIdx == ignoreSideInfos[i].ringIdx &&
                    (sideIdx == ignoreSideInfos[i].sideIdx1 ||
                    sideIdx == ignoreSideInfos[i].sideIdx2))
                    return true;
            }

            return false;

        }
    }

}
