using Mathd;
using System;
using System.Collections.Generic;

namespace Geometry_Algorithm
{
    public partial class GeometryAlgorithm
    {
        double esp = 0.00001f;
        public void SetESP(double esp)
        {
            this.esp = esp;
        }

        /// <summary>
        /// 判断点pt是否在多边形中 
        /// </summary>
        /// <param name="poly">多边形</param>
        /// <param name="polyFaceNormal"></param>
        /// <param name="pt"></param>
        /// <returns></returns>
        public bool IsInsidePolyEx(Poly poly, Vector3d pt, PloySideType checkSideType = PloySideType.Allside)
        {

            PolySide[] polySides = poly.sidesList[0];
            Vector3d n = Vector3d.Cross(pt - polySides[0].startpos, polySides[0].dir);

            if (!IsZero(n) || !IsParallel(n, poly.faceNormal))
                return false;

            return IsInsidePoly(poly, pt, checkSideType);

          
        }

        /// <summary>
        /// 判断点pointMustInPloyPlane是否在多边形中, 
        /// pointMustInPloyPlane必须在多边形的同一平面中,但不一定在多边形中
        /// </summary>
        /// <param name="polySides">多边形中所有点共面</param>
        /// <param name="pointMustInPloyPlane"></param>
        /// <returns></returns>
        public bool IsInsidePoly(Poly poly, Vector3d pointMustInPloyPlane, PloySideType checkSideType = PloySideType.Allside)
        {
            PolySide[] polySides;
            List<PolySide[]> polySidesList = poly.sidesList;
            Vector3d pt = pointMustInPloyPlane;
            Vector3d polyFaceNormal = poly.faceNormal;

            //判断从点引出的平行于sides[0]方向的正向射线是否与其它边相交
            Vector3d n = polySidesList[0][0].startpos - pt;
            int crossPtCount = 0;
            Vector3d crossPt;
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
                    crossPtCount += SolvePolySideCrossPoint(polySides[i], polySide2, polyFaceNormal, true, out crossPt);
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
        public Vector3d[] SolvePolySidesCrossPoints(PolySide[] polySidesA, PolySide[] polySidesB, Vector3d polyFaceNormal)
        {
            List<Vector3d> pts = new List<Vector3d>();
            Vector3d crossPt;
            int crossPtCount;

            for (int i = 0; i < polySidesA.Length; i++)
            {
                for (int j = 0; j < polySidesB.Length; j++)
                {
                    crossPtCount = SolvePolySideCrossPoint(polySidesA[i], polySidesB[j], polyFaceNormal, out crossPt);
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
        public int SolvePolySideCrossPoint(PolySide polySide1, PolySide polySide2, Vector3d polyFaceNormal, out Vector3d crossPt)
        {
            return SolvePolySideCrossPoint(polySide1, polySide2, polyFaceNormal, false, out crossPt);
        }

        /// <summary>
        /// 求解多边形边的交点，
        /// 是否比较边的端点（比较边的端点一般用于点是否在多边形中的判断）
        /// </summary>
        /// <param name="polySide1">多边形的一条边</param>
        /// <param name="polySide2">多边形的一条边</param>
        /// <param name="polyFaceNormal">多边形面向法线</param>
        /// <param name="isCmpSideEndPoint">是否比较边的端点（比较边的端点一般用于点是否在多边形中的判断）</param>
        /// <param name="crossPt">两条边的交点</param>
        /// <returns>返回值为 1: 有1个交点， 0：没有交点</returns>
        public int SolvePolySideCrossPoint(
            PolySide polySide1, PolySide polySide2, Vector3d polyFaceNormal,
            bool isCmpSideEndPoint, out Vector3d crossPt)
        {           
            //m为新的平面的法向，要求m和 polySide1.dir互相垂直,同时m不能与poly的法向平行  
            //既由m确认的平面不能是poly的平面
            Vector3d m = Vector3d.Cross(polySide1.dir, polyFaceNormal);       
            bool isHavCrossPt = SolveCrossPoint(polySide2.startpos, polySide2.dir, polySide1.startpos, m, out crossPt);
            
            //两条边互相平行
            if (!isHavCrossPt)
                return 0;

            double step;
            Vector3d crossPtToPtVec = crossPt - polySide2.startpos;
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
                    Vector3d sideEndPt = polySide1.startpos + polySide1.dir * polySide1.step;
                    Vector3d v = Vector3d.Cross(sideEndPt - polySide2.startpos, polySide2.dir);
                    if (CmpParallelVecDir(v, polyFaceNormal, out step) != 1)
                        return 1;
                    return 0;
                }

                return 1;
            }
            else if(IsEqual(crossPtToPtVec, polySide1.dir * polySide1.step))
            {
                if (isCmpSideEndPoint)
                {
                    Vector3d v = Vector3d.Cross(polySide1.startpos - polySide2.startpos, polySide2.dir);
                    if (CmpParallelVecDir(v, polyFaceNormal, out step) != 1)
                        return 1;
                    return 0;
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
        /// 获取向量vec在方向dir的缩放值（vec必须与dir方向相同）
        /// </summary>
        /// <param name="vec"></param>
        /// <param name="dir"></param>
        /// <returns></returns>
        public double GetScale(Vector3d vec, Vector3d dir)
        {
            double step = 0;
            if (!IsZero(vec.x))
            {
                return vec.x / dir.x;
            }
            else if (!IsZero(vec.y))
            {
                return vec.y / dir.y;
            }
            else if (!IsZero(vec.z))
            {
                return vec.z / dir.z;      
            }
            return step;
        }


        /// <summary>
        /// 比较两平行向量方向
        /// 1:方向相同
        /// -1：方向相反
        /// 0: 没有方向
        /// </summary>
        /// <param name="vec1"></param>
        /// <param name="vec2"></param>
        /// <returns></returns>
        public int CmpParallelVecDir(Vector3d vec1, Vector3d vec2, out double step)
        {
            if (!IsZero(vec1.x))
            {
                step = vec1.x / vec2.x;
                return (step > 0) ? 1 : -1;
            }

            if (!IsZero(vec1.z))
            {
                step = vec1.z / vec2.z;
                return (step > 0) ? 1 : -1;
            }

            if (!IsZero(vec1.y))
            {
                step = vec1.y / vec2.y;
                return (step > 0) ? 1 : -1;
            }

            step = -1;
            return 0;
        }


        /// <summary>
        /// 比较两平行向量方向
        /// 1:方向相同
        /// -1：方向相反
        /// 0: 没有方向
        /// </summary>
        /// <param name="vec1"></param>
        /// <param name="vec2"></param>
        /// <returns></returns>
        public int CmpParallelVecDir(Vector3d vec1, Vector3d vec2)
        {
            double step;

            if (!IsZero(vec1.x))
            {
                step = vec1.x / vec2.x;
                return (step > 0) ? 1 : -1;
            }

            if (!IsZero(vec1.z))
            {
                step = vec1.z / vec2.z;
                return (step > 0) ? 1 : -1;
            }

            if (!IsZero(vec1.y))
            {
                step = vec1.y / vec2.y;
                return (step > 0) ? 1 : -1;
            }

            return 0;
        }


        /// <summary>
        /// 判断两向量是否平行
        /// </summary>
        /// <param name="vec1"></param>
        /// <param name="vec2"></param>
        /// <returns></returns>
        public bool IsParallel(Vector3d vec1, Vector3d vec2)
        {
            Vector3d cmpVec = Vector3d.Cross(vec1, vec2);
            if (IsEqual(cmpVec, Vector3d.zero))
                return true;
            return false;
        }

        /// <summary>
        /// 是否为0值
        /// </summary>
        /// <param name="val"></param>
        /// <returns></returns>
        public bool IsZero(double val)
        {
            if (val > -esp && val < esp)
                return true;
            return false;
        }

    
        /// <summary>
        /// 判断两个向量值是否相等
        /// </summary>
        /// <param name="v1"></param>
        /// <param name="v2"></param>
        /// <returns></returns>
        public bool IsEqual(Vector3d v1, Vector3d v2)
        {
            if (IsZero(v1.x - v2.x) && IsZero(v1.y - v2.y) && IsZero(v1.z - v2.z))
                return true;
            return false;
        }

        /// <summary>
        /// 判断是否为0向量
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public bool IsZero(Vector3d v)
        {
            if (IsZero(v.x) && IsZero(v.y) && IsZero(v.z))
                return true;
            return false;
        }


        /// <summary>
        /// 判断是否在矩形区域中
        /// </summary>
        /// <param name="xmin"></param>
        /// <param name="xmax"></param>
        /// <param name="zmin"></param>
        /// <param name="zmax"></param>
        /// <param name="x"></param>
        /// <param name="z"></param>
        /// <returns></returns>
        public bool IsInRect2D(double xmin, double xmax, double zmin, double zmax, double x, double z)
        {
            if(x >= xmin - esp && x <= xmax + esp &&
                z >= zmin - esp && z <= zmax + esp)
            {
                return true;
            }
            return false;
        }

        /// <summary>
        /// 判断点集在矩形区域中的数量
        /// </summary>
        /// <param name="xmin"></param>
        /// <param name="xmax"></param>
        /// <param name="zmin"></param>
        /// <param name="zmax"></param>
        /// <param name="x"></param>
        /// <param name="z"></param>
        /// <returns></returns>
        public int InRect2DCount(double xmin, double xmax, double zmin, double zmax, Vector3d[] pts, ref int[] inRectIdx)
        {
            int inCount = 0;
   
            for(int i=0; i<pts.Length; i++)
            {
                if (pts[i].x >= xmin - esp && pts[i].x <= xmax + esp &&
                 pts[i].z >= zmin - esp && pts[i].z <= zmax + esp)
                {
                    if(inRectIdx != null)
                        inRectIdx[inCount] = i;
                    inCount++;
                }
            }

            return inCount;    
        }

        /// <summary>
        /// 判断是否在矩形区域中
        /// </summary>
        /// <param name="xmin"></param>
        /// <param name="xmax"></param>
        /// <param name="zmin"></param>
        /// <param name="zmax"></param>
        /// <param name="x"></param>
        /// <param name="z"></param>
        /// <returns></returns>
        public bool IsInRect3D(AABB bound, Vector3d pt)
        {
            if (pt.x >= bound.minX - esp && pt.x <= bound.maxX + esp &&
               pt.z >= bound.minZ - esp && pt.z <= bound.maxZ + esp &&
                pt.y >= bound.minY - esp && pt.y <= bound.maxY + esp)
            {
                return true;
            }
            return false;
        }

        /// <summary>
        /// 判断点集在矩形区域中的数量
        /// </summary>
        /// <param name="xmin"></param>
        /// <param name="xmax"></param>
        /// <param name="zmin"></param>
        /// <param name="zmax"></param>
        /// <param name="x"></param>
        /// <param name="z"></param>
        /// <returns></returns>
        public int InRect3DCount(AABB bound,  Vector3d[] pts, ref int[] inRectIdx)
        {
            int inCount = 0;

            for (int i = 0; i < pts.Length; i++)
            {
                if (pts[i].x >= bound.minX - esp && pts[i].x <= bound.maxX + esp &&
                 pts[i].z >= bound.minZ - esp && pts[i].z <= bound.maxZ + esp &&
                 pts[i].y >= bound.minY - esp && pts[i].y <= bound.maxY + esp)
                {
                    if (inRectIdx != null)
                        inRectIdx[inCount] = i;
                    inCount++;
                }
            }

            return inCount;
        }

        /// <summary>
        /// 获取点集合中y值得最大最小值边界
        /// </summary>
        /// <param name="pts"></param>
        /// <returns></returns>
        public double[] GetYValueBound(Vector3d[] pts)
        {
            if (pts == null || pts.Length == 0)
            {
                int a;
                a = 3;
            }

            double min = pts[0].y, max = pts[0].y;

            for (int i = 1; i < pts.Length; i++)
            {
                if (pts[i].y > max)
                    max = pts[i].y;
                else if (pts[i].y < min)
                    min = pts[i].y;
            }

            return new double[] { min, max };
        }

        /// <summary>
        /// 获取点集合中x值得最大最小值边界
        /// </summary>
        /// <param name="pts"></param>
        /// <returns></returns>
        public double[] GetXValueBound(Vector3d[] pts)
        {
            double min = pts[0].x, max = pts[0].x;

            for (int i = 1; i < pts.Length; i++)
            {
                if (pts[i].x > max)
                    max = pts[i].x;
                else if (pts[i].x < min)
                    min = pts[i].x;
            }

            return new double[] { min, max };
        }

        /// <summary>
        /// 获取点集合中z值得最大最小值边界
        /// </summary>
        /// <param name="pts"></param>
        /// <returns></returns>
        public double[] GetZValueBound(Vector3d[] pts)
        {
            double min = pts[0].z, max = pts[0].z;

            for (int i = 1; i < pts.Length; i++)
            {
                if (pts[i].z > max)
                    max = pts[i].z;
                else if (pts[i].z < min)
                    min = pts[i].z;
            }

            return new double[] { min, max };
        }


        // <summary>
        /// 获取XZ平面上点的边界Rect
        /// </summary>
        /// <param name="verts"></param>
        /// <returns></returns>
        public Vector3d[] GetBoundRectXZ(Vector3d[] verts)
        {
            double xmin = verts[0].x;
            double xmax = verts[0].x;
            double zmin = verts[0].z;
            double zmax = verts[0].z;

            for (int i = 1; i < verts.Length; i++)
            {
                if (verts[i].x < xmin)
                    xmin = verts[i].x;
                else if (verts[i].x > xmax)
                    xmax = verts[i].x;

                if (verts[i].z < zmin)
                    zmin = verts[i].z;
                else if (verts[i].z > zmax)
                    zmax = verts[i].z;
            }

            Vector3d[] boundRect = new Vector3d[]
            {
                    new Vector3d(xmin, 0, zmin),
                    new Vector3d(xmin, 0, zmax),
                    new Vector3d(xmax, 0, zmax),
                    new Vector3d(xmax, 0, zmin),
            };

            return boundRect;
        }


        /// <summary>
        /// 比较向量朝向是否相似
        /// </summary>
        /// <param name="vecA"></param>
        /// <param name="vecB"></param>
        /// <returns></returns>
        public DirCmpInfo CmpVectorDir(Vector3d vecA, Vector3d vecB)
        {
            double val = Vector3d.Dot(vecA, vecB);

            if (IsZero(val))
                return DirCmpInfo.Vertical;    
            else if (val > 0)
                return DirCmpInfo.Same;     
            else
                return DirCmpInfo.Different;    
        }


        /// <summary>
        /// 生成多边形
        /// </summary>
        /// <param name="polyVertexsList"></param>
        /// <param name="faceNormal"></param>
        /// <returns></returns>
        public Poly CreatePoly(Vector3d[] polyVertexs, Vector3d? faceNormal = null)
        {
            return CreatePoly(new List<Vector3d[]> { polyVertexs }, faceNormal);
        }

        /// <summary>
        /// 生成多边形
        /// </summary>
        /// <param name="vertexs"></param>
        /// <returns></returns>
        public Poly CreatePoly(List<Vector3d[]> polyVertexsList, Vector3d? faceNormal)
        {
            Poly poly = new Poly();
            poly.vertexsList = polyVertexsList;
  
            if (faceNormal == null)
            {
                Vector3d[] polyVertexs = polyVertexsList[0];
                Vector3d vec1 = polyVertexs[1] - polyVertexs[0];
                Vector3d vec2 = polyVertexs[2] - polyVertexs[0];
                poly.faceNormal = Vector3d.Cross(vec1, vec2);
                poly.faceNormal.Normalize();
            }
            else
            {
                poly.faceNormal = faceNormal.Value;
            }

            poly.sidesList = CreatePolySidesList(poly.vertexsList, poly.faceNormal);
            return poly;
        }


        /// <summary>
        /// 生成多边形边集合
        /// </summary>
        /// <param name="vertexs"></param>
        /// <returns></returns>
        public List<PolySide[]> CreatePolySidesList(List<Vector3d[]> polyVertexsList, Vector3d faceNormal)
        {
            List<PolySide[]> sidesList = new List<PolySide[]>();
            PolySide side;
            PolySide[] sides;

            for (int j = 0; j < polyVertexsList.Count; j++)
            {
                Vector3d[] polyVertexs = polyVertexsList[j];
                sides = CreatePolySides(polyVertexs);

                for (int i = 0; i < sides.Length; i++)
                {
                    side = sides[i];

                    if (j == 0)
                    {
                        side.vertDir = Vector3d.Cross(side.dir, faceNormal);
                        side.vertDir.Normalize();
                    }
                }

                sidesList.Add(sides);
            }

            return sidesList;
        }



        /// <summary>
        /// 生成基础数据的PolySide组
        /// </summary>
        /// <param name="polyVertexs"></param>
        /// <returns></returns>
        public PolySide[] CreatePolySides(Vector3d[] polyVertexs)
        {
            PolySide[] sides = new PolySide[polyVertexs.Length];

            for (int i = 0; i < polyVertexs.Length; i++)
            {
                if(i == polyVertexs.Length - 1)
                    sides[i] = CreatePolySide(polyVertexs[i], polyVertexs[0]);
                else
                    sides[i] = CreatePolySide(polyVertexs[i], polyVertexs[i+1]);
            }

            return sides;
        }


        /// <summary>
        /// 生成PolySide
        /// </summary>
        /// <param name="startVertex"></param>
        /// <param name="endVertex"></param>
        /// <returns></returns>
        public PolySide CreatePolySide(Vector3d startVertex, Vector3d endVertex)
        {
            PolySide side = new PolySide();
            side.startpos = startVertex;
            side.dir = endVertex - startVertex;
            side.step = side.dir.magnitude;
            side.dir /= side.step;

            return side;
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
        public bool SolveCrossPoint(Vector3d p, Vector3d n, Vector3d o, Vector3d m, out Vector3d pt)
        {
            double value = Vector3d.Dot(n, m);
            if (IsZero(value))
            {
                pt = Vector3d.zero;
                return false;
            }

            double t = Vector3d.Dot(o - p, m) / value;
            pt = p + t * n;
            return true;
        }

        /// <summary>
        /// 判断两个多边形是否互相重叠(只在主多边形上找投影轴, 并且不考虑多边形中空的情况)
        /// </summary>
        /// <param name="mainPoly">主多边形</param>
        /// <param name="otherVertsPloy">需要判断的多边形点</param>
        /// <returns></returns>
        public bool IsOverlap(Poly mainPoly, Vector3d[] otherVertsPloy)
        {
            double[] range;
            PolySide[] sides = mainPoly.sidesList[0];

            for (int i = 0; i < sides.Length; i++)
            {
                range = ProjectPoly(sides[i].vertDir, otherVertsPloy);

                if(!OverlapRange(range[0], range[1], mainPoly.projRange[i].min, mainPoly.projRange[i].max))
                {
                    return false;
                }
            }

            return true;
        }

        bool OverlapRange(double amin, double amax, double bmin, double bmax)
        {
            return ((amin + esp) > bmax || (amax - esp) < bmin) ? false : true;
        }


        /// <summary>
        ///  <para>获取otherVertsPloy相对于mainPoly的覆盖关系:(只在主多边形上找投影轴, 并且不考虑多边形中空的情况)</para>
        /// <para>[ FullOverlap ]: otherVertsPloy完全重叠到mainPloy</para>
        /// <para>[ PartOverlay ]：otherVertsPloy部分重叠mainPloy</para>
        /// <para>[ NotOverlay ]：两个多边形互不重叠</para>
        /// </summary>
        /// <param name="mainPoly"></param>
        /// <param name="otherVertsPloy"></param>
        /// <returns>   
        /// FullOverlap: otherVertsPloy完全重叠到mainPloy
        /// PartOverlay：otherVertsPloy部分重叠mainPloy
        /// NotOverlay：两个多边形互不重叠
        /// </returns>
        public OverlapRelation GetOverlapRelation(Poly mainPoly, Vector3d[] otherVertsPloy)
        {
            double[] range;
            PolySide[] sides = mainPoly.sidesList[0];
            OverlapRelation relation =  OverlapRelation.FullOverlap;
            OverlapRelation tmpRelation;

            for (int i = 0; i < sides.Length; i++)
            {
                range = ProjectPoly(sides[i].vertDir, otherVertsPloy);
                tmpRelation = _OverlapRelation(range[0], range[1], mainPoly.projRange[i].min, mainPoly.projRange[i].max);

                if (tmpRelation ==  OverlapRelation.PartOverlay)
                {
                    relation = OverlapRelation.PartOverlay;
                }
                else if (tmpRelation ==  OverlapRelation.FullOverlap && 
                    relation != OverlapRelation.PartOverlay)
                {
                    relation = OverlapRelation.FullOverlap;
                }
                else if (tmpRelation == OverlapRelation.NotOverlay)
                {
                    return OverlapRelation.NotOverlay;
                }
            }

            return relation;
        }


        /// <summary>
        /// <para>[ FullOverlap ]: vertsPoly完全重叠到mainPloy</para>
        /// <para>[ PartOverlay ]：vertsPoly部分重叠mainPloy</para>
        /// <para>[ NotOverlay ]：两个多边形互不重叠</para>
        /// </summary>
        /// <param name="vertsPolyMin"></param>
        /// <param name="vertsPolyMax"></param>
        /// <param name="mainPloyMin"></param>
        /// <param name="mainPolyMax"></param>
        /// <returns></returns>
        OverlapRelation _OverlapRelation(double vertsPolyMin, double vertsPolyMax, double mainPloyMin, double mainPolyMax)
        {
            if (vertsPolyMin >= mainPloyMin && vertsPolyMax <= mainPolyMax)
                return OverlapRelation.FullOverlap;
            else if((vertsPolyMin + esp) > mainPolyMax || (vertsPolyMax - esp) < mainPloyMin)
                return OverlapRelation.NotOverlay;
            return OverlapRelation.PartOverlay;
        }



        /// <summary>
        /// 计算多边形自身投影轴的范围
        /// </summary>
        /// <param name="poly"></param>
        public void CreatePolySelfProjectAxisRange(Poly poly)
        {
            double[] range;
            PolySide[] sides = poly.sidesList[0];
            Range[] ranges = new Range[sides.Length];

            for (int i = 0; i < sides.Length; i++)
            {
                range = ProjectPoly(sides[i].vertDir, poly.vertexsList[0]);
                ranges[i].min = range[0];
                ranges[i].max = range[1];
            }

            poly.projRange = ranges;
        }


        /// <summary>
        /// 求最大和最小的点积值 相当于 多边形在 轴上的投影范围
        /// 向量投影公式:
        /// |puv| = v.u * axis/|axis|
        /// </summary>
        /// <param name="axis"></param>
        /// <param name="polyVertexs"></param>
        /// <returns></returns>
        public double[] ProjectPoly(Vector3d axis, Vector3d[] polyVertexs)
        {
            double min, max,tmp;
            min = max = Vector3d.Dot(axis, polyVertexs[0]);

            for (int i = 1; i < polyVertexs.Length; i++)
            {
                tmp = Vector3d.Dot(axis, polyVertexs[i]);
                min = Math.Min(min, tmp);
                max = Math.Max(max, tmp);
            }

            return new double[] { min, max };
        }
    }
}
