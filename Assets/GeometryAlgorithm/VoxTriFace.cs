using LinearAlgebra.VectorAlgebra;
using Mathd;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace Geometry_Algorithm
{
    public class VoxTriFace
    {
        VoxSpace voxSpace;
        Vector3d[] vertexs;
        Vector3d[] vertexsProjectionFloor;
        Poly poly;
        Poly polyProjectionFloor;
        Vector3d triFaceNormal;
        Vector3d floorGridNormal = Vector3d.up;
        DirCmpInfo faceDirType = DirCmpInfo.Same;

        List<VoxBox> voxBoxList = new List<VoxBox>();
        VoxViewer voxViewer = new VoxViewer();
        GeometryAlgorithm geoAlgo = new GeometryAlgorithm();

        AABB aabb = new AABB() {minX = 999999f, maxX = 0f, minZ = 999999f, maxZ = 0f };
        int xstartCell, xendCell;
        int zstartCell, zendCell;


        int[] inRectIdx;
        int[] inTriFaceIdx;
        List<Vector3d> edgePloyPts = new List<Vector3d>();

        Vector3d[] preCellRect;
        PolySide[] preCellRectSides;
        public void SetVoxSpace(VoxSpace voxSpace)
        {
            this.voxSpace = voxSpace;
        }

        public void TransTriFaceWorldVertexToVoxSpace(Vector[] triFaceWorldVertex)
        {
            //
            _TransTriFaceWorldVertexToVoxSpace(triFaceWorldVertex);
            CreatePoly();

            //
            CreateProjectionToFloorTriFaceVertexs();
            CreateProjectionToFloorPoly();
                
            CalFloorGridIdxRange();


            //
            inRectIdx = new int[vertexs.Length];
            inTriFaceIdx = new int[4];

            //
            preCellRect = new Vector3d[]
            {
                Vector3d.zero,
                new Vector3d(0, 0, voxSpace.cellSize),
                new Vector3d(voxSpace.cellSize, 0, voxSpace.cellSize),
                new Vector3d(voxSpace.cellSize, 0, 0),
            };

            preCellRectSides = geoAlgo.CreateBaseDataPolySides(preCellRect);


            //
            CreateFloorGridProjTriFaceVoxBox();   
        }

        /// <summary>
        /// 转换三角面的世界顶点到体素空间
        /// </summary>
        /// <param name="triFaceWorldVertex"></param>
        void _TransTriFaceWorldVertexToVoxSpace(Vector[] triFaceWorldVertex)
        {
            vertexs = new Vector3d[triFaceWorldVertex.Length];

            Vector vec;
            for(int i=0;i < triFaceWorldVertex.Length; i++)
            {
                vec = Vector.MatMulColVec(voxSpace.worldToVoxSpace, triFaceWorldVertex[i]);
                vertexs[i] = new Vector3d(vec.Elements[0], vec.Elements[1], vec.Elements[2]);

                if (vertexs[i].x > aabb.maxX) { aabb.maxX = vertexs[i].x; }
                if (vertexs[i].x < aabb.minX) { aabb.minX = vertexs[i].x; }
                if (vertexs[i].z > aabb.maxZ) { aabb.maxZ = vertexs[i].z; }
                if (vertexs[i].z < aabb.minZ) { aabb.minZ = vertexs[i].z; }
                if (vertexs[i].y > aabb.maxY) { aabb.maxY = vertexs[i].y; }
                if (vertexs[i].y < aabb.minY) { aabb.minY = vertexs[i].y; }        
            }

            CalTriFaceNormal();

            if(faceDirType == DirCmpInfo.Vertical)
            {
                float pos;
                for (int i=0; i < vertexs.Length; i++)
                {
                    pos = UnityEngine.Random.Range(0.0001f, 0.0006f);
                    vertexs[i].x += pos;
                    vertexs[i].y += pos;
                    vertexs[i].z += pos;
                }

                CalTriFaceNormal();
            }

            triFaceNormal.Normalize();
        }

        /// <summary>
        /// 计算三角面法线
        /// </summary>
        void CalTriFaceNormal()
        {
            Vector3d vec1 = vertexs[1] - vertexs[0];
            Vector3d vec2 = vertexs[2] - vertexs[0];
            triFaceNormal = Vector3d.Cross(vec1, vec2);
            faceDirType = geoAlgo.CmpVectorDir(triFaceNormal, Vector3d.up);
        }


        /// <summary>
        /// 计算三角面在floorGrid的格子范围
        /// </summary>
        void CalFloorGridIdxRange()
        {
            //xstartCell
            double n = aabb.minX / voxSpace.cellSize;
            xstartCell = (int)Math.Floor(n);

            //xendCell
            n = aabb.maxX / voxSpace.cellSize;
            xendCell = (int)Math.Ceiling(n);
            if (xstartCell == xendCell) xendCell++;

            //zstartCell
            n = aabb.minZ / voxSpace.cellSize;
            zstartCell = (int)Math.Floor(n);

            //zendCell
            n = aabb.maxZ / voxSpace.cellSize;
            zendCell = (int)Math.Ceiling(n);
            if (zstartCell == zendCell) zendCell++;

        }

      
        /// <summary>
        /// 生成三角面到地面的投影顶点
        /// </summary>
        void CreateProjectionToFloorTriFaceVertexs()
        {
            vertexsProjectionFloor = new Vector3d[vertexs.Length];

            if (faceDirType ==  DirCmpInfo.Different)
            {
                for (int i = 0; i < vertexs.Length; i++)
                {
                    vertexsProjectionFloor[i] = vertexs[vertexs.Length - i - 1];
                    vertexsProjectionFloor[i].y = 0;
                }
            }
            else
            {
                for (int i = 0; i < vertexs.Length; i++)
                {
                    vertexsProjectionFloor[i] = vertexs[i];
                    vertexsProjectionFloor[i].y = 0;
                }
            }
        }

        void CreatePoly()
        {
            poly = geoAlgo.CreatePoly(vertexs, triFaceNormal);
            geoAlgo.CreatePolySelfProjectAxisRange(poly);           
        }

        void CreateProjectionToFloorPoly()
        {
            polyProjectionFloor = geoAlgo.CreatePoly(vertexsProjectionFloor, floorGridNormal);
            geoAlgo.CreatePolySelfProjectAxisRange(polyProjectionFloor);
        }

    
        /// <summary>
        /// 生成投影到TriFace上的pts
        /// </summary>
        /// <param name="rect"></param>
        /// <returns></returns>
        Vector3d[] CreateProjectionToTriFacePts(Vector3d[] pts)
        {
            Vector3d[] cellProjectionRect = new Vector3d[pts.Length];

            for (int i = 0; i < pts.Length; i++)
            {
               geoAlgo.SolveCrossPoint(pts[i], floorGridNormal, vertexs[0], triFaceNormal, out cellProjectionRect[i]);
            }

            return cellProjectionRect;
        }

        /// <summary>
        /// 直接根据单元格的标号,生成投影到TriFace上的Rect
        /// </summary>
        /// <param name="xIdxCell"></param>
        /// <param name="zIdxCell"></param>
        /// <returns></returns>
        Vector3d[] CreateFloorGridCellProjectionRect(int xIdxCell, int zIdxCell)
        {
            Vector3d[] cellRect = voxSpace.GetFloorGridCellRect(xIdxCell, zIdxCell);
            Vector3d[] cellProjectionRect = new Vector3d[4];

            for (int i = 0; i < cellRect.Length; i++)
            {
                geoAlgo.SolveCrossPoint(cellRect[i], floorGridNormal, vertexs[0], triFaceNormal, out cellProjectionRect[i]);
            }

            return cellProjectionRect;
        }


        /// <summary>
        /// 生成地面所有网格投影到TriFace上的体素Box
        /// </summary>
        void CreateFloorGridProjTriFaceVoxBox()
        {
            for (int x = xstartCell; x < xendCell; x++)
            {
                for (int z = zstartCell; z < zendCell; z++)
                {
                    CreateFloorGridCellProjTriFaceVoxBox(x, z);
                }
            }
        }

        /// <summary>
        /// 生成指定地面网格单元格投影到TriFace上的体素Box
        /// </summary>
        /// <param name="cellx"></param>
        /// <param name="cellz"></param>
        void CreateFloorGridCellProjTriFaceVoxBox(int cellx, int cellz)
        {
            Vector3d[] rect = voxSpace.GetFloorGridCellRect(cellx, cellz);
            OverlapRelation relation = geoAlgo.GetOverlapRelation(polyProjectionFloor, rect);
            if (relation == OverlapRelation.NotOverlay)
                return;

            Vector3d[] projectionPts;
            
            if (relation == OverlapRelation.PartOverlay)
            {
                int count = geoAlgo.InRect2DCount(rect[0].x, rect[2].x, rect[0].z, rect[1].z, vertexsProjectionFloor, ref inRectIdx);
                if (count == vertexsProjectionFloor.Length)
                {
                    projectionPts = vertexs;
                }
                else
                {
                    edgePloyPts.Clear();

                    if (faceDirType != DirCmpInfo.Vertical)
                    {
                        for (int i = 0; i < rect.Length; i++)
                        {
                            if (geoAlgo.IsInsidePoly2D(polyProjectionFloor, rect[i]))
                                edgePloyPts.Add(rect[i]);
                        }
                    }

                    for (int i = 0; i < count; i++)
                        edgePloyPts.Add(vertexsProjectionFloor[inRectIdx[i]]);

                    for (int i = 0; i < preCellRectSides.Length; i++)
                        preCellRectSides[i].startpos = rect[i];

                    Vector3d[] pts = geoAlgo.SolvePolySidesCrossPoints2D(preCellRectSides, polyProjectionFloor.sidesList[0]);
                    
                    for (int i = 0; i < pts.Length; i++)
                        edgePloyPts.Add(pts[i]);

                    projectionPts = CreateProjectionToTriFacePts(edgePloyPts.ToArray());
                }
            }
            else
            {
                projectionPts = CreateProjectionToTriFacePts(rect);
            }

            CreateVoxBoxToList(projectionPts, cellx, cellz);
        }

        /// <summary>
        /// 根据投影Rect生成体素box
        /// </summary>
        /// <param name="cellProjectionRect"></param>
        /// <param name="floorGridCenter">在地板单元格的中心位置坐标</param>
        void CreateVoxBoxToList(Vector3d[] projectionPts, int cellx, int cellz)
        {
            double[] boundY = geoAlgo.GetYValueBound(projectionPts);
            int[] gridYIdxs = voxSpace.GetWallGridCellIdxRange(boundY[0], boundY[1]);
            CreateVoxBoxToList(gridYIdxs, cellx, cellz);
        }

        /// <summary>
        /// 根据投影Rect生成体素box
        /// </summary>
        /// <param name="cellProjectionRect"></param>
        /// <param name="floorGridCenter">在地板单元格的中心位置坐标</param>
        void CreateVoxBoxToList(int[] gridYIdxs, int cellx, int cellz)
        {
            VoxBox voxBox = new VoxBox(voxBoxList.Count.ToString(), voxSpace, cellx, cellz, gridYIdxs[0], gridYIdxs[1]);
            voxBoxList.Add(voxBox);
        }

   
        public void CreateVoxBoxViewer()
        {
            voxViewer.CreateVoxs(voxBoxList.ToArray(), voxSpace);
        }
    }
}
