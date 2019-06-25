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
        Vector3[] vertexs;
        Vector3[] vertexsProjectionFloor;
        Poly poly;
        Poly polyProjectionFloor;
        Vector3 triFaceNormal;
        Vector3 floorGridNormal = Vector3.up;
        DirCmpInfo faceDirType = DirCmpInfo.Same;

        List<VoxBox> voxBoxList = new List<VoxBox>();
        VoxViewer voxViewer = new VoxViewer();
        GeometryAlgorithm geoAlgo = new GeometryAlgorithm();

        AABB aabb = new AABB() {minX = 999999f, maxX = 0f, minZ = 999999f, maxZ = 0f };
        int xstartCell, xendCell;
        int zstartCell, zendCell;


        int[] inRectIdx;
        int[] inTriFaceIdx;
        List<Vector3> edgePloyPts = new List<Vector3>();

        Vector3[] preCellRect;
        PolySide[] preCellRectSides;
        public void SetVoxSpace(VoxSpace voxSpace)
        {
            this.voxSpace = voxSpace;
        }

        public void TransTriFaceWorldVertexToVoxSpace(Vector3[] triFaceWorldVertex)
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
            preCellRect = new Vector3[]
            {
                Vector3.zero,
                new Vector3(0, 0, voxSpace.cellSize),
                new Vector3(voxSpace.cellSize, 0, voxSpace.cellSize),
                new Vector3(voxSpace.cellSize, 0, 0),
            };

            preCellRectSides = geoAlgo.CreateBaseDataPolySides(preCellRect);


            //
            CreateFloorGridProjTriFaceVoxBox();   
        }

        /// <summary>
        /// 转换三角面的世界顶点到体素空间
        /// </summary>
        /// <param name="triFaceWorldVertex"></param>
        void _TransTriFaceWorldVertexToVoxSpace(Vector3[] triFaceWorldVertex)
        {
            vertexs = new Vector3[triFaceWorldVertex.Length];

            Vector3 vec;
            for(int i=0;i < triFaceWorldVertex.Length; i++)
            {
                vec = voxSpace.worldToVoxSpace.MultiplyPoint(triFaceWorldVertex[i]);
                vertexs[i] = vec;

                if (vec.x > aabb.maxX) { aabb.maxX = vec.x; }
                if (vec.x < aabb.minX) { aabb.minX = vec.x; }
                if (vec.z > aabb.maxZ) { aabb.maxZ = vec.z; }
                if (vec.z < aabb.minZ) { aabb.minZ = vec.z; }
                if (vec.y > aabb.maxY) { aabb.maxY = vec.y; }
                if (vec.y < aabb.minY) { aabb.minY = vec.y; }        
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
            Vector3 vec1 = vertexs[1] - vertexs[0];
            Vector3 vec2 = vertexs[2] - vertexs[0];
            triFaceNormal = Vector3.Cross(vec1, vec2);
            faceDirType = geoAlgo.CmpVectorDir(triFaceNormal, Vector3.up);
        }


        /// <summary>
        /// 计算三角面在floorGrid的格子范围
        /// </summary>
        void CalFloorGridIdxRange()
        {
            //xstartCell
            float n = aabb.minX / voxSpace.cellSize;
            xstartCell = (int)Mathf.Floor(n);

            //xendCell
            n = aabb.maxX / voxSpace.cellSize;
            xendCell = (int)Mathf.Ceil(n);
            if (xstartCell == xendCell) xendCell++;

            //zstartCell
            n = aabb.minZ / voxSpace.cellSize;
            zstartCell = (int)Mathf.Floor(n);

            //zendCell
            n = aabb.maxZ / voxSpace.cellSize;
            zendCell = (int)Mathf.Ceil(n);
            if (zstartCell == zendCell) zendCell++;

        }

      
        /// <summary>
        /// 生成三角面到地面的投影顶点
        /// </summary>
        void CreateProjectionToFloorTriFaceVertexs()
        {
            vertexsProjectionFloor = new Vector3[vertexs.Length];

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
        Vector3[] CreateProjectionToTriFacePts(Vector3[] pts)
        {
            Vector3[] cellProjectionRect = new Vector3[pts.Length];

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
        Vector3[] CreateFloorGridCellProjectionRect(int xIdxCell, int zIdxCell)
        {
            Vector3[] cellRect = voxSpace.GetFloorGridCellRect(xIdxCell, zIdxCell);
            Vector3[] cellProjectionRect = new Vector3[4];

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
            Vector3[] rect = voxSpace.GetFloorGridCellRect(cellx, cellz);
            OverlapRelation relation = geoAlgo.GetOverlapRelation(polyProjectionFloor, rect);
            if (relation == OverlapRelation.NotOverlay)
                return;

            Vector3[] projectionPts;
            
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

                    Vector3[] pts = geoAlgo.SolvePolySidesCrossPoints2D(preCellRectSides, polyProjectionFloor.sidesList[0]);
                    
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
        void CreateVoxBoxToList(Vector3[] projectionPts, int cellx, int cellz)
        {
            float[] boundY = geoAlgo.GetYValueBound(projectionPts);
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
