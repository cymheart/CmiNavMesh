using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace Geometry_Algorithm
{
    public class VoxBox
    {
        VoxSpace voxSpace;

        public Vector3 position;
        float[] yPosRange = null;

        public int floorCellIdxX;
        public int floorCellIdxZ;
        public int heightCellStartIdx;
        public int heightCellEndIdx;

        public string name;


        public VoxBox(
            string name,
            VoxSpace voxSpace,
            int floorCellIdxX, int floorCellIdxZ,
            int heightCellStartIdx, int heightCellEndIdx)
        {
            this.name = name;
            this.voxSpace = voxSpace;
            this.floorCellIdxX = floorCellIdxX;
            this.floorCellIdxZ = floorCellIdxZ;
            this.heightCellStartIdx = heightCellStartIdx;
            this.heightCellEndIdx = heightCellEndIdx;

            CreateRealPosition();
        }

        public void CreateRealPosition()
        {
            position = voxSpace.GetFloorGridCellRectCenterPos(floorCellIdxX, floorCellIdxZ);
            yPosRange = voxSpace.GetWallGridCellPosRange(heightCellStartIdx, heightCellEndIdx);
            position.y = (yPosRange[0] + yPosRange[1]) / 2f;
        }
        
        public int GetHeightCellRangeCount()
        {
            return heightCellEndIdx - heightCellStartIdx;
        }
    }
}
