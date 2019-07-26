using Mathd;

namespace Geometry_Algorithm
{
    public class VoxBox
    {
        VoxSpace voxSpace;
        public string name;

        public Vector3d position;
        public double yPosStart;
        public double yPosEnd;

        public int floorCellIdxX;
        public int floorCellIdxZ;
        public int heightCellStartIdx;
        public int heightCellEndIdx;



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
            yPosStart = heightCellStartIdx * voxSpace.cellHeight;
            yPosEnd = heightCellEndIdx * voxSpace.cellHeight;

        }

        public void CreateRealPosition()
        {
            position = voxSpace.GetFloorGridCellRectCenterPos(floorCellIdxX, floorCellIdxZ);
            position.y = (yPosStart + yPosEnd) / 2f;
        }
  
        public int GetHeightCellRangeCount()
        {
            return heightCellEndIdx - heightCellStartIdx;
        }
    }
}
