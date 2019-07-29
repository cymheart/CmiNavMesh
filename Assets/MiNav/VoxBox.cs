using Mathd;

namespace MINAV
{
    public struct VoxBox
    {
        public SimpleVector3 position;
        public float yPosStart;
        public float yPosEnd;

        public int floorCellIdxX;
        public int floorCellIdxZ;
        public int heightCellStartIdx;
        public int heightCellEndIdx;



        public VoxBox(
            string name,
            VoxelSpace voxSpace,
            int floorCellIdxX, int floorCellIdxZ,
            int heightCellStartIdx, int heightCellEndIdx)
        {
            this.floorCellIdxX = floorCellIdxX;
            this.floorCellIdxZ = floorCellIdxZ;
            this.heightCellStartIdx = heightCellStartIdx;
            this.heightCellEndIdx = heightCellEndIdx;
            yPosStart = heightCellStartIdx * voxSpace.cellHeight;
            yPosEnd = heightCellEndIdx * voxSpace.cellHeight;

            position = new SimpleVector3(0, 0, 0);

        }

        public void CreateRealPosition(VoxelSpace voxSpace)
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
