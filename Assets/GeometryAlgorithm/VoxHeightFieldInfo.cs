using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Geometry_Algorithm
{

    public class VoxHeightFieldInfo
    {
        Dictionary<int, List<VoxHeightSpan>> heightSpanDict = new Dictionary<int, List<VoxHeightSpan>>();

        public void AddVoxBox(VoxBox voxBox)
        {
            int key = GetKey(voxBox.floorCellIdxX, voxBox.floorCellIdxZ);
            List<VoxHeightSpan> cellSpanList;

            if (heightSpanDict.TryGetValue(key, out cellSpanList) == false)
            {
                cellSpanList = new List<VoxHeightSpan>();
                heightSpanDict[key] = cellSpanList;
            }
        }


        void dd()
        {

        }

        int GetKey(int cellx, int cellz)
        {
            return (cellx << 14) | cellz; 
        }

    }
}
