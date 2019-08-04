using Mathd;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MINAV
{
    /// <summary>
    /// 实心跨距组
    /// </summary>
    public class SolidSpanGroup
    {
        VoxelSpace voxSpace;
        public unsafe SolidSpanList* solidSpanGrids;
        public int gridCount = 0;

        public SolidSpanGroup(VoxelSpace voxSpace)
        {
            unsafe
            {
                this.voxSpace = voxSpace;
                solidSpanGrids = voxSpace.solidSpanGrids;
                gridCount = voxSpace.gridCount;
            }
        }

        public void AppendVoxBox(
            int floorCellIdxX, int floorCellIdxZ,
           int heightCellStartIdx, int heightCellEndIdx)
        {
            unsafe
            {
                int idx = voxSpace.GetFloorGridIdx(floorCellIdxX, floorCellIdxZ);
                SolidSpanList* solidSpanList = &(solidSpanGrids[idx]);

                SolidSpanList tmp = *solidSpanList;

                if (solidSpanList->first == null)
                {
                    solidSpanList->floorCellIdxX = floorCellIdxX;
                    solidSpanList->floorCellIdxZ = floorCellIdxZ;
                }

                AppendVoxBoxToSpanHeightList(solidSpanList, heightCellStartIdx, heightCellEndIdx);
            }
        }


        unsafe void AppendVoxBoxToSpanHeightList(
            SolidSpanList* solidSpanList, 
            int heightCellStartIdx, int heightCellEndIdx)
        {
            int voxStartIdx = heightCellStartIdx;
            int voxEndIdx = heightCellEndIdx;
            float yPosStart = voxStartIdx * voxSpace.cellHeight;
            float yPosEnd = voxEndIdx * voxSpace.cellHeight;

            SolidSpan* startNode = null;
            SolidSpan* endNode = null;

            SolidSpan* node = solidSpanList->first;
            for (; node != null; node = node->next)
            {
                if (startNode == null)
                {
                    if (node->ystartCellIdx > voxStartIdx)
                    {
                        startNode = node;
                    }
                    else if (voxStartIdx >= node->ystartCellIdx &&
                        voxStartIdx <= node->yendCellIdx)
                    {
                        yPosStart = node->ystartPos;
                        voxStartIdx = node->ystartCellIdx;
                        startNode = node;
                    }
                }

                if (endNode == null)
                {
                    if (node->ystartCellIdx > voxEndIdx)
                    {
                        endNode = node->prev;
                        break;
                    }
                    else if (voxEndIdx >= node->ystartCellIdx &&
                        voxEndIdx <= node->yendCellIdx)
                    {
                        yPosEnd = node->yendPos;
                        voxEndIdx = node->yendCellIdx;
                        endNode = node;
                        break;
                    }
                }
            }
   
            if (startNode != null && node == null)
                endNode = solidSpanList->last;


            SolidSpan* voxSpan = voxSpace.GetSoildSpan();
            voxSpan->ystartPos = yPosStart;
            voxSpan->yendPos = yPosEnd;
            voxSpan->ystartCellIdx = voxStartIdx;
            voxSpan->yendCellIdx = voxEndIdx;

            if (endNode != null && endNode->next == startNode)
            {
                solidSpanList->AddAfter(endNode, voxSpan);
                return;
            }
            
            if(startNode != null && endNode == null)
            {
                solidSpanList->AddFirst(voxSpan);
                return;
            }

            if (startNode == null && endNode == null)
            {
                if (node == solidSpanList->first)
                    solidSpanList->AddFirst(voxSpan);
                else
                    solidSpanList->AddLast(voxSpan);
            }
            else
            {
                var prevNode = startNode->prev;
                var mnode = startNode;
                SolidSpan* tmpNode;
                bool flag = true;

                while (flag)
                {
                    if (mnode == endNode)
                        flag = false;

                    tmpNode = mnode->next;
                    solidSpanList->Remove(mnode);
                    mnode = tmpNode;
                }

                if (prevNode == null)
                    solidSpanList->AddFirst(voxSpan);
                else
                    solidSpanList->AddAfter(prevNode, voxSpan);
            }
        }
    }
}
