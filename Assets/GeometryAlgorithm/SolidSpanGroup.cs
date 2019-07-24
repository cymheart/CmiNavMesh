using Mathd;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Geometry_Algorithm
{
    /// <summary>
    /// 实心跨距
    /// </summary>
    public struct SolidSpan
    {
        public double startPos;
        public double endPos;

        public int startCellIdx;
        public int endCellIdx;
    }

    
    /// <summary>
    /// 实心跨距组
    /// </summary>
    public class SolidSpanGroup
    {
        public Dictionary<int, LinkedList<SolidSpan>> soildSpanDict = new Dictionary<int, LinkedList<SolidSpan>>();

        public void AppendVoxBox(VoxBox voxBox)
        {
            int key = GetKey(voxBox.floorCellIdxX, voxBox.floorCellIdxZ);
            LinkedList<SolidSpan> cellSpanList;

            if (soildSpanDict.TryGetValue(key, out cellSpanList) == false)
            {
                cellSpanList = new LinkedList<SolidSpan>();
                soildSpanDict[key] = cellSpanList;
            }

            AppendVoxBoxToSpanHeightList(cellSpanList, voxBox);
        }

        void AppendVoxBoxToSpanHeightList(LinkedList<SolidSpan> cellSpanList, VoxBox voxBox)
        {
            int voxStartIdx = voxBox.heightCellStartIdx;
            int voxEndIdx = voxBox.heightCellStartIdx;
            double yPosStart = voxBox.yPosRange[0];
            double yPosEnd = voxBox.yPosRange[1];    

            LinkedListNode<SolidSpan> startNode = null;
            LinkedListNode<SolidSpan> endNode = null;

            var node = cellSpanList.First;
            for (; node != null; node = node.Next)
            {
                if(node.Value.startCellIdx > voxStartIdx && startNode == null)
                {
                    startNode = node;
                }
                else if (voxStartIdx >= node.Value.startCellIdx  &&
                    voxStartIdx <= node.Value.endCellIdx)
                {
                    yPosStart = node.Value.startPos;
                    voxStartIdx = node.Value.startCellIdx;
                    startNode = node;
                }

                if (node.Value.startCellIdx > voxEndIdx && endNode == null)
                {
                    endNode = node.Previous;
                    break;
                }
                else if (voxEndIdx >= node.Value.startCellIdx && 
                    voxEndIdx <= node.Value.endCellIdx)
                {
                    yPosEnd = node.Value.endPos;
                    voxEndIdx = node.Value.endCellIdx;
                    endNode = node;
                    break;
                }
            }

            if(startNode != null && endNode == null)
                endNode = cellSpanList.Last;

            SolidSpan voxSpan = new SolidSpan()
            {
                startPos = yPosStart,
                endPos = yPosEnd,
                startCellIdx = voxStartIdx,
                endCellIdx = voxEndIdx
            };

            if (startNode == null && endNode == null)
            {
                if(node == cellSpanList.First)
                    cellSpanList.AddFirst(voxSpan);
                else
                    cellSpanList.AddLast(voxSpan);
            }     
            else
            {
                var prevNode = startNode.Previous;
                var mnode = startNode;
                LinkedListNode<SolidSpan> tmpNode;
                bool flag = true;

                while(flag)
                {
                    if (mnode == endNode)
                        flag = false;

                    tmpNode = mnode.Next;
                    cellSpanList.Remove(mnode);
                    mnode = tmpNode;
                }

                if(prevNode == null)
                    cellSpanList.AddFirst(voxSpan);
                else
                    cellSpanList.AddAfter(prevNode, voxSpan);
            }
        }

        public int GetKey(int cellx, int cellz)
        {
            if (cellx < 0 || cellz < 0)
                return -1;

            return (cellx << 14) | cellz; 
        }

        public int[] GetCellIdxs(int key)
        {
            int[] idxs = new int[] { key >> 14, 0x3FFF & key };
            return idxs;
        }

    }
}
