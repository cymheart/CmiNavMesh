using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace Geometry_Algorithm
{
    public class PolySplitTriangles
    {
        GeometryAlgorithm geoAlgor = new GeometryAlgorithm();
        LinkedList<Vector3> polyVertList = new LinkedList<Vector3>();
        LinkedList<LinkedListNode<Vector3>[]> triList = new LinkedList<LinkedListNode<Vector3>[]>();

        void dd(Poly poly)
        {
            PolySide[] sides = poly.sidesList[0];
            Vector3 dir;
            LinkedListNode<Vector3>[] abc;

            CreatePolyVertToList(poly);


            for (var node = polyVertList.First; node != null;)
            {
                abc = GetTriNodes(node);
                Vector3 ab = abc[1].Value - abc[0].Value;
                Vector3 bc = abc[2].Value - abc[1].Value;

                dir = Vector3.Cross(ab, bc);
                int ret = geoAlgor.CmpParallelVecDir(dir, poly.faceNormal);
                if (ret == -1)
                    continue;

                Vector3 ac = abc[2].Value - abc[0].Value;
                Vector3 ba = abc[0].Value - abc[1].Value;
                Vector3 ca = abc[0].Value - abc[2].Value;
                Vector3 cb = abc[1].Value - abc[2].Value;
                Vector3 aCross = Vector3.Cross(ab, ac);
                Vector3 bCross = Vector3.Cross(bc, ba);
                Vector3 cCross = Vector3.Cross(ca, cb);


                //
                bool isInTri = false;
                for (var node2 = polyVertList.First; node2 != null; node2 = node2.Next)
                {
                    if (node2 == abc[0] || node2 == abc[1] || node2 == abc[2])
                        continue;

                    Vector3 pt = node2.Value;
                    isInTri |= TestPointInTri(pt, abc[0].Value, abc[1].Value, abc[2].Value, aCross, bCross, cCross);
                }

                if (isInTri == false)
                    triList.AddLast(abc);
            }

        }

        void CreatePolyVertToList(Poly poly)
        {
            Vector3[] triPts = poly.vertexsList[0];
            for (int i = 0; i < triPts.Length; i++)
            {
                polyVertList.AddLast(triPts[i]);
            }
        }


        LinkedListNode<Vector3>[] GetTriNodes(LinkedListNode<Vector3> firstNode)
        {
            LinkedListNode<Vector3>[] triNodes = new LinkedListNode<Vector3>[3];

            triNodes[0] = firstNode;

            if (firstNode.Next == null)
            {
                triNodes[1] = polyVertList.First;
                triNodes[2] = polyVertList.First.Next;
            }
            else
            {
                triNodes[1] = firstNode.Next;

                if (firstNode.Next.Next == null)
                {
                    triNodes[2] = polyVertList.First;
                }
                else
                {
                    triNodes[2] = firstNode.Next.Next;
                }
            }

            return triNodes;
        }


        bool TestPointInTri(Vector3 pt, 
            Vector3 triPtA, Vector3 triPtB, Vector3 triPtC,
            Vector3 aCross, Vector3 bCross, Vector3 cCross)
        {
            Vector3 ab = triPtB - triPtA;
            Vector3 ad = pt - triPtA;
            Vector3 s2 = Vector3.Cross(ab, ad); 
            int ret = geoAlgor.CmpParallelVecDir(aCross, s2);
            if (ret == -1)
                return false;

            Vector3 bc = triPtC - triPtB;
            Vector3 bd = pt - triPtB;
            s2 = Vector3.Cross(bc, bd);
            ret = geoAlgor.CmpParallelVecDir(bCross, s2);
            if (ret == -1)
                return false;

            Vector3 ca = triPtA - triPtC;
            Vector3 cd = pt - triPtC;
            s2 = Vector3.Cross(ca, cd);
            ret = geoAlgor.CmpParallelVecDir(cCross, s2);
            if (ret == -1)
                return false;

            return true;
        }

    }
}
