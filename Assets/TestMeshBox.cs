using Geometry_Algorithm;
using LinearAlgebra.VectorAlgebra;
using Mathd;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using MINAV;
using System.Runtime.InteropServices;
using System;
using System.IO;

public class TestMeshBox : MonoBehaviour {

    

    public List<GameObject> goList;
    public GameObject txta;


    void Start () {
    }

    public void Click()
    {
        IntPtr voxSpace = ExportFunc.CreateVoxelSpace();
        CalMeshVerts2(voxSpace);

        System.Diagnostics.Stopwatch stopwatch = new System.Diagnostics.Stopwatch();
        stopwatch.Start();

        ExportFunc.SetCellSize(voxSpace, 0.1f, 0.1f);
        ExportFunc.CreateSpaceGrids(voxSpace);
        IntPtr solidSpanGroup = ExportFunc.CreateSolidSpanGroup(voxSpace);
        ExportFunc.CreateVoxels(voxSpace, solidSpanGroup);

        stopwatch.Stop();

       // VoxBoxViewer voxBoxViewer = new VoxBoxViewer(null);
       // voxBoxViewer.AppendVoxBoxs(voxSpace, solidSpanGroup);

        long ms = stopwatch.ElapsedMilliseconds;
        txta.transform.GetComponent<Text>().text = "用时:" + ms + "毫秒, " + "vox数量:" + 0 + "," + 0;

        Debug.Log("用时:" + ms + "毫秒");
        Debug.Log("voxel数量:" + 0 + "个");
    }

    void CalMeshVerts2(IntPtr voxSpace)
    {
        Transform tf;
        for (int j = 0; j < goList[0].transform.childCount; j++)
        {
            tf = goList[0].transform.GetChild(j);
            MeshFilter mf = tf.GetComponent<MeshFilter>();
            Vector3[] vectors = mf.mesh.vertices;
            Vector3[] normals = mf.mesh.normals;
            int[] idxs = mf.mesh.triangles;
            Vector3[] vects = new Vector3[3];
            Vector[] vectxs = new Vector[3];

            for (int i = 0; i < idxs.Length; i += 3)
            {
                vects[0] = vectors[idxs[i]];
                vects[1] = vectors[idxs[i + 1]];
                vects[2] = vectors[idxs[i + 2]];

                Vector3 v = mf.transform.localToWorldMatrix.MultiplyPoint(vects[0]);
                vectxs[0] = new Vector(new double[] { v.x, v.y, v.z, 1 }, VectorType.Column);

                v = mf.transform.localToWorldMatrix.MultiplyPoint(vects[1]);
                vectxs[1] = new Vector(new double[] { v.x, v.y, v.z, 1 }, VectorType.Column);

                v = mf.transform.localToWorldMatrix.MultiplyPoint(vects[2]);
                vectxs[2] = new Vector(new double[] { v.x, v.y, v.z, 1 }, VectorType.Column);

                SimpleVector3 vs = new SimpleVector3()
                {
                    x = (float)vectxs[0].Elements[0],
                    y = (float)vectxs[0].Elements[1],
                    z = (float)vectxs[0].Elements[2]
                };

                SimpleVector3 vs1 = new SimpleVector3()
                {
                    x = (float)vectxs[1].Elements[0],
                    y = (float)vectxs[1].Elements[1],
                    z = (float)vectxs[1].Elements[2]
                };

                SimpleVector3 vs2 = new SimpleVector3()
                {
                    x = (float)vectxs[2].Elements[0],
                    y = (float)vectxs[2].Elements[1],
                    z = (float)vectxs[2].Elements[2]
                };


                ExportFunc.TransModelVertexs(voxSpace, vs, vs1, vs2);
            }
        }
    }


    public void Click2()
    {
        VoxelSpace voxSpace = new VoxelSpace();
        CalMeshVerts(voxSpace);

        System.Diagnostics.Stopwatch stopwatch = new System.Diagnostics.Stopwatch();
        stopwatch.Start();

        voxSpace.CreateSpaceGrids();
        SolidSpanGroup solidSpanGroup = new SolidSpanGroup(voxSpace);

        voxSpace.CreateVoxels(solidSpanGroup);

        stopwatch.Stop();

        //VoxBoxViewer voxBoxViewer = new VoxBoxViewer(voxSpace);
       // voxBoxViewer.AppendVoxBoxs(solidSpanGroup);

        long ms = stopwatch.ElapsedMilliseconds;
        txta.transform.GetComponent<Text>().text = "用时:" + ms + "毫秒, " + "vox数量:" + 0 + "," + 0;

        Debug.Log("用时:" + ms + "毫秒");
        Debug.Log("voxel数量:" + 0 + "个");
    }

    void CalMeshVerts(VoxelSpace voxSpace)
    {
        FileInfo myFile = new FileInfo(@"j:\verts.txt");
        StreamWriter sw = myFile.CreateText();

        Transform tf;
        for (int j = 0; j < goList[0].transform.childCount; j++)
        {
            tf = goList[0].transform.GetChild(j);
            MeshFilter mf = tf.GetComponent<MeshFilter>();
            Vector3[] vectors = mf.mesh.vertices;
            Vector3[] normals = mf.mesh.normals;
            int[] idxs = mf.mesh.triangles;
            Vector3[] vects = new Vector3[3];
            Vector[] vectxs = new Vector[3];

            for (int i = 0; i < idxs.Length; i += 3)
            {
                vects[0] = vectors[idxs[i]];
                vects[1] = vectors[idxs[i + 1]];
                vects[2] = vectors[idxs[i + 2]];

                Vector3 v = mf.transform.localToWorldMatrix.MultiplyPoint(vects[0]);
                vectxs[0] = new Vector(new double[] { v.x, v.y, v.z, 1 }, VectorType.Column);

                v = mf.transform.localToWorldMatrix.MultiplyPoint(vects[1]);
                vectxs[1] = new Vector(new double[] { v.x, v.y, v.z, 1 }, VectorType.Column);

                v = mf.transform.localToWorldMatrix.MultiplyPoint(vects[2]);
                vectxs[2] = new Vector(new double[] { v.x, v.y, v.z, 1 }, VectorType.Column);

                SimpleVector3 vs = new SimpleVector3()
                {
                    x = (float)vectxs[0].Elements[0],
                    y = (float)vectxs[0].Elements[1],
                    z = (float)vectxs[0].Elements[2]
                };

                SimpleVector3 vs1 = new SimpleVector3()
                {
                    x = (float)vectxs[1].Elements[0],
                    y = (float)vectxs[1].Elements[1],
                    z = (float)vectxs[1].Elements[2]
                };

                SimpleVector3 vs2 = new SimpleVector3()
                {
                    x = (float)vectxs[2].Elements[0],
                    y = (float)vectxs[2].Elements[1],
                    z = (float)vectxs[2].Elements[2]
                };

                SimpleVector3[] vsGroup = new SimpleVector3[3]
                {
                    vs, vs1,vs2
                };

                WriteVerts(sw, vsGroup);

              //  voxSpace.TransModelVertexs(vsGroup);
            }
        }

        sw.Close();
    }

    void WriteVerts(StreamWriter sw, SimpleVector3[] verts)
    {
        string totalTxt = "";
        string txt;
        for (int i = 0; i < verts.Length; i++)
        {
            txt = verts[i].x + " " + verts[i].y + " " + verts[i].z;
            totalTxt += " " + txt;
        }

        sw.WriteLine(totalTxt);
    }
	
	// Update is called once per frame
	void Update () {
		
	}
}
