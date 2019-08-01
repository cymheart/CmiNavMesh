using Geometry_Algorithm;
using LinearAlgebra.VectorAlgebra;
using Mathd;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using MINAV;

public class TestMeshBox : MonoBehaviour {

    public List<GameObject> goList;
    public GameObject txta;

    void Start () {
        VoxelSpace voxSpace = new VoxelSpace();
        CalMeshVerts(voxSpace);
        voxSpace.CreateSpaceGrids();

        System.Diagnostics.Stopwatch stopwatch = new System.Diagnostics.Stopwatch();
        stopwatch.Start();

        SolidSpanGroup solidSpanGroup = new SolidSpanGroup(voxSpace);
        voxSpace.CreateVoxels(solidSpanGroup);

        stopwatch.Stop();

        VoxBoxViewer voxBoxViewer = new VoxBoxViewer(voxSpace);
        voxBoxViewer.AppendVoxBoxs(solidSpanGroup);



        long ms = stopwatch.ElapsedMilliseconds;
        txta.transform.GetComponent<Text>().text = "用时:" + ms + "毫秒, " + "vox数量:" + 0 + "," + 0;

        Debug.Log("用时:" + ms + "毫秒");
        Debug.Log("voxel数量:" + 0 + "个");
    }

    void CalMeshVerts(VoxelSpace voxSpace)
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

                voxSpace.TransModelVertexs(vectxs);
            }
        }
    }
	
	// Update is called once per frame
	void Update () {
		
	}
}
