using Geometry_Algorithm;
using LinearAlgebra.VectorAlgebra;
using Mathd;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestMeshBox : MonoBehaviour {

    // Use this for initialization
    VoxTriFace voxTriFace = new VoxTriFace();

    void Start () {

        MeshFilter mf = GetComponent<MeshFilter>();
        Vector3[] vectors = mf.mesh.vertices;
        Vector3[] normals = mf.mesh.normals;
        int[] idxs = mf.mesh.triangles;

        Vector3[] vects = new Vector3[3];

        vects[0] = vectors[idxs[0]];
        vects[1] = vectors[idxs[1]];
        vects[2] = vectors[idxs[2]];

        Vector[] vectxs = new Vector[3];

        Vector3 v = mf.transform.localToWorldMatrix.MultiplyPoint(vects[0]);
        vectxs[0] = new Vector(new double[] { v.x, v.y, v.z,1 }, VectorType.Column);

        v = mf.transform.localToWorldMatrix.MultiplyPoint(vects[1]);
        vectxs[1] = new Vector(new double[] { v.x, v.y, v.z,1 }, VectorType.Column);

        v = mf.transform.localToWorldMatrix.MultiplyPoint(vects[2]);
        vectxs[2] = new Vector(new double[] { v.x, v.y, v.z, 1 }, VectorType.Column);

        VoxSpace voxSpace = new VoxSpace();

        voxTriFace.SetVoxSpace(voxSpace);
        voxTriFace.TransTriFaceWorldVertexToVoxSpace(vectxs);
        voxTriFace.CreateVoxBoxViewer();
    }
	
	// Update is called once per frame
	void Update () {
		
	}
}
