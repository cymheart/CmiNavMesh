using Mathd;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace Geometry_Algorithm
{
    public class VoxViewer
    {
        List<GameObject> voxList = new List<GameObject>();
        VoxSpace voxSpace;

        public void CreateVoxs(VoxBox[] voxBoxs, VoxSpace voxSpace)
        {
            this.voxSpace = voxSpace;

            Vector3 size = new Vector3();
            GameObject vox;

            for(int i=0; i<voxBoxs.Length; i++)
            {
                size.Set((float)voxSpace.cellSize,
                    (float)(voxBoxs[i].GetHeightCellRangeCount() * voxSpace.cellHeight),
                    (float)voxSpace.cellSize);

                Vector3 pos = new Vector3((float)voxBoxs[i].position.x, (float)voxBoxs[i].position.y, (float)voxBoxs[i].position.z);
                vox = CreateVoxBoxMesh(pos, size, voxBoxs[i].name);
                voxList.Add(vox);
            }
        }


        GameObject CreateVoxBoxMesh(Vector3 centerPos, Vector3 size, string name)
        {
            GameObject vox = new GameObject(name);
            MeshFilter mf = vox.AddComponent<MeshFilter>();
            MeshRenderer mr = vox.AddComponent<MeshRenderer>();

            Vector3[] vertices = new Vector3[24];
            Vector3[] normals = new Vector3[24];
            int[] triangles = new int[36];

            float xw = size.x / 2f;
            float yw = size.y / 2f;
            float zw = size.z / 2f;
            float cx = 0;
            float cy = 0;
            float cz = 0;

            //forward
            vertices[0].Set(cx + xw, cy - yw, cz + zw);
            vertices[1].Set(cx - xw, cy - yw, cz + zw);
            vertices[2].Set(cx + xw, cy + yw, cz + zw);
            vertices[3].Set(cx - xw, cy + yw, cz + zw);

            normals[0] = Vector3.forward;
            normals[1] = Vector3.forward;
            normals[2] = Vector3.forward;
            normals[3] = Vector3.forward;

            //back
            vertices[4].Set(vertices[2].x, vertices[2].y, cz - zw);
            vertices[5].Set(vertices[3].x, vertices[3].y, cz - zw);
            vertices[6].Set(vertices[0].x, vertices[0].y, cz - zw);
            vertices[7].Set(vertices[1].x, vertices[1].y, cz - zw);

            normals[4] = Vector3.back;
            normals[5] = Vector3.back;
            normals[6] = Vector3.back;
            normals[7] = Vector3.back;

            //up
            vertices[8] = vertices[2];
            vertices[9] = vertices[3];
            vertices[10] = vertices[4];
            vertices[11] = vertices[5];

            normals[8] = Vector3.up;
            normals[9] = Vector3.up;
            normals[10] = Vector3.up;
            normals[11] = Vector3.up;

            //down
            vertices[12].Set(vertices[10].x, cy - yw, vertices[10].z);
            vertices[13].Set(vertices[11].x, cy - yw, vertices[11].z);
            vertices[14].Set(vertices[8].x, cy - yw, vertices[8].z);
            vertices[15].Set(vertices[9].x, cy - yw, vertices[9].z);

            normals[12] = Vector3.down;
            normals[13] = Vector3.down;
            normals[14] = Vector3.down;
            normals[15] = Vector3.down;

            //right
            vertices[16] = vertices[6];
            vertices[17] = vertices[0];
            vertices[18] = vertices[4];
            vertices[19] = vertices[2];

            normals[16] = Vector3.right;
            normals[17] = Vector3.right;
            normals[18] = Vector3.right;
            normals[19] = Vector3.right;


            //left
            vertices[20].Set(cx - xw, vertices[18].y, vertices[18].z);
            vertices[21].Set(cx - xw, vertices[19].y, vertices[19].z);
            vertices[22].Set(cx - xw, vertices[16].y, vertices[16].z);
            vertices[23].Set(cx - xw, vertices[17].y, vertices[17].z);

            normals[20] = Vector3.left;
            normals[21] = Vector3.left;
            normals[22] = Vector3.left;
            normals[23] = Vector3.left;

            int currentCount = 0;
            for (int i = 0; i < 24; i = i + 4)
            {
                triangles[currentCount++] = i;
                triangles[currentCount++] = i + 3;
                triangles[currentCount++] = i + 1;

                triangles[currentCount++] = i;
                triangles[currentCount++] = i + 2;
                triangles[currentCount++] = i + 3;

            }

            //for (int i = 0; i < vertices.Length; i++)
            //{
            //    vertices[i] = voxSpace.voxSpaceToWorld.MultiplyPoint(vertices[i]);
            //    normals[i] = voxSpace.voxSpaceToWorld.MultiplyPoint(normals[i]);
            //}

            mf.mesh.vertices = vertices;
            mf.mesh.triangles = triangles;
            mf.mesh.normals = normals;
            mf.gameObject.transform.position = centerPos;

            Material mat = new Material(Shader.Find("Standard"));
            (mf.gameObject.GetComponent<Renderer>() as Renderer).material = mat;

            return vox;
        }
    }
}
