using System;
using System.Collections.Generic;
using UnityEngine;

public class RaycastHitInfoVerbose : RaycastHitInfo
{
    private float _triangleEdgeFacing;

    public float triangleEdgeFacing { get { return _triangleEdgeFacing; } set { _triangleEdgeFacing = value; } } //Integer part dictates which edge is being faced; fractional part dictates how far along the edge

    public RaycastHitInfoVerbose(RaycastHit hit, Vector3 casterForward) : base(hit)
    {
        if (hit.collider is MeshCollider)
        {
            _triangleEdgeFacing = GetTriangleEdgeFacing(hit.point, casterForward, hit.normal, ((MeshCollider)hit.collider).sharedMesh, hit.triangleIndex);
        }
        else
        {
            _triangleEdgeFacing = -1f;
        }
    }

    public RaycastHitInfoVerbose(Transform hitTransform, ArticulationBody hitArticulationBody, Rigidbody hitRigidbody, Collider hitCollider, int hitTriangleIndex, float hitDistance, Vector3 hitNormal, Vector3 hitPoint, Vector3 hitBarycentricCoordinate, Vector3 casterForward) : base(hitTransform, hitArticulationBody, hitRigidbody, hitCollider, hitTriangleIndex, hitDistance, hitNormal, hitPoint, hitBarycentricCoordinate)
    {
        if (hitCollider is MeshCollider)
        {
            _triangleEdgeFacing = GetTriangleEdgeFacing(hitPoint, casterForward, hitNormal, ((MeshCollider)hitCollider).sharedMesh, hitTriangleIndex);
        }
        else
        {
            _triangleEdgeFacing = -1f;
        }
    }

    public static float GetTriangleEdgeFacing(Vector3 point, Vector3 forward, Vector3 normal, Mesh mesh, int triangleIndex)
    {
        int[] triangles = mesh.triangles;
        Vector3[] vertices = mesh.vertices;
        return GetTriangleEdgeFacing(point, forward, normal, vertices[triangles[triangleIndex * 3]], vertices[triangles[triangleIndex * 3 + 1]], vertices[triangles[triangleIndex * 3 + 2]]);
    }

    public static float GetTriangleEdgeFacing(Vector3 point, Vector3 forward, Vector3 normal, Vector3 vertex1, Vector3 vertex2, Vector3 vertex3)
    {
        Vector3 directionOnTrianglePlane = forward.RemoveComponentAlongAxis(normal).normalized;
        Quaternion rotation = Quaternion.LookRotation(directionOnTrianglePlane, normal).ShortestRotation(new Quaternion());

        List<Vector3> relativeVertices = new List<Vector3>() { rotation * (vertex1 - point), rotation * (vertex2 - point), rotation * (vertex3 - point) };
        List<float> vertexAngles = new List<float>();
        foreach (Vector3 vertex in relativeVertices)
        {
            float angle = vertex.normalized.SignedAngle(Vector3.forward, Vector3.up);
            vertexAngles.Add(angle < 0f ? angle + 360 : angle);
        }

        List<float> sortedVertexAngles = new List<float>(vertexAngles);
        sortedVertexAngles.Sort();
        sortedVertexAngles.RemoveAt(1);

        float edgeFacing;
        if (sortedVertexAngles.Contains(vertexAngles[0]))
        {
            if (sortedVertexAngles.Contains(vertexAngles[1]))
            {
                edgeFacing = 0f;
            }
            else
            {
                edgeFacing = 2f;
            }
        }
        else
        {
            edgeFacing = 1f;
        }

        int rightVertexIndex = vertexAngles.IndexOf(sortedVertexAngles[0]);
        int leftVertexIndex = vertexAngles.IndexOf(sortedVertexAngles[1]);

        if (leftVertexIndex < rightVertexIndex)
        {
            edgeFacing -= relativeVertices[leftVertexIndex].x / (relativeVertices[rightVertexIndex].x - relativeVertices[leftVertexIndex].x);
        }
        else
        {
            edgeFacing += relativeVertices[rightVertexIndex].x / (relativeVertices[rightVertexIndex].x - relativeVertices[leftVertexIndex].x);
        }

        return edgeFacing;
    }

    public static Vector3 GetTriangleEdgePointFaced(float edgeFacing, Vector3 vertex1, Vector3 vertex2, Vector3 vertex3)
    {
        List<Vector3> vertices = new List<Vector3>() { vertex1, vertex2, vertex3 };
        int index = (int)Math.Floor(edgeFacing);

        return Vector3.Lerp(vertices[index], vertices[(index + 1) % 3], edgeFacing - index);
    }
}