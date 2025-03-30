using UnityEngine;

public class MeshUpdater : MonoBehaviour
{
    [SerializeField] private SkinnedMeshRenderer skinnedMeshRenderer;
    [SerializeField] private MeshCollider meshCollider;

    public void Apply()
    {
        Mesh mesh = new Mesh();
        skinnedMeshRenderer.BakeMesh(mesh, true);
        meshCollider.sharedMesh = mesh;
    }

    public void FixedUpdate()
    {
        Apply();
    }
}