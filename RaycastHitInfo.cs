using UnityEngine;

public class RaycastHitInfo
{
    private Transform _transform;
    private ArticulationBody _articulationBody;
    private Rigidbody _rigidbody;
    private Collider _collider;
    private int _triangleIndex;
    private float _distance;
    private Vector3 _normal;
    private Vector3 _point;
    private Vector3 _barycentricCoordinate;
    private TransformInfo _transformInfo;

    public Transform transform { get { return _transform; } set { _transform = value; } }
    public ArticulationBody articulationBody { get { return _articulationBody; } set { _articulationBody = value; } }
    public Rigidbody rigidbody { get { return _rigidbody; } set { _rigidbody = value; } }
    public Collider collider { get { return _collider; } set { _collider = value; } }
    public int triangleIndex { get { return _triangleIndex; } set { _triangleIndex = value; } }
    public float distance { get { return _distance; } set { _distance = value; } }
    public Vector3 normal { get { return _normal; } set { _normal = value; } }
    public Vector3 point { get { return _point; } set { _point = value; } }
    public Vector3 barycentricCoordinate { get { return _barycentricCoordinate; } set { _barycentricCoordinate = value; } }
    public TransformInfo transformInfo { get { return _transformInfo; } set { _transformInfo = value; } }

    public RaycastHitInfo(RaycastHit hit)
    {
        _transform = hit.transform;
        _articulationBody = hit.articulationBody;
        _rigidbody = hit.rigidbody;
        _collider = hit.collider;
        _triangleIndex = hit.triangleIndex;
        _distance = hit.distance;
        _normal = hit.normal;
        _point = hit.point;
        _barycentricCoordinate = hit.barycentricCoordinate;
        _transformInfo = new TransformInfo(hit.transform);
    }

    public RaycastHitInfo(RaycastHitInfo hit)
    {
        _transform = hit.transform;
        _articulationBody = hit.articulationBody;
        _rigidbody = hit.rigidbody;
        _collider = hit.collider;
        _triangleIndex = hit.triangleIndex;
        _distance = hit.distance;
        _normal = hit.normal;
        _point = hit.point;
        _barycentricCoordinate = hit.barycentricCoordinate;
        _transformInfo = new TransformInfo(hit.transform);
    }

    public RaycastHitInfo(Transform hitTransform, ArticulationBody hitArticulationBody, Rigidbody hitRigidbody, Collider hitCollider, int hitTriangleIndex, float hitDistance, Vector3 hitNormal, Vector3 hitPoint, Vector3 hitBarycentricCoordinate)
    {
        _transform = hitTransform;
        _articulationBody = hitArticulationBody;
        _rigidbody = hitRigidbody;
        _collider = hitCollider;
        _triangleIndex = hitTriangleIndex;
        _distance = hitDistance;
        _normal = hitNormal;
        _point = hitPoint;
        _barycentricCoordinate = hitBarycentricCoordinate;
        _transformInfo = new TransformInfo(hitTransform);
    }
}