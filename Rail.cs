using UnityEngine;
using UnityEngine.Splines;

[RequireComponent(typeof(SplineContainer))]
public class Rail : SimulationObject
{
    [SerializeField] private int _resolution;
    [SerializeField] private int _iterations;
    [SerializeField] private PhysicsMaterial _sharedMaterial;

    private SplineContainer _splineContainer;
    private float _splineLength;
    private Bounds _bounds;
    
    public PhysicsMaterial sharedMaterial { get { return _sharedMaterial; } }
    public float splineLength { get { return _splineLength; } }
    public Bounds bounds { get { return _bounds; } }

    public override void OnEnable()
    {
        _splineContainer = GetComponent<SplineContainer>();
        _splineLength = _splineContainer.Spline.GetLength();
        _bounds = _splineContainer.Spline.GetBounds();

        base.OnEnable();
    }

    public void EvaluateInWorldSpace(float t, out Vector3 position, out Vector3 forward, out Vector3 up)
    {
        _splineContainer.EvaluateInWorldSpace(t, out position, out forward, out up);
    }

    public Vector3 GetNearestPointInWorldSpace(Vector3 position, out float t, out float distance)
    {
        return _splineContainer.GetNearestPointInWorldSpace(position, out t, out distance, _resolution, _iterations);
    }
}
