using UnityEngine;

[RequireComponent(typeof(Collider))]
public class GuidedObject : SimulationObject
{
    private Collider _collider;
    private Vector3 _velocity = Vector3.zero;
    private Vector3 _angularVelocity = Vector3.zero;

    public bool isKinematic { get; set; }
    public virtual Vector3 gravity { get { return Vector3.down; } }
    public override Vector3 velocity { get { return _velocity; } }
    public override Vector3 angularVelocity { get { return _angularVelocity; } }
    public new Collider collider { get { return _collider; } }

    private Vector3 _prevPosition;
    private Quaternion _prevRotation;

    public override void LateSimulationUpdate(float deltaTime)
    {
        float scaledDeltaTime = deltaTime * SimulationController.globalTimeScale * relativeTimeScale;
        _velocity = (transform.position - _prevPosition) / scaledDeltaTime;
        transform.rotation.DivideBy(_prevRotation).ToAngleAxis(out float angle, out Vector3 axis);
        _angularVelocity = angle * axis / scaledDeltaTime;

        _prevPosition = transform.position;
        _prevRotation = transform.rotation;
    }

    public override RaycastHitInfoVerbose GetAdjustedSurfaceInfo(RaycastHitInfoVerbose surfaceInfo)
    {
        return surfaceInfo; //FIX
    }

    //Unity Methods

    public override void OnEnable()
    {
        _collider = GetComponent<Collider>();
        _prevPosition = transform.position;
        _prevRotation = transform.rotation;

        base.OnEnable();
    }
}