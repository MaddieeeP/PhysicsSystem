using UnityEngine;
using UnityEngine.Splines;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]

public class SplineFollow : Entity //FIX - add editor with speed and duration options, orientation options
{
    [SerializeField] private SplineContainer _splineContainer;
    [SerializeField] private float _tSpeed = 0.1f;
    [SerializeField] private AnimationCurve _curve;

    private Rigidbody _rb;
    private Collider _collider;
    private float _t;
    private float _bufferedT;
    private bool _direction = true;
    private Vector3 _trueVelocityBuffer = Vector3.zero;
    private Vector3 _velocity = Vector3.zero;
    private Vector3 _angularVelocity = Vector3.zero;

    public override Vector3 gravity { get { return Vector3.zero; } }
    public override Vector3 velocity { get { return _velocity; } }
    public override Vector3 angularVelocity { get { return _angularVelocity; } }

    public override bool isKinematic { get { return _rb.isKinematic; } set { _rb.isKinematic = value; } }
    public override Collider collider { get { return _collider; } }
    public override Vector3 up { get { return transform.up; } }

    public override void AddGravityForce(Vector3 force) { }
    public override void AddForce(Vector3 force, ForceMode forceMode = ForceMode.Force) { }
    public override void AddTorque(Vector3 torque, ForceMode forceMode = ForceMode.Force) { }
    public override void AddForceAtPosition(Vector3 force, Vector3 position, ForceMode forceMode = ForceMode.Force) { }

    public override void SimulationUpdate(float deltaTime)
    {
        float scaledDeltaTime = deltaTime * SimulationController.globalTimeScale * relativeTimeScale;

        if (_direction)
        {
            _t += _tSpeed * scaledDeltaTime;
            if (_t >= 1f)
            {
                _t = 2 - _t;
                _direction = false;
            }
        }
        else
        {
            _t -= _tSpeed * scaledDeltaTime;
            if (_t <= 0f)
            {
                _t = -_t;
                _direction = true;
            }
        }

        _splineContainer.Spline.Evaluate(_curve.Evaluate(_t), out Vector3 position, out Vector3 forward, out Vector3 up);
        _trueVelocityBuffer = (position - transform.position) / deltaTime;
        _rb.velocity = _trueVelocityBuffer;
        Quaternion.Inverse(transform.rotation).ToAngleAxis(out float angle, out Vector3 axis);
        _rb.angularVelocity = angle * axis * Mathf.Deg2Rad / deltaTime;
    }

    public override void LateSimulationUpdate(float deltaTime)
    {
        float timeScale = SimulationController.globalTimeScale * relativeTimeScale;
        _velocity = _trueVelocityBuffer;// / timeScale;
        _angularVelocity = _rb.angularVelocity / timeScale;
        _bufferedT = _t;
        base.LateSimulationUpdate(deltaTime);
    }

    public override RaycastHitInfoVerbose GetPredictedTransformedSurfaceInfo(RaycastHitInfoVerbose surfaceInfo, float deltaTime)
    {
        float nextT;
        float scaledDeltaTime = deltaTime * SimulationController.globalTimeScale * relativeTimeScale;

        if (_direction)
        {
            nextT =_bufferedT + _tSpeed * scaledDeltaTime;
            if (nextT >= 1f)
            {
                nextT = 2 - nextT;
            }
        }
        else
        {
            nextT = _bufferedT - _tSpeed * scaledDeltaTime;
            if (nextT <= 0f)
            {
                nextT = -nextT;
            }
        }
        _splineContainer.Spline.Evaluate(_curve.Evaluate(nextT), out Vector3 position, out Vector3 forward, out Vector3 up);

        RaycastHitInfoVerbose newSurfaceInfo = new RaycastHitInfoVerbose(surfaceInfo);
        //newSurfaceInfo.normal = transform.rotation * Quaternion.Inverse(_prevRotation) * surfaceInfo.normal;
        newSurfaceInfo.point += position - transform.position;
        return newSurfaceInfo;
    }
     
    //Unity Methods
    public override void OnEnable()
    {
        _t = 0;
        _bufferedT = 0;
        _rb = gameObject.GetComponent<Rigidbody>();
        _collider = gameObject.GetComponent<Collider>();
        base.OnEnable();
    }
}
