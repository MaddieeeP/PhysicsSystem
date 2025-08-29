using UnityEngine;
using UnityEngine.Splines;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]

public class SplineFollow : SimulationObject, IEntity //FIX - add editor with speed and duration options, orientation options, check time scaling
{
    [SerializeField] private SplineContainer _splineContainer;
    [SerializeField] private float _tSpeed = 0.1f;
    [SerializeField] private AnimationCurve _curve;

    private Rigidbody _rb;
    private Collider _collider;
    private float _t;
    private float _bufferedT;
    private bool _movingInPositiveDirection = true;
    private Vector3 _trueVelocityBuffer = Vector3.zero;
    private Vector3 _linearVelocity = Vector3.zero;
    private Vector3 _angularVelocity = Vector3.zero;

    public Vector3 gravity { get { return Vector3.zero; } }
    public Vector3 linearVelocity { get { return _linearVelocity; } }
    public Vector3 angularVelocity { get { return _angularVelocity; } }

    public bool isKinematic { get { return _rb.isKinematic; } set { _rb.isKinematic = value; } }
    public Collider collider { get { return _collider; } }

    public void TrySetGravity(Vector3 force, int priority) { }
    public void AddForce(Vector3 force, ForceMode forceMode = ForceMode.Force) { }
    public void AddTorque(Vector3 torque, ForceMode forceMode = ForceMode.Force) { }
    public void AddForceAtPosition(Vector3 force, Vector3 position, ForceMode forceMode = ForceMode.Force) { }

    public override void PreSimulationUpdate()
    {
        if (_movingInPositiveDirection)
        {
            _t += _tSpeed * simulationController.simulationDeltaTime;
            if (_t >= 1f)
            {
                _t = 2 - _t;
                _movingInPositiveDirection = false;
            }
        }
        else
        {
            _t -= _tSpeed * simulationController.simulationDeltaTime;
            if (_t <= 0f)
            {
                _t = -_t;
                _movingInPositiveDirection = true;
            }
        }

        _splineContainer.EvaluateInWorldSpace(_curve.Evaluate(_t), out Vector3 position, out Vector3 forward, out Vector3 up);
        _trueVelocityBuffer = (position - transform.position) / simulationController.simulationDeltaTime;
        _rb.linearVelocity = _trueVelocityBuffer;
        Quaternion.Inverse(transform.rotation).ToAngleAxis(out float angle, out Vector3 axis);
        _rb.angularVelocity = angle * axis * Mathf.Deg2Rad / simulationController.simulationDeltaTime;
    }

    public override void PostSimulationUpdate()
    {
        _linearVelocity = _trueVelocityBuffer;
        _angularVelocity = _rb.angularVelocity;
        _bufferedT = _t;
        base.PostSimulationUpdate();
    }

    public RaycastHitInfoVerbose GetPredictedTransformedSurfaceInfo(RaycastHitInfoVerbose surfaceInfo, float deltaTime)
    {
        float nextT;

        if (_movingInPositiveDirection)
        {
            nextT =_bufferedT + _tSpeed * deltaTime;
            if (nextT >= 1f)
            {
                nextT = 2 - nextT;
            }
        }
        else
        {
            nextT = _bufferedT - _tSpeed * deltaTime;
            if (nextT <= 0f)
            {
                nextT = -nextT;
            }
        }
        _splineContainer.EvaluateInWorldSpace(_curve.Evaluate(nextT), out Vector3 position, out Vector3 forward, out Vector3 up);

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
