using UnityEngine;

[DefaultExecutionOrder(100)]
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]

public class BasicEntity : SimulationObject, IEntity
{
    [SerializeField] protected bool _usePrevGravityIn0g = true; //If true and _gravityBuffer == Vector3.zero, _gravity will not be overwritten and will be applied as gravity force in ForceUpdate()

    private Rigidbody _rb;
    private Collider _collider;
    private Vector3 _gravityBuffer = default;
    private Vector3 _gravity = new(0f, -10f, 0f); //If _usePrevGravityIn0g and _gravityBuffer == Vector3.zero, this value will initially be defaulted to (not recommended)

    private bool _ignoreForceUntilNextSimulationUpdate = false; //When assigning to velocities, forces must be ignored until the next update so that the unpredictable order of force applications does not cause unintended behaviour

    private Vector3 _linearVelocity = default;
    private Vector3 _angularVelocity = default;
    private Vector3 _linearVelocityChangeAccumulator = default;
    private Vector3 _linearAccelerationAccumulator = default;
    private Vector3 _angularVelocityChangeAccumulator = default;
    private Vector3 _angularAccelerationAccumulator = default;

    //Getters and setters
    protected Vector3 actualLinearVelocity { get { return _rb.linearVelocity; } set { _rb.linearVelocity = value; } }
    protected Vector3 actualAngularVelocity { get { return _rb.angularVelocity; } set { _rb.angularVelocity = value; } }
    public bool isKinematic { get { return _rb.isKinematic; } set { _rb.isKinematic = value; } }
    public Collider collider { get { return _collider; } }
    public Vector3 linearVelocity { get { return _linearVelocity; } }
    public Vector3 angularVelocity { get { return _angularVelocity; } }
    public Vector3 gravity { get { return _gravity; } }
    public virtual Vector3 up { get { return -_gravity; } }

    public void AddGravityForce(Vector3 force)
    {
        _gravityBuffer += force;
        AddForce(force, ForceMode.Acceleration);
    }

    public void AddForce(Vector3 force, ForceMode forceMode = ForceMode.Force)
    {
        if (isKinematic || _ignoreForceUntilNextSimulationUpdate)
        {
            return;
        }

        switch (forceMode)
        {
            case ForceMode.Force:
                _linearAccelerationAccumulator += force / _rb.mass; //Force is time-scaled because it should be applied every ForceUpdate
                break;
            case ForceMode.Acceleration:
                _linearAccelerationAccumulator += force; //Acceleration is time-scaled because it should be applied every ForceUpdate
                break;
            case ForceMode.Impulse:
                _linearVelocityChangeAccumulator += force / _rb.mass; //Impulse is not time-scaled because it will be applied once
                break;
            case ForceMode.VelocityChange:
                _linearVelocityChangeAccumulator += force; //VelocityChange is not time-scaled because it will be applied once
                break;
        }
    }

    public void AddTorque(Vector3 torque, ForceMode forceMode = ForceMode.Force)
    {
        if (isKinematic || _ignoreForceUntilNextSimulationUpdate)
        {
            return;
        }
        switch (forceMode)
        {
            case ForceMode.Force:
                _angularAccelerationAccumulator += torque / _rb.mass; //Force is time-scaled because it should be applied every ForceUpdate
                break;
            case ForceMode.Acceleration:
                _angularAccelerationAccumulator += torque; //Acceleration is time-scaled because it should be applied every ForceUpdate
                break;
            case ForceMode.Impulse:
                _angularVelocityChangeAccumulator += torque / _rb.mass; //Impulse is not time-scaled because it will be applied once
                break;
            case ForceMode.VelocityChange:
                _angularVelocityChangeAccumulator += torque; //VelocityChange is not time-scaled because it will be applied once
                break;
        }
    }

    public void AddForceAtPosition(Vector3 force, Vector3 position, ForceMode forceMode = ForceMode.Force)
    {
        Vector3 lever = position - transform.TransformPoint(_rb.centerOfMass);
        Vector3 torque = force.RemoveComponentAlongAxis(lever) * lever.magnitude;

        AddForce(force, forceMode);
        AddTorque(torque, forceMode);
    }

    public void MovePosition(Vector3 targetPosition)
    {
        _rb.MovePosition(targetPosition);
    }

    public void MoveRotation(Quaternion targetRotation)
    {
        _rb.MoveRotation(targetRotation);
    }

    public void SetVelocities(Vector3 linearVelocity, Vector3 angularVelocity) //Does not apply if SimulationController.globalTimeScale * relativeTimeScale == 0f or isKinematic
    {
        float timeScale = SimulationController.globalTimeScale * relativeTimeScale;
        _rb.linearVelocity = linearVelocity * timeScale;
        _rb.angularVelocity = angularVelocity * timeScale;

        _linearVelocityChangeAccumulator = default;
        _linearAccelerationAccumulator = default;
        _angularVelocityChangeAccumulator = default;
        _angularAccelerationAccumulator = default;

        _gravityBuffer = default;
        _ignoreForceUntilNextSimulationUpdate = true;
    }

    public RaycastHitInfoVerbose GetPredictedTransformedSurfaceInfo(RaycastHitInfoVerbose surfaceInfo, float deltaTime)
    {
        return surfaceInfo;
    }

    public override void SimulationUpdate(float deltaTime) //Apply forces from last execution cycle; refresh buffers
    {
        if (_usePrevGravityIn0g && _gravityBuffer == default)
        {
            AddForce(_gravity, ForceMode.Acceleration); //_gravity is not updated, will not apply if _ignoreForceUntilNextSimulationUpdate
        }
        else
        {
            _gravity = _gravityBuffer;
        }

        _gravityBuffer = default;
        _ignoreForceUntilNextSimulationUpdate = false;

        float timeScale = SimulationController.globalTimeScale * relativeTimeScale;

        if (timeScale == 0f || _rb.isKinematic)
        {
            _rb.linearVelocity = default;
            _rb.angularVelocity = default;
        }
        else
        {
            Vector3 finalVelocity = _rb.linearVelocity + (_linearVelocityChangeAccumulator + timeScale * deltaTime * _linearAccelerationAccumulator) * timeScale;
            Vector3 finalAngularVelocity = _rb.angularVelocity + (_angularVelocityChangeAccumulator + timeScale * deltaTime * _angularAccelerationAccumulator) * timeScale;
            _rb.linearVelocity = finalVelocity;
            _rb.angularVelocity = finalAngularVelocity;
        }

        _linearVelocityChangeAccumulator = default;
        _linearAccelerationAccumulator = default;
        _angularVelocityChangeAccumulator = default;
        _angularAccelerationAccumulator = default;
    }

    public override void LateSimulationUpdate(float deltaTime) //Maintain data integrity after simulation; update relevant data about self
    {
        float timeScale = SimulationController.globalTimeScale * relativeTimeScale;

        if (timeScale == 0f || _rb.isKinematic) //_linearVelocity and _angularVelocity are not updated
        {
            return;
        }

        _linearVelocity = _rb.linearVelocity / timeScale;
        _angularVelocity = _rb.angularVelocity / timeScale;
    }

    //Unity Methods
    public override void OnEnable()
    {
        _rb = gameObject.GetComponent<Rigidbody>();
        _collider = gameObject.GetComponent<Collider>();
        base.OnEnable();
    }
}