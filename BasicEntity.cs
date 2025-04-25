using UnityEngine;

[DefaultExecutionOrder(100)]
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]

public class BasicEntity : Entity
{
    [SerializeField] protected bool _usePrevGravityIn0g = true; //If true and _gravityBuffer == Vector3.zero, _gravity will not be overwritten and will be applied as gravity force in ForceUpdate()

    private Rigidbody _rb;
    private Collider _collider;
    private Vector3 _gravityBuffer = default;
    private Vector3 _gravity = new(0f, -10f, 0f); //If _usePrevGravityIn0g and _gravityBuffer == Vector3.zero, value will be defaulted to (not recommended)

    private bool _ignoreForceUntilNextSimulationUpdate = false; //When assigning to velocities, forces must be ignored until the next update so that the unpredictable order of force applications does not cause unintended behaviour

    private Vector3 _velocity = default;
    private Vector3 _angularVelocity = default;
    private Vector3 _velocityChangeAccumulator = default;
    private Vector3 _accelerationAccumulator = default;
    private Vector3 _angularVelocityChangeAccumulator = default;
    private Vector3 _angularAccelerationAccumulator = default;

    //getters and setters
    public override bool isKinematic { get { return _rb.isKinematic; } set { _rb.isKinematic = value; } }
    public override Collider collider { get { return _collider; } }
    public override Vector3 velocity { get { return _velocity; } }
    public override Vector3 angularVelocity { get { return _angularVelocity; } }
    public override Vector3 gravity { get { return _gravity; } }

    public override void AddGravityForce(Vector3 force)
    {
        _gravityBuffer += force;
        AddForce(force, ForceMode.Acceleration);
    }

    public override void AddForce(Vector3 force, ForceMode forceMode = ForceMode.Force)
    {
        if (isKinematic || _ignoreForceUntilNextSimulationUpdate)
        {
            return;
        }

        switch (forceMode)
        {
            case ForceMode.Force:
                _accelerationAccumulator += force / _rb.mass; //Force is time-scaled because it should be applied every ForceUpdate
                break;
            case ForceMode.Acceleration:
                _accelerationAccumulator += force; //Acceleration is time-scaled because it should be applied every ForceUpdate
                break;
            case ForceMode.Impulse:
                _velocityChangeAccumulator += force / _rb.mass; //Impulse is not time-scaled because it will be applied once
                break;
            case ForceMode.VelocityChange:
                _velocityChangeAccumulator += force; //VelocityChange is not time-scaled because it will be applied once
                break;
        }
    }

    public override void AddTorque(Vector3 torque, ForceMode forceMode = ForceMode.Force)
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

    public override void AddForceAtPosition(Vector3 force, Vector3 position, ForceMode forceMode = ForceMode.Force)
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

    public void SetVelocities(Vector3 velocity, Vector3 angularVelocity) //Does not apply if SimulationController.globalTimeScale * relativeTimeScale == 0f or isKinematic
    {
        _rb.velocity = velocity;
        _rb.angularVelocity = angularVelocity;

        _velocityChangeAccumulator = default;
        _accelerationAccumulator = default;
        _angularVelocityChangeAccumulator = default;
        _angularAccelerationAccumulator = default;

        _gravityBuffer = default;
        _ignoreForceUntilNextSimulationUpdate = true;
    }

    protected virtual void ModifyTrueVelocities(ref Vector3 trueVelocity, ref Vector3 trueAngularVelocity, float deltaTime) { }

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
            _rb.velocity = default;
            _rb.angularVelocity = default;
        }
        else
        {
            Vector3 trueVelocity = _rb.velocity + (_velocityChangeAccumulator + timeScale * deltaTime * _accelerationAccumulator) * timeScale;
            Vector3 trueAngularVelocity = _rb.angularVelocity + (_angularVelocityChangeAccumulator + timeScale * deltaTime * _angularAccelerationAccumulator) * timeScale;
            ModifyTrueVelocities(ref trueVelocity, ref trueAngularVelocity, deltaTime);
            _rb.velocity = trueVelocity;
            _rb.angularVelocity = trueAngularVelocity;
        }

        _velocityChangeAccumulator = default;
        _accelerationAccumulator = default;
        _angularVelocityChangeAccumulator = default;
        _angularAccelerationAccumulator = default;
    }

    public override void LateSimulationUpdate(float deltaTime) //Maintain data integrity after simulation; update relevant data about self
    {
        float timeScale = SimulationController.globalTimeScale * relativeTimeScale;

        if (timeScale == 0f || _rb.isKinematic) //_velocity and _angularVelocity are not updated
        {
            return;
        }

        _velocity = _rb.velocity / timeScale;
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