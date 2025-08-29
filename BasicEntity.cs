using UnityEngine;

[DefaultExecutionOrder(100)]
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]

public class BasicEntity : SimulationObject, IEntity
{
    protected Rigidbody _rb;
    private Collider _collider;
    private Vector3 _gravity = default;
    private int _currentGravityPriority = 0;
    private bool _ignoreForceUntilNextSimulationUpdate = false; //When assigning to velocities, forces must be ignored until the next update so that the unpredictable order of force applications does not cause unintended behaviour
    
    //Getters and setters
    public bool isKinematic { get { return _rb.isKinematic; } set { _rb.isKinematic = value; } }
    public Collider collider { get { return _collider; } }
    public Vector3 linearVelocity { get { return _rb.linearVelocity; } protected set { _rb.linearVelocity = value; } }
    public Vector3 angularVelocity { get { return _rb.angularVelocity; } protected set { _rb.angularVelocity = value; } }
    public Vector3 gravity { get { return _gravity; } }

    public void TrySetGravity(Vector3 force, int priority)
    {
        if (priority > _currentGravityPriority)
        {
            _currentGravityPriority = priority;
            _gravity = force;
        }
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
                _rb.linearVelocity += force / _rb.mass * simulationController.simulationDeltaTime * simulationController.simulationDeltaTime; //Force is time-scaled because it should be applied every ForceUpdate
                break;
            case ForceMode.Acceleration:
                _rb.linearVelocity += force * simulationController.simulationDeltaTime * simulationController.simulationDeltaTime; //Acceleration is time-scaled because it should be applied every ForceUpdate
                break;
            case ForceMode.Impulse:
                _rb.linearVelocity += force / _rb.mass * simulationController.simulationDeltaTime; //Impulse is not time-scaled because it will be applied once
                break;
            case ForceMode.VelocityChange:
                _rb.linearVelocity += force * simulationController.simulationDeltaTime; //VelocityChange is not time-scaled because it will be applied once
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
                _rb.angularVelocity += torque / _rb.mass * simulationController.simulationDeltaTime * simulationController.simulationDeltaTime; //Force is time-scaled because it should be applied every ForceUpdate
                break;
            case ForceMode.Acceleration:
                _rb.angularVelocity += torque * simulationController.simulationDeltaTime * simulationController.simulationDeltaTime; //Acceleration is time-scaled because it should be applied every ForceUpdate
                break;
            case ForceMode.Impulse:
                _rb.angularVelocity += torque / _rb.mass * simulationController.simulationDeltaTime; //Impulse is not time-scaled because it will be applied once
                break;
            case ForceMode.VelocityChange:
                _rb.angularVelocity += torque * simulationController.simulationDeltaTime; //VelocityChange is not time-scaled because it will be applied once
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

    public void MovePosition(Vector3 targetPosition) => _rb.MovePosition(targetPosition);

    public void MoveRotation(Quaternion targetRotation) => _rb.MoveRotation(targetRotation);

    public void SetVelocities(Vector3 linearVelocity, Vector3 angularVelocity) //Does not apply if isKinematic
    {
        _rb.linearVelocity = linearVelocity;
        _rb.angularVelocity = angularVelocity;

        _ignoreForceUntilNextSimulationUpdate = true;
    }

    public RaycastHitInfoVerbose GetPredictedTransformedSurfaceInfo(RaycastHitInfoVerbose surfaceInfo, float deltaTime)
    {
        return surfaceInfo;
    }

    public override void PreSimulationUpdate()
    {
        if (!_ignoreForceUntilNextSimulationUpdate)
        {
            _rb.linearVelocity += gravity * simulationController.simulationDeltaTime * simulationController.simulationDeltaTime;
        }

        _ignoreForceUntilNextSimulationUpdate = false;
    }

    //Unity Methods
    public override void OnEnable()
    {
        _rb = gameObject.GetComponent<Rigidbody>();
        _collider = gameObject.GetComponent<Collider>();
        base.OnEnable();
    }
}