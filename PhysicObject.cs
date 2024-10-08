using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]

public class PhysicObject : MonoBehaviour
{
    private const float velocityCap = 100f;
    private const float angularVelocityCap = 100f;
    public static float globalTime = 1f; //Use for slowdown and speed up effect and pausing, should not be updated in FixedUpdate()

    [SerializeField] protected bool _usePrevGravityIn0g = true; //If true and _gravityBuffer == Vector3.zero, _gravity will not be overwritten and will be applied as gravity force in ForceUpdate()
    [Space]
    [SerializeField] protected bool _simulationModeAsScript = false;

    private Rigidbody _rb;

    private float _relativeTime = 1f; //Objects can experience the passage of time differently - this may have unexpected effects on acceleration if they interact, should not be updated in FixedUpdate()
    private float _simulationDeltaTime = 0.01f;
    private Vector3 _gravityBuffer = default;
    private Vector3 _gravity = new(0f, -10f, 0f); //If _usePrevGravityIn0g and _gravityBuffer == Vector3.zero, value will be defaulted to (not recommended)
    private Vector3 _normalForce = default;

    private bool _ignoreForceUntilNextForceUpdate = false; //When assigning to velocities and _forceAccumulator, forces must be ignored until the next update so that the unpredictable order of force applications does not cause unintended behaviour
    private bool _ignoreTorqueUntilNextForceUpdate = false; //When assigning to angular velocities and _torqueAccumulator, forces must be ignored until the next update so that the unpredictable order of force applications does not cause unintended behaviour

    private Vector3 _subjectiveVelocity = default;
    private Vector3 _subjectiveAngularVelocity = default;
    private Vector3 _forceAccumulator = default;
    private Vector3 _torqueAccumulator = default;

    //getters and setters
    public Rigidbody rb { get { return _rb; } }
    protected Vector3 normalForce { get { return _normalForce; } }
    public float relativeTime { get { return _relativeTime; } }
    public virtual float experiencedSimulationDeltaTime { get { return _simulationDeltaTime * _relativeTime * globalTime; } }
    public bool isKinematic { get { return _rb.isKinematic; } set { _rb.isKinematic = value; } }
    public Vector3 gravity { get { return _gravity; } }
    public Vector3 subjectiveVelocity { get { return _subjectiveVelocity; } }
    public Vector3 subjectiveAngularVelocity { get { return _subjectiveAngularVelocity; } }

    public virtual Vector3 groundNormal { get { return _normalForce.normalized; } }

    public void SetRigidbodyPointer()
    {
        _rb = GetComponent<Rigidbody>();
    }

    public virtual void SetRelativeTime(float value)
    {
        _relativeTime = value;
    }

    public virtual void SetIsKinematic(bool value)
    {
        _rb.isKinematic = value;
    }

    public virtual void SetSimulationModeAsScript(bool value)
    {
        if (value)
        {
            SubscribeToLateFixedUpdate();
            return;
        }

        UnsubscribeFromLateFixedUpdate();
    }

    public virtual void SetSimulationDeltaTime(float value)
    {
        _simulationDeltaTime = value;
    }

    public void SubscribeToLateFixedUpdate()
    {
        LateFixedUpdateBroadcaster.LateFixedUpdate += LateFixedUpdate;
    }

    public void UnsubscribeFromLateFixedUpdate()
    {
        LateFixedUpdateBroadcaster.LateFixedUpdate -= LateFixedUpdate;
    }

    protected Vector3 GetCurrentForceAccumulator()
    {
        return _forceAccumulator;
    }

    protected Vector3 GetCurrentTorqueAccumulator()
    {
        return _torqueAccumulator;
    }

    public void AddGravityForce(Vector3 force)
    {
        _gravityBuffer += force;
        AddForce(force, ForceMode.Acceleration);
    }

    public new void AddForce(Vector3 force, ForceMode forceMode = ForceMode.Force) //Forces affect subjective velocities, not actual velocities
    {
        if (isKinematic || _ignoreForceUntilNextForceUpdate)
        {
            return;
        }

        Vector3 interpretedForce = default;

        switch (forceMode)
        {
            case ForceMode.Force:
                interpretedForce = force * experiencedSimulationDeltaTime / _rb.mass; //Force is dependent on experiencedTime because it should be applied every FixedUpdate
                break;
            case ForceMode.Acceleration:
                interpretedForce = force * experiencedSimulationDeltaTime; //Acceleration is dependent on experiencedTime because it should be applied every FixedUpdate
                break;
            case ForceMode.Impulse:
                interpretedForce = force / _rb.mass; //Impulse is not dependent on experiencedTime because it will be applied once
                break;
            case ForceMode.VelocityChange:
                interpretedForce = force; //VelocityChange is not dependent on experiencedTime because it will be applied once
                break;
        }

        _forceAccumulator += interpretedForce;
    }

    public new void AddTorque(Vector3 torque, ForceMode forceMode = ForceMode.Force) //Forces affect subjective velocities, not actual velocities
    {
        if (isKinematic || _ignoreTorqueUntilNextForceUpdate)
        {
            return;
        }

        Vector3 interpretedTorque = default;

        switch (forceMode)
        {
            case ForceMode.Force:
                interpretedTorque = torque * experiencedSimulationDeltaTime / _rb.mass; //Force is dependent on experiencedTime because it should be applied every FixedUpdate
                break;
            case ForceMode.Acceleration:
                interpretedTorque = torque * experiencedSimulationDeltaTime; //Acceleration is dependent on experiencedTime because it should be applied every FixedUpdate
                break;
            case ForceMode.Impulse:
                interpretedTorque = torque / _rb.mass; //Impulse is not dependent on experiencedTime because it will be applied once
                break;
            case ForceMode.VelocityChange:
                interpretedTorque = torque; //VelocityChange is not dependent on experiencedTime because it will be applied once
                break;
        }

        _torqueAccumulator += interpretedTorque;
    }

    public new void AddForceAtPosition(Vector3 force, Vector3 position, ForceMode forceMode = ForceMode.Force)
    {
        if (isKinematic || (_ignoreForceUntilNextForceUpdate && _ignoreTorqueUntilNextForceUpdate))
        {
            return;
        }

        Vector3 lever = position - transform.TransformPoint(_rb.centerOfMass);
        Vector3 torque = force.RemoveComponentAlongAxis(lever) * lever.magnitude;

        AddForce(force, forceMode);
        AddTorque(torque, forceMode);
    }

    public void ForceUpdate()
    {
        if (_usePrevGravityIn0g && _gravityBuffer == default)
        {
            AddForce(_gravity, ForceMode.Acceleration); //_gravity is not updated
        }
        else
        {
            _gravity = _gravityBuffer;
        }

        _gravityBuffer = default;
        _ignoreForceUntilNextForceUpdate = false;
        _ignoreTorqueUntilNextForceUpdate = false;

        if (globalTime == 0f || _relativeTime == 0f || _rb.isKinematic) //_subjectiveVelocity and _subjectiveAngularVelocity are not updated so that their values can be retrieved later
        {
            _rb.velocity = default;
            _rb.angularVelocity = default;

            _forceAccumulator = default;
            _torqueAccumulator = default;

            _normalForce = default;
            return;
        }

        SetVelocity(_subjectiveVelocity + _forceAccumulator, false);
        SetAngularVelocity(_subjectiveAngularVelocity + _torqueAccumulator, false);
    }

    public void BuiltInForceCorrection()
    {
        if (globalTime == 0f || _relativeTime == 0f || _rb.isKinematic) //_subjectiveVelocity and _subjectiveAngularVelocity are not updated so that their values can be retrieved later
        {
            return;
        }

        _subjectiveVelocity = _rb.velocity / _relativeTime / globalTime;
        _subjectiveAngularVelocity = _rb.angularVelocity / _relativeTime / globalTime;
    }

    public void SetVelocity(Vector3 newSubjectiveVelocity, bool ignoreForceUntilNextForceUpdate = true)
    {
        _subjectiveVelocity = Vector3.ClampMagnitude(newSubjectiveVelocity, velocityCap);
        _rb.velocity = _relativeTime * globalTime * _subjectiveVelocity;

        _forceAccumulator = default;

        _ignoreForceUntilNextForceUpdate = ignoreForceUntilNextForceUpdate;
    }

    public void SetAngularVelocity(Vector3 newSubjectiveAngularVelocity, bool ignoreTorqueUntilNextForceUpdate = true)
    {
        _subjectiveAngularVelocity = Vector3.ClampMagnitude(newSubjectiveAngularVelocity, angularVelocityCap);
        _rb.angularVelocity = _relativeTime * globalTime * _subjectiveAngularVelocity;

        _torqueAccumulator = default;

        _ignoreTorqueUntilNextForceUpdate = ignoreTorqueUntilNextForceUpdate;
    }

    public virtual void OnEnable()
    {
        SetRigidbodyPointer();

        if (_simulationModeAsScript)
        {
            return;
        }

        SubscribeToLateFixedUpdate();
        SetSimulationDeltaTime(Time.fixedDeltaTime);
    }

    public virtual void OnDisable()
    {
        if (_simulationModeAsScript)
        {
            return;
        }

        UnsubscribeFromLateFixedUpdate();
    }

    public virtual void FixedUpdate()
    {
        if (_simulationModeAsScript)
        {
            return;
        }

        ForceUpdate();
    }

    public virtual void LateFixedUpdate() //Runs after physics simulate
    {
        Vector3 expectedVelocity = _subjectiveVelocity;
        BuiltInForceCorrection();
        _normalForce = _subjectiveVelocity - expectedVelocity;
    }
}