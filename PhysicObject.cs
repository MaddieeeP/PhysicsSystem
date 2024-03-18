using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Linq;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]
[RequireComponent(typeof(MeshFilter))]

public class PhysicObject : MonoBehaviour
{
    private const float velocityCap = 100f;
    private const float angularVelocityCap = 100f;
    public static float globalTime = 1f; //Use for slowdown and speed up effect

    [SerializeField] private float _relativeTime = 1f; //Objects can experience the passage of time differently - this may have unexpected effects on acceleration if they interact
    [SerializeField] protected bool receiveForce = true; //If false, the object will continue to move and rotate but forces will not affect its motion (gravity will still be updated for orientation and collisions will still be processed as per usual)
    [SerializeField] protected bool receiveGravityForceWhenPaused = false;
    [SerializeField] protected bool receiveNonGravityForceWhenPaused = false;
    [SerializeField] protected float _footToCenterDist = 1.2f; //Value should be slightly above the distance from the bottom of the collider and the transform position
    [SerializeField] protected bool _usePrevGravityIn0g = true; //If true and _gravityBuffer == Vector3.zero, _gravity will not be overwritten and will be applied as gravity force in ForceUpdate()
    [SerializeField] protected Vector3 _gravity = new Vector3(0f, 9.81f, 0f); //If _usePrevGravityIn0g and _gravityBuffer == Vector3.zero, value will be defaulted to (not recommended)
    private Vector3 _gravityBuffer = default;
    private bool _paused = false;
    private bool _grounded = false;
    private bool _ignoreForceUntilNextForceUpdate = false; //When assigning to velocities and _force/_torqueAccumulator, forces must be ignored until the next update so that the unpredictable order of force applications does not cause unintended behaviour
    private List<Vector3> _currentTetherPoints = new List<Vector3>();
    private List<Vector3> _currentTetherOffsets = new List<Vector3>();
    private List<float> _currentTetherMaxLengths = new List<float>();
    private List<float> _currentTetherBounceMultipliers = new List<float>();
    protected Vector3 _groundNormal = default;
    protected PhysicMaterial _groundPhysicMaterial;

    private Vector3 _subjectiveVelocity = default;
    private Vector3 _subjectiveAngularVelocity = default;
    private Vector3 _forceAccumulator = default;
    private Vector3 _torqueAccumulator = default;
    private float _prevGlobalTime = 1f; //Used to track globalTime in the last simulation step
    private float _prevRelativeTime = 1f; //Used to track _relativeTime in the last simulation step
    private bool _prevPaused = false; //Used to track _paused in the last FixedUpdate()

    //getters and setters
    public float relativeTime { get { return _relativeTime; } set { _relativeTime = value; } }
    public float experiencedTime { get { return globalTime * _relativeTime; } }
    public float experiencedDeltaTime { get { return Time.deltaTime * _relativeTime * globalTime; } }
    public float experiencedFixedDeltaTime { get { return Time.fixedDeltaTime * _relativeTime * globalTime; } }
    public bool paused { get { return _paused; } set { _paused = value; } }
    public bool isKinematic { get { return gameObject.GetComponent<Rigidbody>().isKinematic; } set { gameObject.GetComponent<Rigidbody>().isKinematic = value; } }
    public bool grounded { get { return _grounded; } set { _grounded = value; } }
    public Vector3 gravity { get { return _gravity; } }
    public Vector3 groundNormal { get { return _groundNormal; } }
    public Vector3 subjectiveVelocity { get { return _subjectiveVelocity; } } //Although accessing RigidBody.velocity is usually preferred, it lags behind _subjectiveVelocity which may make it unreliable in calculations, but it will not update when the object is frozen
    public Vector3 subjectiveAngularVelocity { get { return _subjectiveAngularVelocity; } } //Although accessing RigidBody.angularVelocity is usually preferred, it lags behind _subjectiveAngularVelocity which may make it unreliable in calculations, but it will not update when the object is frozen

    public void AddForce(Vector3 force, ForceMode forceMode = ForceMode.Force, bool isGravityForce = false)
    {
        if (isGravityForce)
        {
            _gravityBuffer += force;
            if (_paused && !receiveGravityForceWhenPaused)
            {
                return;
            }
        }
        
        if (isKinematic || !receiveForce || _ignoreForceUntilNextForceUpdate || (_paused && !receiveNonGravityForceWhenPaused))
        {
            return;
        }

        Rigidbody rb = gameObject.GetComponent<Rigidbody>();
        Vector3 interpretedForce = default;

        switch (forceMode)
        {
            case ForceMode.Force:
                interpretedForce = force * experiencedFixedDeltaTime / rb.mass;
                break;
            case ForceMode.Acceleration:
                interpretedForce = force * experiencedFixedDeltaTime;
                break;
            case ForceMode.Impulse:
                interpretedForce = force / rb.mass;
                break;
            case ForceMode.VelocityChange:
                interpretedForce = force;
                break;
        }

        _forceAccumulator += interpretedForce;
    }

    public void AddTorque(Vector3 torque, ForceMode forceMode = ForceMode.Force)
    {
        if (isKinematic || !receiveForce || _ignoreForceUntilNextForceUpdate || (_paused && !receiveNonGravityForceWhenPaused))
        {
            return;
        }

        Rigidbody rb = gameObject.GetComponent<Rigidbody>();
        Vector3 interpretedTorque = default;

        switch (forceMode)
        {
            case ForceMode.Force:
                interpretedTorque = torque * experiencedFixedDeltaTime / rb.mass;
                break;
            case ForceMode.Acceleration:
                interpretedTorque = torque * experiencedFixedDeltaTime;
                break;
            case ForceMode.Impulse:
                interpretedTorque = torque / rb.mass;
                break;
            case ForceMode.VelocityChange:
                interpretedTorque = torque;
                break;
        }

        _torqueAccumulator += interpretedTorque;
    }

    public void AddForceAtPosition(Vector3 force, Vector3 position, ForceMode forceMode = ForceMode.Force)
    {
        if (isKinematic || !receiveForce || _ignoreForceUntilNextForceUpdate || (_paused && !receiveNonGravityForceWhenPaused))
        {
            return;
        }

        Vector3 lever = position - transform.TransformPoint(gameObject.GetComponent<Rigidbody>().centerOfMass);
        Vector3 torque = force.RemoveComponentAlongAxis(lever) * lever.magnitude;

        AddForce(force, forceMode);
        AddTorque(torque, forceMode);
    }

    public void StandAt(Vector3 footingPosition, Vector3 up)
    {
        transform.position = footingPosition + up * _footToCenterDist;
    }

    public void StandAt(Vector3 footingPosition) => StandAt(footingPosition, transform.up);

    private void CheckGround(Vector3 deltaVelocity)
    {
        if (_paused)
        {
            return;
        }

        Rigidbody rb = gameObject.GetComponent<Rigidbody>();

        Vector3 deltaMovement = (rb.velocity + deltaVelocity) * Time.fixedDeltaTime - transform.up * _footToCenterDist;
        if (deltaMovement.magnitude > 0f)
        {
            if (Physics.Raycast(transform.position, deltaMovement.normalized, out RaycastHit hit, deltaMovement.magnitude, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
            {
                _groundNormal = hit.normal;
                _grounded = true;
                _groundPhysicMaterial = hit.collider.material;
                return;
            }
        }
        _groundNormal = -gravity.normalized;
        _grounded = false;
        _groundPhysicMaterial = null;
    }

    public void CheckGround() => CheckGround(_forceAccumulator / relativeTime);

    public void ForceUpdate()
    {
        Rigidbody rb = gameObject.GetComponent<Rigidbody>();

        if (_usePrevGravityIn0g && _gravityBuffer == default)
        {
            AddForce(_gravity, ForceMode.Acceleration, true); //_gravity is not updated
        }
        else
        {
            _gravity = _gravityBuffer;
        }

        _gravityBuffer = default;
        _ignoreForceUntilNextForceUpdate = false;

        for (int i = 0; i < _currentTetherPoints.Count; i++) //tether forces are applied after all other forces, and are not ignored when _ignoreForceUntilNextUpdate was set to true
        {
            ApplyTether(_currentTetherPoints[i], _currentTetherOffsets[i], _currentTetherMaxLengths[i], _currentTetherBounceMultipliers[i]);
        }

        _currentTetherPoints = new List<Vector3>();
        _currentTetherOffsets = new List<Vector3>();
        _currentTetherMaxLengths = new List<float>();
        _currentTetherBounceMultipliers = new List<float>();

        if (isKinematic || _paused || experiencedTime == 0f)
        {
            rb.velocity = default;
            rb.angularVelocity = default;

            _forceAccumulator = default;
            _torqueAccumulator = default;
            _prevPaused = _paused;
            _prevGlobalTime = globalTime;
            _prevRelativeTime = _relativeTime;
            return;
        }

        if (globalTime != _prevGlobalTime || _relativeTime != _prevRelativeTime || _paused != _prevPaused)
        {
            rb.velocity = _subjectiveVelocity * _prevRelativeTime * _prevGlobalTime;
            rb.angularVelocity = _subjectiveAngularVelocity * _prevRelativeTime * _prevGlobalTime;
        }

        if (_prevRelativeTime * _prevGlobalTime != 0f)
        {
            //_subjectiveVelocity and _subjectiveAngularVelocity are reset to account for collisions as there is no normal contact force
            _subjectiveVelocity = Vector3.ClampMagnitude(rb.velocity / _prevRelativeTime / _prevGlobalTime + _forceAccumulator, velocityCap);
            _subjectiveAngularVelocity = Vector3.ClampMagnitude(rb.angularVelocity / _prevRelativeTime / _prevGlobalTime + _torqueAccumulator, angularVelocityCap);
        }

        //GroundedCheck and other code may rely on _forceAccumulator and _torqueAccumulator being representative of all, or all except certain forces/torque being applied
        //This is the only place where forces should be applied to ensure they are accurate, unless _ignoreForceUntilNextForceUpdate
        rb.velocity = _subjectiveVelocity * experiencedTime;
        rb.angularVelocity = _subjectiveAngularVelocity * experiencedTime;

        _forceAccumulator = default;
        _torqueAccumulator = default;
        _prevPaused = _paused;
        _prevGlobalTime = globalTime;
        _prevRelativeTime = _relativeTime;
    }

    public void Tether(Vector3 tetherPoint, float tetherMaxLength, float bounceMultiplier = 0f, Vector3 tetherOffset = default)
    {
        _currentTetherPoints.Add(tetherPoint);
        _currentTetherOffsets.Add(tetherOffset);
        _currentTetherMaxLengths.Add(tetherMaxLength);
        _currentTetherBounceMultipliers.Add(bounceMultiplier);
    }

    public void ApplyTether(Vector3 tetherPoint, Vector3 tetherOffset, float tetherMaxLength, float bounceMultiplier = 0f)
    {
        Rigidbody rb = gameObject.GetComponent<Rigidbody>();

        Vector3 tetheredPosition = transform.position + transform.rotation * tetherOffset;

        if (Vector3.Distance(tetheredPosition, tetherPoint) < tetherMaxLength)
        {
            return;
        }

        Vector3 direction = (tetheredPosition - tetherPoint).normalized;

        if (rb.velocity.IsComponentInDirectionPositive(direction))
        {
            AddForceAtPosition(rb.velocity.ComponentAlongAxis(direction) * -(1 + Math.Abs(bounceMultiplier)), tetheredPosition, ForceMode.VelocityChange);
        }

        transform.position = tetherPoint + direction * tetherMaxLength - transform.rotation * tetherOffset;
    }

    public void Halt() => SetVelocity(default, default);

    public void SetVelocity(Vector3 newSubjectiveVelocity, Vector3 newSubjectiveAngularVelocity = default)
    {
        Rigidbody rb = gameObject.GetComponent<Rigidbody>();

        _subjectiveVelocity = newSubjectiveVelocity;
        _subjectiveAngularVelocity = newSubjectiveAngularVelocity;
        rb.velocity = _subjectiveVelocity * experiencedTime;
        rb.angularVelocity = _subjectiveAngularVelocity * experiencedTime;

        _ignoreForceUntilNextForceUpdate = true;
    }

    protected virtual void FixedUpdate()
    {
        CheckGround();
        ForceUpdate();
    }
}