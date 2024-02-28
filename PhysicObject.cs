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
    public static float globalTime = 1f;

    [SerializeField] protected float _relativeTime = 1f;
    [SerializeField] protected bool receiveGravityForceWhenPaused = false;
    [SerializeField] protected bool receiveNonGravityForceWhenPaused = false;
    [SerializeField] protected float _footToCenterDist = 1.2f;
    [SerializeField] protected bool _usePrevGravityIn0g = true;
    [SerializeField] protected Vector3 _gravity = new Vector3(0f, 9.81f, 0f);
    private Vector3 _gravityBuffer = default;
    private bool _paused = false;
    private bool _grounded = false;
    protected Vector3 _groundNormal = default;
    protected PhysicMaterial _groundPhysicMaterial;

    private Vector3 _subjectiveVelocity = default;
    private Vector3 _subjectiveAngularVelocity = default;
    private Vector3 _forceAccumulator = default;
    private Vector3 _torqueAccumulator = default;
    private float prevGlobalTime = 1f;

    //getters and setters
    public float relativeTime { get { return _relativeTime; } set { SetRelativeTime(value); } }
    public bool paused { get { return _paused; } set { SetPaused(value); } }
    public bool isKinematic { get { return gameObject.GetComponent<Rigidbody>().isKinematic; } set { gameObject.GetComponent<Rigidbody>().isKinematic = value; } }
    public bool grounded { get { return _grounded; } set { SetGrounded(value); } }
    public Vector3 gravity { get { return _gravity; } }
    public Vector3 groundNormal { get { return _groundNormal; } }
    protected Vector3 subjectiveVelocity { get { return gameObject.GetComponent<Rigidbody>().velocity / relativeTime; } }
    protected Vector3 subjectiveAngularVelocity { get { return gameObject.GetComponent<Rigidbody>().angularVelocity / relativeTime; } }

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
        
        if (isKinematic || (_paused && !receiveNonGravityForceWhenPaused))
        {
            return;
        }

        Rigidbody rb = gameObject.GetComponent<Rigidbody>();
        Vector3 interpretedForce = default;

        switch (forceMode)
        {
            case ForceMode.Force:
                interpretedForce = force * Time.fixedDeltaTime / rb.mass;
                break;
            case ForceMode.Acceleration:
                interpretedForce = force * Time.fixedDeltaTime;
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
        if (isKinematic || (_paused && !receiveNonGravityForceWhenPaused))
        {
            return;
        }

        Rigidbody rb = gameObject.GetComponent<Rigidbody>();
        Vector3 interpretedTorque = default;

        switch (forceMode)
        {
            case ForceMode.Force:
                interpretedTorque = torque * Time.fixedDeltaTime / rb.mass;
                break;
            case ForceMode.Acceleration:
                interpretedTorque = torque * Time.fixedDeltaTime;
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

    public void AddForceAtPosition(Vector3 force, Vector3 position, ForceMode forceMode = ForceMode.Force, bool isGravityForce = false)
    {
        if (isKinematic || force == Vector3.zero)
        {
            return;
        }

        Vector3 lever = position - transform.TransformPoint(gameObject.GetComponent<Rigidbody>().centerOfMass);
        Vector3 torque = force.RemoveComponentAlongAxis(lever) * lever.magnitude;

        AddForce(force, forceMode, isGravityForce);
        AddTorque(torque, forceMode);
    }

    public void StandAt(Vector3 footingPosition, Vector3 up)
    {
        transform.position = footingPosition + up * _footToCenterDist;
    }

    public void StandAt(Vector3 footingPosition) => StandAt(footingPosition, transform.up);

    public void SetGrounded(bool value)
    {
        _grounded = value;
    }
 
    public void SetPaused(bool value)
    {
        if (_paused && value == false)
        {
            RefreshVelocities();
        }
        _paused = value;
    }

    public void SetRelativeTime(float value)
    {
        if (_relativeTime == 0f)
        {
            RefreshVelocities();
        }
        _relativeTime = value;
    }

    public void RefreshVelocities()
    {
        Rigidbody rb = gameObject.GetComponent<Rigidbody>();

        rb.velocity = _subjectiveVelocity * _relativeTime * globalTime;
        rb.angularVelocity = _subjectiveAngularVelocity * _relativeTime * globalTime;
    }

    public void CheckGround(Vector3 deltaVelocity)
    {
        Vector3 deltaMovement = (subjectiveVelocity + deltaVelocity) * Time.fixedDeltaTime - transform.up * _footToCenterDist;
        if (Physics.Raycast(transform.position, deltaMovement, out RaycastHit hit, deltaMovement.magnitude, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            _groundNormal = hit.normal;
            _grounded = true;
            _groundPhysicMaterial = hit.collider.material;
            return;
        }
        _groundNormal = -gravity.normalized;
        _grounded = false;
        _groundPhysicMaterial = null;
    }

    public void CheckGround() => CheckGround(_forceAccumulator / relativeTime);

    public void ForceUpdate()
    {
        Rigidbody rb = gameObject.GetComponent<Rigidbody>();

        if (_paused) //_forceAccumulator and _torqueAccumulator are not reset so they will stack over time, velocity and angularVelocity will return after unpausing
        {
            rb.velocity = default;
            rb.angularVelocity = default;
            return;
        }

        if (_usePrevGravityIn0g && _gravityBuffer == default)
        {
            AddForce(_gravity, ForceMode.Acceleration, true); //_gravity is not updated
        } else
        {
            _gravity = _gravityBuffer;
        }

        _gravityBuffer = default; //_gravityBuffer is added to by ForceField objects between FixedUpdate calls

        if (isKinematic) //force and torque are not applied
        {
            _forceAccumulator = default;
            _torqueAccumulator = default;
            return;
        }
        
        if (globalTime * _relativeTime == 0f) //force and torque are not applied, but _subjectiveVelocity and _subjectiveAngularVelocity are not reset
        {
            _forceAccumulator = default;
            _torqueAccumulator = default;
            rb.velocity = default;
            rb.angularVelocity = default;
            return;
        }

        if (prevGlobalTime == 0f && globalTime != prevGlobalTime) //rigidbody velocity and angularVelocity must be recalculated before _subjectiveVelocity and _subjectiveAngularVelocity
        {
            RefreshVelocities();
        }

        //_subjectiveVelocity and _subjectiveAngularVelocity are reset to account for collisions as there is no normal contact force
        _subjectiveVelocity = Vector3.ClampMagnitude((rb.velocity + _forceAccumulator) / relativeTime, velocityCap);
        _subjectiveAngularVelocity = Vector3.ClampMagnitude((rb.angularVelocity + _torqueAccumulator) / relativeTime, angularVelocityCap);
        
        _forceAccumulator = default;
        _torqueAccumulator = default;

        //This is the only place where forces should actually be applied to the rigidbody (except Halt())
        //GroundedCheck and other code may rely on _forceAccumulator and _torqueAccumulator being representative of all, or all except certain forces/torque being applied

        rb.velocity = _subjectiveVelocity * _relativeTime * globalTime;
        rb.angularVelocity = _subjectiveAngularVelocity * _relativeTime * globalTime;
    }

    public void Tether(Vector3 tetherPoint, float tetherMaxLength, float bounceMultiplier = 0f)
    {
        Rigidbody rb = gameObject.GetComponent<Rigidbody>();

        if (Vector3.Distance(transform.position, tetherPoint) < tetherMaxLength)
        {
            return;
        }

        Vector3 direction = (transform.position - tetherPoint).normalized;

        if (rb.velocity.IsComponentInDirectionPositive(direction))
        {
            AddForce(rb.velocity.ComponentAlongAxis(direction) * -(1 + Math.Abs(bounceMultiplier)), ForceMode.VelocityChange);
        }

        transform.position = tetherPoint + direction * tetherMaxLength;
    }

    public void Halt()
    {
        Rigidbody rb = gameObject.GetComponent<Rigidbody>();

        _subjectiveVelocity = default;
        _subjectiveAngularVelocity = default;
        rb.velocity = default;
        rb.angularVelocity = default;
    }

    void FixedUpdate()
    {
        CheckGround();
        ForceUpdate();
    }
}