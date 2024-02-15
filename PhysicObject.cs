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
    private const float velocityCap = 200f;
    private const float angularVelocityCap = 200f;

    [SerializeField] protected float _relativeTime = 1f;
    [SerializeField] protected float _footToCenterDist = 1.2f;
    [SerializeField] protected bool _usePrevGravityIn0g = true;
    [SerializeField] protected Vector3 _gravity = new Vector3(0f, 9.81f, 0f);
    private Vector3 _gravityBuffer = default;
    protected bool _grounded = false;
    protected Vector3 _groundNormal = default;
    protected PhysicMaterial _groundPhysicMaterial;

    private Vector3 _subjectiveVelocity = default;
    private Vector3 _subjectiveAngularVelocity = default;
    private Vector3 _forceAccumulator = default;
    private Vector3 _torqueAccumulator = default;

    //getters and setters
    public float relativeTime { get { return _relativeTime; } set { _relativeTime = value; } }
    public bool isKinematic { get { return rb.isKinematic; } set { rb.isKinematic = value; } }
    public bool grounded { get { return _grounded; } }
    public Vector3 gravity { get { return _gravity; } }
    public Vector3 groundNormal { get { return _groundNormal; } }
    protected Vector3 subjectiveVelocity { get { return rb.velocity / relativeTime; } }
    protected Vector3 subjectiveAngularVelocity { get { return rb.angularVelocity / relativeTime; } }

    //encapsulation
    private Rigidbody rb { get { return gameObject.GetComponent<Rigidbody>(); } }

    public void AddForce(Vector3 force, ForceMode forceMode = ForceMode.Force, bool isGravityForce = false)
    {
        if (isGravityForce)
        {
            _gravityBuffer += force;
        }

        if (isKinematic || force == Vector3.zero)
        {
            return;
        }

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
        if (isKinematic || torque == Vector3.zero)
        {
            return;
        }

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

        Vector3 lever = position - transform.TransformPoint(rb.centerOfMass); //TEST
        Vector3 torque = force.RemoveComponentAlongAxis(lever) * lever.magnitude;

        AddForce(force, forceMode, isGravityForce);
        AddTorque(torque, forceMode);
    }

    public void StandAt(Vector3 footingPosition, Vector3 up)
    {
        transform.position = footingPosition + up * _footToCenterDist;
    }

    public void StandAt(Vector3 footingPosition) => StandAt(footingPosition, transform.up);

    public void CheckGround(Vector3 deltaVelocity)
    {
        Vector3 deltaMovement = (subjectiveVelocity + deltaVelocity) * Time.fixedDeltaTime - transform.up * _footToCenterDist;
        if (Physics.Raycast(transform.position, deltaMovement, out RaycastHit hit, deltaMovement.magnitude, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            //Debug.DrawRay(transform.position, deltaMovement, Color.red, 0f, false);
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
        if (_usePrevGravityIn0g && _gravityBuffer == default)
        {
            AddForce(_gravity, ForceMode.Acceleration, true);
        } else
        {
            _gravity = _gravityBuffer;
        }

        _gravityBuffer = default;

        if (isKinematic)
        {
            _forceAccumulator = default;
            _torqueAccumulator = default;
            return;
        }
        
        _subjectiveVelocity = Vector3.ClampMagnitude((rb.velocity + _forceAccumulator) / relativeTime, velocityCap);
        _subjectiveAngularVelocity = Vector3.ClampMagnitude((rb.angularVelocity + _torqueAccumulator) / relativeTime, angularVelocityCap);
        
        _forceAccumulator = default;
        _torqueAccumulator = default;

        //This is the only place where forces should actually be applied to the rigidbody (except Halt())
        //GroundedCheck and other code may rely on _forceAccumulator and _torqueAccumulator being representative of all, or all except certain forces/torque being applied

        rb.velocity = _subjectiveVelocity * relativeTime;
        rb.angularVelocity = _subjectiveAngularVelocity * relativeTime;
    }

    public void Tether(Vector3 tetherPoint, float tetherMaxLength, float bounceMultiplier = 0f)
    {
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