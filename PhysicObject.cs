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
    [SerializeField] protected float _colliderBottomToCenterDist = 1f;
    [SerializeField] protected float raycastPaddingDist = 0f;
    [SerializeField] protected List<Collider> _ignoreColliders = new List<Collider>();
    private List<GameObject> _currentCollsions = new List<GameObject>();
    private Dictionary<GravitationalField, Vector3> _fieldGravityVectors = new Dictionary<GravitationalField, Vector3>() { };
    private Vector3 _gravity = default;
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
    public List<GameObject> currentCollisions { get { return _currentCollsions; } }
    public Vector3 colliderBottomPosition { get { return transform.position - transform.up * Math.Abs(_colliderBottomToCenterDist); } }

    //encapsulation
    protected Rigidbody rb { get { return gameObject.GetComponent<Rigidbody>(); } }

    public void AddForce(Vector3 force, ForceMode forceMode = ForceMode.Force)
    {
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

    public void AddForceAtPosition(Vector3 force, Vector3 position, ForceMode forceMode = ForceMode.Force)
    {
        if (isKinematic || force == Vector3.zero)
        {
            return;
        }

        Vector3 lever = position - transform.TransformPoint(rb.centerOfMass); //TEST
        Vector3 torque = force.RemoveComponentAlongAxis(lever) * lever.magnitude;

        AddForce(force, forceMode);
        AddTorque(torque, forceMode);
    }

    public void StandAt(Vector3 footingPosition, Vector3 up)
    {
        transform.position = footingPosition + up * _colliderBottomToCenterDist;
    }

    public void StandAt(Vector3 footingPosition) => StandAt(footingPosition, transform.up);

    public void StartCollisionCheck()
    {
        _fieldGravityVectors = new Dictionary<GravitationalField, Vector3>() { };
        List<GravitationalField> fields = FindObjectsOfType<GravitationalField>().Cast<GravitationalField>().ToList();

        foreach (GravitationalField field in fields)
        {
            if (GetComponent<Collider>().bounds.Intersects(field.GetComponent<Collider>().bounds))
            {
                _currentCollsions.Add(field.gameObject);
                _fieldGravityVectors[field] = field.GetGravity(this);
            }
        }
    }

    public void CheckGround(Vector3 deltaVelocity)
    {
        Vector3 gravityDirection = gravity.normalized;
        Vector3 deltaMovement = (rb.velocity + deltaVelocity) * Time.fixedDeltaTime - transform.up * raycastPaddingDist;
        //Debug.DrawRay(colliderBottomPosition, deltaMovement);
        if (Physics.Raycast(colliderBottomPosition, deltaMovement, out RaycastHit hit, deltaMovement.magnitude, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            _groundNormal = hit.normal;
            _grounded = true;
            _groundPhysicMaterial = hit.collider.material;
            return;
        }
        _groundNormal = -1f * gravityDirection;
        _grounded = false;
        _groundPhysicMaterial = null;
    }

    public void CheckGround() => CheckGround(_forceAccumulator);

    public void ForceUpdate()
    {
        if (isKinematic)
        {
            _forceAccumulator = default;
            _torqueAccumulator = default;
            return;
        }
        
        _subjectiveVelocity = Vector3.ClampMagnitude(rb.velocity / relativeTime + _forceAccumulator, velocityCap);
        _subjectiveAngularVelocity = Vector3.ClampMagnitude(rb.angularVelocity / relativeTime + _torqueAccumulator, angularVelocityCap);

        _forceAccumulator = default;
        _torqueAccumulator = default;

        //This is the only place where forces should be applied to the rigidbody
        //GroundedCheck and other code may rely on _forceAccumulator and _torqueAccumulator being representative of ALL forces/torque being applied

        rb.velocity = _subjectiveVelocity * relativeTime;
        rb.angularVelocity = _subjectiveAngularVelocity * relativeTime;
    }

    public void Tether(Vector3 tetherPoint, float tetherMaxLength, float bounceMultiplier = 1f)
    {
        if (Vector3.Distance(transform.position, tetherPoint) < tetherMaxLength)
        {
            return;
        }

        Vector3 direction = (transform.position - tetherPoint).normalized;

        if (rb.velocity.IsComponentInDirectionPositive(direction))
        {
            AddForce(rb.velocity.ComponentAlongAxis(direction) * -(1 + Math.Abs(bounceMultiplier)));
        }

        transform.position = tetherPoint + direction * tetherMaxLength;
    }

    public void UpdateGravity()
    {
        Vector3 newGravity = default;
        List<GravitationalField> fields = _fieldGravityVectors.Keys.ToList();
        foreach (GravitationalField field in fields)
        {
            Vector3 vector = field.GetGravity(this);
            _fieldGravityVectors[field] = vector;
            newGravity += vector;
        }
        _gravity = newGravity;
    }

    public void ApplyGravity()
    {
        UpdateGravity();
        AddForce(_gravity, ForceMode.Acceleration);
    }

    public void Halt()
    {
        _subjectiveVelocity = default;
        _subjectiveAngularVelocity = default;
    }

    public void EnterCollision(Collision collision)
    {
        if (_ignoreColliders.Contains(collision.collider))
        {
            return;
        }

        _currentCollsions.Add(collision.gameObject);
    }

    public void StayCollision(Collision collision)
    {
        if (_ignoreColliders.Contains(collision.collider))
        {
            return;
        }

        GameObject obj = collision.gameObject;

        if (!_currentCollsions.Contains(obj))
        {
            _currentCollsions.Add(obj);
        }
    }

    public void ExitCollision(Collision collision)
    {
        if (_ignoreColliders.Contains(collision.collider))
        {
            return;
        }

        _currentCollsions.Remove(collision.gameObject);
    }

    public void EnterTrigger(Collider collider)
    {
        if (_ignoreColliders.Contains(collider))
        {
            return;
        }

        _currentCollsions.Add(collider.gameObject);

        GravitationalField field = collider.gameObject.GetComponent<GravitationalField>();
        if (field != null)
        {
            _fieldGravityVectors[field] = field.GetGravityOnEnter(this);
        }
    }

    public void StayTrigger(Collider collider)
    {
        if (_ignoreColliders.Contains(collider))
        {
            return;
        }

        GameObject obj = collider.gameObject;

        if (!_currentCollsions.Contains(obj))
        {
            _currentCollsions.Add(obj);
        }
    }

    public void ExitTrigger(Collider collider)
    {
        if (_ignoreColliders.Contains(collider))
        {
            return;
        }

        _currentCollsions.Remove(collider.gameObject);

        GravitationalField field = collider.gameObject.GetComponent<GravitationalField>();
        if (field != null)
        {
            if (field.applyAfterExit)
            {
                _fieldGravityVectors[field] = field.GetGravityOnExit(this);
            }
            else
            {
                _fieldGravityVectors.Remove(field);
            }
        }
    }

    void Start()
    {
        StartCollisionCheck();
    }

    void FixedUpdate()
    {
        ApplyGravity();
        CheckGround(_forceAccumulator);
        ForceUpdate();
    }

    void OnCollisionEnter(Collision collision)
    {
        EnterCollision(collision);
    }

    void OnCollisionStay(Collision collision)
    {
        StayCollision(collision);
    }

    void OnCollisionExit(Collision collision)
    {
        ExitCollision(collision);
    }

    void OnTriggerEnter(Collider other)
    {
        EnterTrigger(other);
    }

    void OnTriggerStay(Collider other)
    {
        StayTrigger(other);
    }

    void OnTriggerExit(Collider other)
    {
        ExitTrigger(other);
    }
}