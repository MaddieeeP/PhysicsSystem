using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Linq;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]
[RequireComponent(typeof(MeshFilter))]

public class PhysicsObject : MonoBehaviour
{
    private const float velocityCap = 200f;
    private const float angularVelocityCap = 200f;

    [SerializeField] protected float _relativeTime = 1f;
    [SerializeField] protected float _colliderBottomToCenterDist = 1f;
    [SerializeField] protected float _hoverHeight = 0.5f;
    [SerializeField] protected float _hoverStrength = 50f;
    [SerializeField] protected float _hoverDamp = 20f;
    [SerializeField] protected float _hoverSnap = 0.1f;
    [SerializeField] protected float _orientStrength = 100f;
    [SerializeField] protected float _orientDamp = 20f;
    [SerializeField] protected float _moveMaxSpeed = 5f;
    [SerializeField] protected float _moveMaxAcceleration = 0.5f;
    [SerializeField] protected float _moveMaxDeceleration = 0.5f;
    [SerializeField] protected List<Collider> _ignoreColliders = new List<Collider>();
    private List<GameObject> _currentCollsions = new List<GameObject>();
    private Dictionary<GravitationalField, Vector3> _fieldGravityVectors = new Dictionary<GravitationalField, Vector3>() { };
    private Vector3 _gravity = default;
    protected bool _grounded = false;
    private Vector3 _groundNormal = default;

    private Vector3 _forceAccumulator = default;
    private Vector3 _torqueAccumulator = default;

    //getters and setters
    public float relativeTime { get { return _relativeTime; } set { _relativeTime = value; } }
    public bool isKinematic { get { return rb.isKinematic; } set { rb.isKinematic = value; } }
    public bool grounded { get { return _grounded; } }
    public float moveMaxSpeed { get { return _moveMaxSpeed; } }
    public float moveMaxAcceleration { get { return Math.Abs(_moveMaxAcceleration); } }
    public float moveMaxDeceleration { get { return Math.Abs(_moveMaxDeceleration); } }
    public Vector3 gravity { get { return _gravity; } }
    public Vector3 groundNormal { get { return _groundNormal; } }
    public List<GameObject> currentCollisions { get { return _currentCollsions; } }
    public Vector3 colliderBottomPosition { get { return transform.position - transform.up * Math.Abs(_colliderBottomToCenterDist); } }

    //encapsulation
    protected Rigidbody rb { get { return gameObject.GetComponent<Rigidbody>(); } }

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

    public void OnCollisionEnter(Collision collision)
    {
        EnterCollision(collision);
    }

    public void OnCollisionStay(Collision collision)
    {
        StayCollision(collision);
    }

    public void OnCollisionExit(Collision collision)
    {
        ExitCollision(collision);
    }

    public void OnTriggerEnter(Collider other)
    {
        EnterTrigger(other);
    }

    public void OnTriggerStay(Collider other)
    {
        StayTrigger(other);
    }

    public void OnTriggerExit(Collider other)
    {
        ExitTrigger(other);
    }

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
                interpretedForce = force * Time.fixedDeltaTime * relativeTime / rb.mass;
                break;
            case ForceMode.Acceleration:
                interpretedForce = force * Time.fixedDeltaTime * relativeTime;
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
                interpretedTorque = torque * Time.fixedDeltaTime * relativeTime / rb.mass;
                break;
            case ForceMode.Acceleration:
                interpretedTorque = torque * Time.fixedDeltaTime * relativeTime;
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
        Vector3 torque = force.RemoveComponentInDirection(lever) * lever.magnitude;

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
        Vector3 deltaMovement = ((rb.velocity + deltaVelocity) * Time.fixedDeltaTime) + (transform.rotation * Vector3.up * -_hoverHeight);
        if (Physics.Raycast(colliderBottomPosition, deltaMovement.normalized, out RaycastHit hit, deltaMovement.magnitude, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            _groundNormal = hit.normal;
            _grounded = true;
            return;
        }
        _groundNormal = -1f * gravityDirection;
        _grounded = false;
    }

    public void HoverWithForce() //FIX - Test more + touching ground + snap?
    {
        Vector3 gravityDirection = gravity.normalized;
        Vector3 hoverForce = default;

        if (Physics.Raycast(colliderBottomPosition, gravityDirection, out RaycastHit hit, float.PositiveInfinity, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            float maxMultiplier = 1 / Time.fixedDeltaTime;
            float hoverError = _hoverHeight - hit.distance;
            Vector3 velocityParallelToGravity = rb.velocity.ComponentInDirection(gravityDirection);
            bool travellingAgainstGravity = velocityParallelToGravity.IsComponentInDirectionPositive(-1f * gravityDirection);
            
            if (hoverError > 0)
            {
                hoverForce = (Math.Min(_hoverStrength, maxMultiplier) * -hoverError * gravityDirection) - (Math.Min(_hoverDamp, maxMultiplier) * velocityParallelToGravity);
            }
            else if (!travellingAgainstGravity && Math.Abs(hoverError) < _hoverSnap)
            {
                hoverForce = (Math.Min(_hoverStrength, maxMultiplier) * -hoverError * gravityDirection) + (Math.Min(_hoverDamp, maxMultiplier) * velocityParallelToGravity);
            }
        }
        AddForce(hoverForce, ForceMode.Acceleration);
    }

    public void MoveWithForce(Vector3 moveVector, bool ignoreVelocity = false)
    {
        Vector3 velocityChange = moveVector.FlattenAgainstDirection(groundNormal) - rb.velocity.RemoveComponentInDirection(groundNormal);
        Vector3 force = default;

        if (ignoreVelocity)
        {
            force = moveVector;
        }
        else
        {
            force = velocityChange;
        }

        if (velocityChange.IsComponentInDirectionPositive(rb.velocity) || rb.velocity.ComponentInDirection(velocityChange) == Vector3.zero)
        {
            force = Vector3.ClampMagnitude(force, moveMaxAcceleration);
        }
        else
        {
            force = Vector3.ClampMagnitude(force, moveMaxDeceleration);
        }

        force = force / Time.fixedDeltaTime;

        AddForce(force, ForceMode.Acceleration);
    }

    public void OrientWithForce(Quaternion targetRotation) //FIX?
    {
        Quaternion deltaRotation = transform.rotation.ShortestRotation(targetRotation);
        Vector3 rotAxis;
        float rotDegrees;

        deltaRotation.ToAngleAxis(out rotDegrees, out rotAxis);
        rotAxis.Normalize();

        float rotRadians = rotDegrees * Mathf.Deg2Rad;
        Vector3 rotationForce = rotAxis * rotRadians * _orientStrength - rb.angularVelocity * _orientDamp;
        
        AddTorque(rotationForce, ForceMode.Acceleration);
    }

    public void PhysicsUpdate()
    {
        if (isKinematic)
        {
            _forceAccumulator = default;
            _torqueAccumulator = default;
            return;
        }

        CheckGround(_forceAccumulator);

        //This is the only place where forces should be applied to the rigidbody
        //GroundedCheck and other code rely on _forceAccumulator and _torqueAccumulator being representative of ALL forces/torque being applied
        rb.velocity += _forceAccumulator;
        rb.angularVelocity += _torqueAccumulator;

        if (rb.velocity.magnitude > velocityCap)
        {
            rb.velocity.SetMagnitude(velocityCap);
        }
        if (rb.angularVelocity.magnitude > angularVelocityCap)
        {
            rb.angularVelocity.SetMagnitude(angularVelocityCap);
        }

        _forceAccumulator = default;
        _torqueAccumulator = default;
    }

    public void Move(Vector3 moveVector)
    {
        transform.position += moveVector;
    }
    
    public void Orient(Quaternion newRotation)
    {
        transform.rotation = newRotation;
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
        AddForce(_gravity, ForceMode.Acceleration);
    }

    public void Halt()
    {
        rb.velocity = default;
        rb.angularVelocity = default;
    }

    void Start()
    {
        StartCollisionCheck();
    }

    void FixedUpdate()
    {
        PhysicsUpdate();
    }

    public void TetherBounce(Vector3 tetherPoint, float tetherMaxLength, float bounceMultiplier = 1f)
    {
        if (Vector3.Distance(transform.position, tetherPoint) < tetherMaxLength)
        {
            return;
        }

        Vector3 direction = (transform.position - tetherPoint).normalized;

        if (rb.velocity.IsComponentInDirectionPositive(direction))
        {
            rb.velocity = rb.velocity - rb.velocity.ComponentInDirection(direction) * (1 + Math.Abs(bounceMultiplier));
        }

        transform.position = tetherPoint + direction * tetherMaxLength;
    }

    public void TetherFlatten(Vector3 tetherPoint, float tetherMaxLength)
    {
        if (Vector3.Distance(transform.position, tetherPoint) < tetherMaxLength)
        {
            return;
        }

        Vector3 direction = (transform.position - tetherPoint).normalized;

        if (rb.velocity.IsComponentInDirectionPositive(direction))
        {
            rb.velocity = rb.velocity.FlattenAgainstDirection(direction);
        }

        transform.position = tetherPoint + direction * tetherMaxLength;
    }
}