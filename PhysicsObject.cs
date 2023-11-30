using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]
[RequireComponent(typeof(MeshFilter))]

public class PhysicsObject : MonoBehaviour
{
    public static float relativeTime = 1f;
    public static float velocityCap = 200f;

    [SerializeField] protected float _height = 1f;
    [SerializeField] protected float _groundedRayDist = 0.15f;
    protected bool _hasPhysics = true;
    protected bool _grounded = false;
    [SerializeField] protected float _moveMaxSpeed = 5f;
    [SerializeField] protected float _moveMaxAcceleration = 0.5f;
    [SerializeField] protected float _moveMaxDeceleration = 0.5f;
    private Dictionary<GravitationalField, Vector3> _fieldGravityVectors = new Dictionary<GravitationalField, Vector3>() { };
    private List<GameObject> _currentCollsions = new List<GameObject>();
    protected Vector3 _position = default;
    protected Quaternion _rotation = default;

    public float height { get { return _height; } }
    public bool hasPhysics { get { return _hasPhysics; } set { _hasPhysics = value; } }
    public bool grounded { get { return _grounded; } }
    public float moveMaxSpeed { get { return _moveMaxSpeed; } }
    public float moveMaxAcceleration { get { return Math.Abs(_moveMaxAcceleration); } }
    public float moveMaxDeceleration { get { return Math.Abs(_moveMaxDeceleration); } }
    public Vector3 gravity
    {
        get
        {
            Vector3 gravityVector = Vector3.zero;
            List<GravitationalField> fields = _fieldGravityVectors.Keys.ToList();
            foreach (GravitationalField field in fields)
            {
                Vector3 vector = field.GetGravity(this);
                _fieldGravityVectors[field] = vector;
                gravityVector += vector;
            }
            return gravityVector;
        }
    }
    public Vector3 bottomOffset { get { return bottomPosition - transform.position; } }
    public Vector3 bottomPosition { get { return collider.bounds.center + transform.up * (- 0.5f * height + 0.01f); } }
    public Vector3 groundNormal { get { return GroundNormal(); } }
    public List<GameObject> currentCollisions { get { return _currentCollsions; } }
    public Vector3 position { get { return _position; } set { _position = value; } }
    public Quaternion rotation { get { return _rotation; } set { _rotation = value; } }
    public Rigidbody rigidBody { get { return gameObject.GetComponent<Rigidbody>(); } }
    new Collider collider { get { return gameObject.GetComponent<Collider>(); } }
    public Mesh mesh { get { return gameObject.GetComponent<MeshFilter>().mesh; } }

    public void EnterCollision(Collision collision)
    {
        _currentCollsions.Add(collision.gameObject);

        GravitationalField field = collision.gameObject.GetComponent<GravitationalField>();
        if (field != null)
        {
            _fieldGravityVectors[field] = field.GetGravityOnEnter(this);
            Debug.Log(collision.gameObject.name);
        }
    }

    public void StayCollision(Collision collision)
    {
        GameObject obj = collision.gameObject;

        if (!_currentCollsions.Contains(obj))
        {
            _currentCollsions.Add(obj);
        }
    }

    public void ExitCollision(Collision collision)
    {
        _currentCollsions.Remove(collision.gameObject);

        GravitationalField field = collision.gameObject.GetComponent<GravitationalField>();
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

    public void EnterTrigger(Collider collider)
    {
        _currentCollsions.Add(collider.gameObject);

        GravitationalField field = collider.gameObject.GetComponent<GravitationalField>();
        if (field != null)
        {
            _fieldGravityVectors[field] = field.GetGravityOnEnter(this);
        }
    }

    public void StayTrigger(Collider collider)
    {
        GameObject obj = collider.gameObject;

        if (!_currentCollsions.Contains(obj))
        {
            _currentCollsions.Add(obj);
        }
    }

    public void ExitTrigger(Collider collider)
    {
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
        if (!_hasPhysics) 
        {
            return;
        }

        rigidBody.AddForce(force, forceMode);
    }

    public void AddForceAtPosition(Vector3 force, Vector3 position, ForceMode forceMode = ForceMode.Force)
    {
        if (!_hasPhysics)
        {
            return;
        }

        rigidBody.AddForceAtPosition(force, position, forceMode);
    }

    public void StandAt(Vector3 position)
    {
        transform.position = position - bottomOffset;
    }

    public void StartCollisionCheck()
    {
        _fieldGravityVectors = new Dictionary<GravitationalField, Vector3>() { };
        List<GravitationalField> fields = FindObjectsOfType<GravitationalField>().Cast<GravitationalField>().ToList();

        foreach (GravitationalField field in fields)
        {
            if (collider.bounds.Intersects(field.collider.bounds))
            {
                _currentCollsions.Add(field.gameObject);
                _fieldGravityVectors[field] = field.GetGravity(this);
            }
        }
    }

    public bool GroundedCheck(Vector3 force)
    {
        Vector3 gravityDirection = gravity.normalized;

        if (Physics.Raycast(bottomPosition, gravityDirection, 0.05f))
        {
            return true;
        }

        return false;
    }

    private Vector3 GroundNormal()
    {
        Vector3 gravityDirection = gravity.normalized;
        Vector3 normal = -1f * gravityDirection;

        if (Physics.Raycast(bottomPosition, gravityDirection, out RaycastHit hit, _groundedRayDist))
        {
            normal = hit.normal;
        }

        return normal.normalized;
    }

    private Vector3 MoveForce(Vector3 moveVector)
    {
        Vector3 velocityChange = moveVector.FlattenAgainstDirection(groundNormal) - rigidBody.velocity.RemoveComponentInDirection(groundNormal); //v-u
        Vector3 force = rigidBody.mass * velocityChange / Time.fixedDeltaTime * 0.5f; //ma
        
        if (velocityChange.magnitude != 0)
        {
            force = force / velocityChange.magnitude;
        }

        if (velocityChange.IsComponentInDirectionPositive(rigidBody.velocity) || rigidBody.velocity.ComponentInDirection(velocityChange) == Vector3.zero)
        {
            force = force * Math.Min(moveMaxAcceleration, velocityChange.magnitude);
        }
        else
        {
            force = force * Math.Min(moveMaxDeceleration, velocityChange.magnitude);
        }

        return force;
    }

    public void PhysicsUpdate()
    {
        if (!_hasPhysics)
        {
            Halt();
            transform.position = _position;
            transform.rotation = _rotation;
            return;
        }

        _position = transform.position;
        _rotation = transform.rotation;

        AddForce(gravity, ForceMode.Acceleration);
        
        _grounded = GroundedCheck(rigidBody.velocity + gravity * Time.fixedDeltaTime);
    }

    public void MoveWithForce(Vector3 moveVector)
    {
        if (!_hasPhysics)
        {
            return;
        }

        Vector3 angularVelocity = rigidBody.angularVelocity;
        Vector3 moveForce = MoveForce(moveVector);
        AddForce(moveForce);
        rigidBody.angularVelocity = angularVelocity;
    }

    public void Move(Vector3 moveVector)
    {
        _position += moveVector;
        transform.position += moveVector;
    }
     
    public void Halt()
    {
        rigidBody.velocity = Vector3.zero;
        rigidBody.angularVelocity = Vector3.zero;
    }

    void Start()
    {
        StartCollisionCheck();
    }

    void FixedUpdate()
    {
        PhysicsUpdate();
    }

    public void Tether(Vector3 tetherPoint, float tetherMaxLength)
    {
        if (Vector3.Distance(transform.position, tetherPoint) < tetherMaxLength)
        {
            return;
        }

        Vector3 direction = (tetherPoint - transform.position).normalized;

        if (!rigidBody.velocity.IsComponentInDirectionPositive(direction))
        {
            rigidBody.velocity = rigidBody.velocity.FlattenAgainstDirection(direction);
        }

        transform.position = tetherPoint - direction * tetherMaxLength;
    }
}