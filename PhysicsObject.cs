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
    protected bool _hasPhysics = true;
    protected bool _grounded = false;
    [SerializeField] protected float _moveMaxSpeed = 5f;
    [SerializeField] protected float _moveMaxAcceleration = 0.5f;
    [SerializeField] protected float _moveMaxDeceleration = 0.5f;
    protected List<GameObject> _currentCollsions = new List<GameObject>();

    public float height { get { return _height; } }
    public bool hasPhysics { get { return _hasPhysics; } set { _hasPhysics = value; } }
    public bool grounded { get { return _grounded; } }

    public float moveMaxSpeed { get { return _moveMaxSpeed; } }
    public float moveMaxAcceleration { get { return Math.Abs(_moveMaxAcceleration); } }
    public float moveMaxDeceleration { get { return Math.Abs(_moveMaxDeceleration); } }

    public Vector3 totalGravity
    {
        get
        {
            Vector3 gravity = Vector3.zero;
            foreach (GameObject obj in currentCollisions)
            {
                GravitationalField field = obj.GetComponent<GravitationalField>();
                if (field != null)
                {
                    if (field.overwrite)
                    {
                        return field.GetGravity(this);
                    }
                    gravity += field.GetGravity(this);
                }
            }
            return gravity;
        }
    }
    public float gravityMagnitude { get { return totalGravity.magnitude; } }
    public Vector3 gravityDirection { get { return totalGravity.normalized; } }

    public Vector3 bottomOffset { get { return bottomPosition - transform.position; } }
    public Vector3 bottomPosition { get { return collider.bounds.center + transform.up * (- 0.5f * height + 0.01f); } }
    public Vector3 groundNormal { get { return GroundNormal(); } }

    public List<GameObject> currentCollisions { get { return _currentCollsions; } }

    public Rigidbody rigidBody { get { return gameObject.GetComponent<Rigidbody>(); } }
    new Collider collider { get { return gameObject.GetComponent<Collider>(); } }
    public Mesh mesh { get { return gameObject.GetComponent<MeshFilter>().mesh; } }

    public void EnterCollision(Collision collision)
    {
        _currentCollsions.Add(collision.gameObject);
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

    public void AddForce(Vector3 force, ForceMode forceMode = ForceMode.Force)
    {
        rigidBody.AddForce(force, forceMode);
    }

    public void AddForceAtPosition(Vector3 force, Vector3 position, ForceMode forceMode = ForceMode.Force)
    {
        rigidBody.AddForceAtPosition(force, position, forceMode);
    }

    public void StandAt(Vector3 position)
    {
        transform.position = position - bottomOffset;
    }

    public void StartCollisionCheck()
    {
        List<GravitationalField> fields = FindObjectsOfType<GravitationalField>().Cast<GravitationalField>().ToList();

        foreach (GravitationalField field in fields)
        {
            if (collider.bounds.Intersects(field.collider.bounds))
            {
                _currentCollsions.Add(field.gameObject);
            }
        }
    }

    public bool GroundedCheck(Vector3 force)
    {
        if (Physics.Raycast(bottomPosition, gravityDirection, 0.05f))
        {
            return true;
        }
        return false;
    }

    private Vector3 GroundNormal()
    {
        Vector3 normal = -1f * gravityDirection;

        if (Physics.Raycast(bottomPosition, gravityDirection, out RaycastHit hit, 0.15f))
        {
            normal = hit.normal;
        }

        return normal.normalized;
    }

    private Vector3 Gravity()
    {
        return rigidBody.mass * totalGravity;
    }

    private Vector3 Move(Vector3 moveVector)
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
            return;
        }

        Vector3 totalForce = new Vector3(0f, 0f, 0f);
        totalForce += Gravity();
        //totalForce += AirResistance(); -gamey
        AddForceAtPosition(totalForce, collider.bounds.center);
        
        _grounded = GroundedCheck(rigidBody.velocity + totalForce / rigidBody.mass * Time.fixedDeltaTime);
    }

    public void MoveWithForce(Vector3 moveVector)
    {
        Vector3 angularVelocity = rigidBody.angularVelocity;
        Vector3 moveForce = Move(moveVector);
        AddForce(moveForce);
        rigidBody.angularVelocity = angularVelocity;
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