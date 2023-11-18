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

    public bool hasPhysics = true;
    private bool _grounded = false;
    public bool grounded { get { return _grounded; } }

    public GameObject tether = null;
    public float tetherMaxLength;

    [SerializeField] private float _moveMaxSpeed = 5f;
    public float moveMaxSpeed { get { return _moveMaxSpeed; } }
    [SerializeField] private float _moveMaxAcceleration = 0.5f;
    public float moveMaxAcceleration { get { return Math.Abs(_moveMaxAcceleration); } }
    [SerializeField] private float _moveMaxDeceleration = 0.5f;
    public float moveMaxDeceleration { get { return Math.Abs(_moveMaxDeceleration); } }


    private List<GameObject> _currentCollsions = new List<GameObject>();
    public List<GameObject> currentCollisions { get { return _currentCollsions; } }






    //FIX

    public float airDensity = 1.2f;
    public float airMagnitude = 0f; //Speed of airflow
    private Vector3 _airDirection = Vector3.zero; //Direction of airflow
    public Vector3 airDirection
    {
        get { return _airDirection.normalized; }
        set { _airDirection = value.normalized; }
    }








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

    public Vector3 bottomOffset { get { return transform.up * (collider.bounds.center.y - collider.bounds.size.y / 2 + 0.01f); } } //FIX - LOCAL!!!!!!!!!!!
    public Vector3 bottomPosition { get { return transform.position + bottomOffset; } }
    public Vector3 groundNormal { get { return GroundNormal(); } }

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
        transform.position = position - bottomOffset; //FIX - gravity direction - ?
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

    private void _groundedCheck(Vector3 force)
    {
        Debug.DrawRay(bottomPosition, force.normalized * 10f, Color.red, 10f, true);

        if (Physics.Raycast(bottomPosition, gravityDirection, 1f))
        {
            _grounded = true;
            return;
        }
        _grounded = false;
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

    private Vector3 AirResistance() //FIX - a purely physics based approach is not a good idea. These values should be able to be tweaked in a more game-y way
    {
        float dragCoefficient = 0.1f;
        float surfaceArea = (collider.bounds.size.x * collider.bounds.size.y + collider.bounds.size.y * collider.bounds.size.z + collider.bounds.size.z * collider.bounds.size.x) / 3f;
        Vector3 force = 0.5f * airDensity * surfaceArea * (float)Math.Pow((airMagnitude - rigidBody.velocity.magnitude), 2) * (airDirection - rigidBody.velocity.normalized) * dragCoefficient;
        return force;
    }

    private Vector3 Move(Vector3 moveVector)
    {
        float fixedTimeDelta = 0.01f;

        Vector3 velocityChange = moveVector.FlattenAgainstDirection(groundNormal) - rigidBody.velocity.RemoveComponentInDirection(groundNormal); //v-u
        Vector3 force = rigidBody.mass * velocityChange / fixedTimeDelta * 0.5f; //ma
        
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

    private void TetherUpdate()
    {
        if (Vector3.Distance(transform.position, tether.transform.position) < tetherMaxLength)
        {
            return;
        }

        Vector3 direction = (tether.transform.position - transform.position).normalized;

        if (!rigidBody.velocity.IsComponentInDirectionPositive(direction))
        {
            rigidBody.velocity = rigidBody.velocity.FlattenAgainstDirection(direction);
        }

        transform.position = tether.transform.position - direction * tetherMaxLength;
    }

    public void PhysicsUpdate(Vector3 moveVector)
    {
        if (!hasPhysics)
        {
            return;
        }

        Vector3 totalForce = new Vector3(0f, 0f, 0f);
        totalForce += Gravity();
        //totalForce += AirResistance();
        AddForceAtPosition(totalForce, collider.bounds.center);

        Vector3 angularVelocity = rigidBody.angularVelocity;
        Vector3 moveForce = Move(moveVector);
        AddForce(moveForce);
        rigidBody.angularVelocity = angularVelocity;

        _groundedCheck(totalForce + moveForce);

        if (tether != null)
        {
            TetherUpdate();
        }
    }

    public void PhysicsUpdate()
    {
        if (!hasPhysics)
        {
            return;
        }

        Vector3 totalForce = new Vector3(0f, 0f, 0f);
        totalForce += Gravity();
        //totalForce += AirResistance();

        AddForceAtPosition(totalForce, collider.bounds.center);
        _groundedCheck(totalForce);

        if (tether != null)
        {
            TetherUpdate();
        }
    }

    void Start()
    {
        StartCollisionCheck();
    }

    void FixedUpdate()
    {
        PhysicsUpdate();
    }
}