using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Linq;
using UnityEngine;

public class PhysicActor : PhysicObject
{
    [SerializeField] protected float _moveMaxSpeed = 5f;
    [SerializeField] protected float _moveMaxAcceleration = 0.5f;
    [SerializeField] protected float _moveMaxDeceleration = 0.5f;
    [SerializeField] protected float _hoverHeight = 0.5f;
    [SerializeField] protected float _hoverStrength = 50f;
    [SerializeField] protected float _hoverDamp = 20f;
    [SerializeField] protected float _hoverSnap = 0.1f;
    [SerializeField] protected float _orientGroundedStrength = 100f;
    [SerializeField] protected float _orientAerialStrength = 90f;
    [SerializeField] protected float _orientDamp = 20f;
    [SerializeField] protected float _jumpVelocity = 2f;
    [SerializeField] protected AnimationCurve _groundAngleInfluence;
    [SerializeField] protected Vector3 _compositeUp = default;

    //getters and setters
    public float moveMaxSpeed { get { return _moveMaxSpeed; } }
    public float moveMaxAcceleration { get { return Math.Abs(_moveMaxAcceleration); } }
    public float moveMaxDeceleration { get { return Math.Abs(_moveMaxDeceleration); } }
    public float currentOrientStrength { get { return grounded ? _orientGroundedStrength : _orientAerialStrength; } }

    public void HoverWithForce() //FIX - Test more + touching ground + snap? + bounciness using physicmaterial
    {
        Vector3 gravityDirection = gravity.normalized;
        Vector3 hoverForce = default;

        if (Physics.Raycast(colliderBottomPosition, gravityDirection, out RaycastHit hit, float.PositiveInfinity, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            float maxMultiplier = 1 / Time.fixedDeltaTime;
            float hoverError = _hoverHeight - hit.distance;
            Vector3 velocityParallelToGravity = rb.velocity.ComponentAlongAxis(gravityDirection);
            bool travellingAgainstGravity = velocityParallelToGravity.IsComponentInDirectionPositive(-1f * gravityDirection);

            if (hoverError > 0)
            {
                hoverForce = (Math.Min(_hoverStrength, maxMultiplier) * -hoverError * gravityDirection) - (Math.Min(_hoverDamp, maxMultiplier) * velocityParallelToGravity);
            }
            else if (!travellingAgainstGravity && Math.Abs(hoverError) < _hoverSnap) //FIX
            {
                hoverForce = default; // (Math.Min(_hoverStrength, maxMultiplier) * -hoverError * gravityDirection) + (Math.Min(_hoverDamp, maxMultiplier) * velocityParallelToGravity);
            }
        }
        AddForce(hoverForce, ForceMode.Acceleration);
    }

    public void MoveWithForce(Vector3 moveInputVector, bool additive = false)
    {
        Vector3 moveVector = moveInputVector.FlattenAgainstAxis(groundNormal) * moveMaxSpeed * relativeTime;
        Vector3 velocityChange = moveVector - rb.velocity.RemoveComponentAlongAxis(groundNormal);

        Vector3 force = additive ? Vector3.ClampMagnitude(moveVector, velocityChange.magnitude) : velocityChange;
        force = velocityChange.IsComponentInDirectionPositive(-rb.velocity) ? Vector3.ClampMagnitude(force, moveMaxDeceleration) : force = Vector3.ClampMagnitude(force, moveMaxAcceleration);

        if (!_grounded)
        {
            AddForce(force, ForceMode.VelocityChange);
            return;
        }

        force = rb.velocity.RemoveComponentAlongAxis(groundNormal).magnitude / Math.Max(relativeTime, 0.01f) < 0.01f ? force * _groundPhysicMaterial.staticFriction : force * _groundPhysicMaterial.dynamicFriction;

        AddForce(force, ForceMode.VelocityChange);
    }

    public void OrientWithForce(Quaternion targetRotation, float strength) //FIX?
    {
        Quaternion deltaRotation = transform.rotation.ShortestRotation(targetRotation);
        Vector3 rotAxis;
        float rotDegrees;

        deltaRotation.ToAngleAxis(out rotDegrees, out rotAxis);
        rotAxis.Normalize();

        float rotRadians = rotDegrees * Mathf.Deg2Rad;
        Vector3 rotationForce = rotAxis * rotRadians * strength - rb.angularVelocity * _orientDamp; //FIX - subjective?

        AddTorque(rotationForce, ForceMode.Acceleration);
    }

    public void Jump()
    {
        AddForce(_compositeUp * _jumpVelocity, ForceMode.VelocityChange);
        _grounded = false;
    }

    public void PhysicsUpdate(Vector3 moveInput, Vector3 targetForward, Transform relativeTransform)
    {
        Vector3 moveVector = moveInput.FlattenAgainstAxis(groundNormal);
        Quaternion targetRotation = Quaternion.LookRotation(targetForward, _compositeUp);

        ApplyGravity();
        MoveWithForce(moveVector);
        OrientWithForce(targetRotation, currentOrientStrength);
        CheckGround();
        HoverWithForce(); //HoverForce should not be included with CheckGround()
        
        float t = Vector3.Angle(-gravity.normalized, groundNormal) / 180f;
        _compositeUp = Vector3.Lerp(-gravity.normalized, groundNormal, t).normalized;

        ForceUpdate();
    }

    public void FixedUpdate()
    {
        Vector2 moveInput = default;
        Transform relativeTransform = transform;
        PhysicsUpdate(relativeTransform.rotation * new Vector3(moveInput.x, 0f, moveInput.y), relativeTransform.forward, relativeTransform);
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
