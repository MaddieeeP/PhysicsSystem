using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Linq;
using UnityEngine;

public class PhysicActor : PhysicObject
{
    [SerializeField] protected float _minimumDynamicSpeed = 0.01f;
    [SerializeField] protected float _moveMaxSpeed = 5f;
    [SerializeField] protected float _moveMaxAcceleration = 0.5f;
    [SerializeField] protected float _moveMaxDeceleration = 0.5f;
    [SerializeField] protected float _moveControlGrounded = 1f;
    [SerializeField] protected float _moveControlAerial = 0.01f;
    [SerializeField] protected float _moveStrengthGrounded = 1f;
    [SerializeField] protected float _moveStrengthAerial = 0.5f;
    [Space]
    [SerializeField] protected float _hoverStrength = 0.75f;
    [SerializeField] protected float _hoverDamp = 0.95f;
    [SerializeField] protected float _hoverSnap = 0.1f;
    [Space]
    [SerializeField] protected float _orientStrengthGrounded = 0.95f;
    [SerializeField] protected float _orientStrengthAerial = 0.75f;
    [SerializeField] protected float _orientDamp = 0.5f;
    [Space]
    [SerializeField] protected float _jumpVelocity = 5f;
    [Space]
    [SerializeField] protected AnimationCurve _groundAngleInfluence;

    protected Vector3 _compositeUp = default;

    //getters and setters
    public float moveMaxSpeed { get { return _moveMaxSpeed; } }
    public float moveMaxAcceleration { get { return Math.Abs(_moveMaxAcceleration); } }
    public float moveMaxDeceleration { get { return Math.Max(Math.Abs(_moveMaxDeceleration), moveMaxAcceleration); } }
    public float currentOrientStrength { get { return grounded ? _orientStrengthGrounded : _orientStrengthAerial; } }
    public float currentMoveControl { get { return grounded ? _moveControlGrounded : _moveControlAerial; } }
    public float currentMoveStrength { get { return grounded ? _moveStrengthGrounded : _moveStrengthAerial; } }
    public Vector3 compositeUp { get { return _compositeUp.normalized; } }

    public void HoverWithForce() //FIX - Test more + touching ground + snap? + bounciness using physicmaterial + relativeTime?
    {
        Vector3 gravityDirection = gravity.normalized;
        Vector3 hoverForce = default;

        if (Physics.Raycast(transform.position, gravityDirection, out RaycastHit hit, float.PositiveInfinity, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            float hoverError = _footToCenterDist - hit.distance;
            Vector3 velocityParallelToGravity = subjectiveVelocity.ComponentAlongAxis(gravityDirection);
            bool travellingAgainstGravity = velocityParallelToGravity.IsComponentInDirectionPositive(-1f * gravityDirection);

            if (hoverError > 0)
            {
                hoverForce = _hoverStrength * -hoverError * gravityDirection / Time.fixedDeltaTime - _hoverDamp * velocityParallelToGravity; //FIX - this needs to be more intuitive...????
            }
            else if (!travellingAgainstGravity && Math.Abs(hoverError) < _hoverSnap) //FIX
            {
                hoverForce = default; // (Math.Min(_hoverStrength, maxMultiplier) * -hoverError * gravityDirection) + (Math.Min(_hoverDamp, maxMultiplier) * velocityParallelToGravity);
            }
        }
        AddForce(hoverForce, ForceMode.VelocityChange);
    }

    public void MoveWithForce(Vector3 moveInputVector, float strength = 1f, float control = 1f) //accelerate subjectiveVelocity to moveInputVector or add moveInputVector as force to subjectiveVelocity until max speed reached
    {
        Vector3 moveVector = moveInputVector.FlattenAgainstAxis(groundNormal) * moveMaxSpeed;
        Vector3 velocityOnPlane = subjectiveVelocity.RemoveComponentAlongAxis(groundNormal);
        Vector3 velocityChange = moveVector - velocityOnPlane;
        Vector3 additiveMoveVector = moveVector.ClampInDirection(velocityChange, out Vector3 _);
        Vector3 force = Vector3.Lerp(additiveMoveVector, velocityChange, control) * strength; //lerp between additive force and change force

        Vector3 forceParallel = force.ComponentAlongAxis(moveVector);
        Vector3 forcePerpendicular = force - forceParallel;
        Vector3 velocityParallel = velocityOnPlane.ComponentAlongAxis(forceParallel);

        forceParallel = forceParallel.ClampRelativeChange(velocityParallel, moveMaxAcceleration, moveMaxDeceleration, out _);
        forcePerpendicular = Vector3.ClampMagnitude(forcePerpendicular, moveMaxDeceleration - forceParallel.magnitude); //perpendicular force will always be zero or a deceleration; total force magnitude will never exceed moveMaxDeceleration
        force = forceParallel + forcePerpendicular;

        if (grounded)
        {
            force *= subjectiveVelocity.RemoveComponentAlongAxis(groundNormal).magnitude < _minimumDynamicSpeed ? _groundPhysicMaterial.staticFriction : _groundPhysicMaterial.dynamicFriction;
        }

        AddForce(force, ForceMode.VelocityChange);
    }

    public void OrientWithForce(Quaternion targetRotation, float strength)
    {
        Quaternion deltaRotation = transform.rotation.ShortestRotation(targetRotation);
        deltaRotation.ToAngleAxis(out float rotDegrees, out Vector3 rotAxis);
        Vector3 rotationForce = rotAxis * rotDegrees * Mathf.Deg2Rad * strength / Time.fixedDeltaTime - subjectiveAngularVelocity * _orientDamp;

        AddTorque(rotationForce, ForceMode.VelocityChange);
    }

    public void Jump()
    {
        AddForce(_compositeUp * _jumpVelocity, ForceMode.VelocityChange);
        grounded = false;
    }

    public void CalculateCompositeUp()
    {
        Vector3 gravityUp = -gravity.normalized;
        float t = Vector3.Angle(gravityUp, groundNormal) / 180f;
        float groundAngleMultiplier = Math.Clamp(_groundAngleInfluence.Evaluate(t), 0f, 1f);
        _compositeUp = (gravityUp.normalized * (1 - t) + groundNormal * t).normalized;
    }

    public void PhysicsUpdate(Vector3 moveInput, Vector3 targetForward, Transform relativeTransform)
    {
        MoveWithForce(moveInput, currentMoveStrength, currentMoveControl); //MoveWithForce must be before CheckGround()
        CheckGround();
        CalculateCompositeUp();

        OrientWithForce(Quaternion.LookRotation(targetForward.RemoveComponentAlongAxis(_compositeUp), _compositeUp), currentOrientStrength);
        HoverWithForce();
        ForceUpdate();
    }

    public void FixedUpdate()
    {
        Vector2 moveInput = default;
        Transform relativeTransform = transform;
        PhysicsUpdate(relativeTransform.rotation * new Vector3(moveInput.x, 0f, moveInput.y), relativeTransform.forward, relativeTransform);
    }
}
