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

    private Vector3 _targetMove = default;
    private Vector3 _targetForward = default;
    private Vector3 _compositeUp = default;

    //getters and setters
    public float moveMaxSpeed { get { return _moveMaxSpeed; } }
    public float moveMaxAcceleration { get { return Math.Abs(_moveMaxAcceleration); } }
    public float moveMaxDeceleration { get { return Math.Max(Math.Abs(_moveMaxDeceleration), moveMaxAcceleration); } }
    public float currentOrientStrength { get { return grounded ? _orientStrengthGrounded : _orientStrengthAerial; } }
    public float currentMoveControl { get { return grounded ? _moveControlGrounded : _moveControlAerial; } }
    public float currentMoveStrength { get { return grounded ? _moveStrengthGrounded : _moveStrengthAerial; } }
    public Vector3 compositeUp { get { return _compositeUp.normalized; } }

    public void SetTargetMove(Vector3 targetMove)
    {
        _targetMove = Vector3.ClampMagnitude(targetMove, 1f);
    }

    public void SetTargetForward(Vector3 targetForward)
    {
        _targetForward = targetForward;
    }

    public void Jump()
    {
        if (paused)
        {
            return;
        }

        AddForce(_compositeUp * _jumpVelocity, ForceMode.VelocityChange);
        grounded = false;
    }

    protected void HoverWithForce() //FIX - Test more + touching ground + snap? + bounciness using physicmaterial + relativeTime?
    {
        if (paused)
        {
            return;
        }

        Rigidbody rb = gameObject.GetComponent<Rigidbody>();
        Vector3 gravityDirection = gravity.normalized;
        Vector3 hoverForce = default;

        if (Physics.Raycast(transform.position, gravityDirection, out RaycastHit hit, float.PositiveInfinity, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            float hoverError = _footToCenterDist - hit.distance;
            Vector3 velocityParallelToGravity = (rb.velocity * relativeTime).ComponentAlongAxis(gravityDirection);
            bool travellingAgainstGravity = velocityParallelToGravity.IsComponentInDirectionPositive(-1f * gravityDirection);

            if (hoverError > 0)
            {
                hoverForce = _hoverStrength * -hoverError * gravityDirection / Time.fixedDeltaTime - _hoverDamp * velocityParallelToGravity; //FIX - this needs to be more intuitive...????
            }
            else if (!travellingAgainstGravity && Math.Abs(hoverError) < _hoverSnap) //FIX
            {
                hoverForce = default;
                //hoverForce = (Math.Min(_hoverStrength, maxMultiplier) * -hoverError * gravityDirection) + (Math.Min(_hoverDamp, maxMultiplier) * velocityParallelToGravity);
            }
        }
        AddForce(hoverForce, ForceMode.VelocityChange);
    }

    protected void MoveWithForce(float strength = 1f, float control = 1f) //accelerate subjectiveVelocity to _targetMove * _moveMaxSpeed or add _targetMove as force to subjectiveVelocity until max speed reached
    {
        if (paused)
        {
            return;
        }

        Rigidbody rb = gameObject.GetComponent<Rigidbody>();

        Vector3 moveVector = _targetMove.FlattenAgainstAxis(gravity).FlattenAgainstAxis(groundNormal) * _moveMaxSpeed; //moveVector is flattened twice so that the movement is predictable in edge cases
        Vector3 velocityOnPlane = (rb.velocity * relativeTime).RemoveComponentAlongAxis(groundNormal);
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
            force *= (rb.velocity * relativeTime).RemoveComponentAlongAxis(groundNormal).magnitude < _minimumDynamicSpeed ? _groundPhysicMaterial.staticFriction : _groundPhysicMaterial.dynamicFriction;
        }

        AddForce(force, ForceMode.VelocityChange);
    }

    protected void OrientWithForce(Quaternion targetRotation, float strength)
    {
        if (paused)
        {
            return;
        }

        Rigidbody rb = gameObject.GetComponent<Rigidbody>();

        Quaternion deltaRotation = transform.rotation.ShortestRotation(targetRotation);
        deltaRotation.ToAngleAxis(out float rotDegrees, out Vector3 rotAxis);
        Vector3 rotationForce = rotAxis * rotDegrees * Mathf.Deg2Rad * strength / Time.fixedDeltaTime - (rb.angularVelocity * relativeTime) * _orientDamp;

        AddTorque(rotationForce, ForceMode.VelocityChange);
    }

    protected void CalculateCompositeUp()
    {
        Vector3 gravityUp = -gravity.normalized;
        float t = Vector3.Angle(gravityUp, groundNormal) / 180f;
        float groundAngleMultiplier = Math.Clamp(_groundAngleInfluence.Evaluate(t), 0f, 1f);
        _compositeUp = (gravityUp.normalized * (1 - t) + groundNormal * t).normalized;
    }

    protected void PhysicsUpdate()
    {
        Vector3 lookForward = _targetForward == default ? transform.forward : _targetForward;
        
        MoveWithForce(currentMoveStrength, currentMoveControl); //MoveWithForce must be before CheckGround()
        CheckGround();
        CalculateCompositeUp();

        OrientWithForce(Quaternion.LookRotation(lookForward.RemoveComponentAlongAxis(_compositeUp), _compositeUp), currentOrientStrength);
        HoverWithForce(); //HoverWithForce must be after CheckGround()
        ForceUpdate();
    }

    protected virtual void FixedUpdate()
    {
        PhysicsUpdate();
    }
}