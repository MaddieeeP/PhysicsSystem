using System;
using UnityEngine;

public class PhysicActor : PhysicObject
{
    [SerializeField] protected float _hoverHeight = 0.5f;
    [SerializeField] protected float _minimumDynamicSpeed = 0.01f;
    [SerializeField] protected float _moveMaxSpeed = 5f;
    [SerializeField] protected float _moveMaxAcceleration = 5f; //Must be positive
    [SerializeField] protected float _moveMaxDeceleration = 10f; //Must be positive and not less than _moveMaxAcceleration
    [SerializeField] protected float _moveControlGrounded = 1f;
    [SerializeField] protected float _moveControlAerial = 0.5f;
    [SerializeField] protected float _moveStrengthAerial = 0.5f;
    [Space]
    [SerializeField] protected float _hoverStrength = 0.75f;
    [SerializeField] protected float _hoverDamp = 0.95f;
    [SerializeField] protected float _hoverSnap = 0.1f;
    [Space]
    [SerializeField] protected Vector3 _relativeCastOrigin = new Vector3(0f, 0f, 0f);
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
    protected Vector3 _groundNormal = default;
    protected PhysicMaterial _groundPhysicMaterial;

    //getters and setters
    public bool grounded { get { return _groundNormal != Vector3.zero; } }
    public float moveMaxSpeed { get { return _moveMaxSpeed; } }
    public float moveMaxAcceleration { get { return _moveMaxAcceleration; } }
    public float moveMaxDeceleration { get { return _moveMaxDeceleration; } }
    public Vector3 targetMove { get { return _targetMove; } }
    public Vector3 targetForward { get { return _targetForward; } }
    public Vector3 compositeUp { get { return _compositeUp.normalized; } }
    public override Vector3 groundNormal { get { return _groundNormal; } }
    public virtual bool forceUngrounded { get { return false; } }

    public void SetTargetMove(Vector3 targetMove)
    {
        _targetMove = Vector3.ClampMagnitude(targetMove, 1f);
    }

    public void SetTargetForward(Vector3 targetForward)
    {
        if (targetForward == Vector3.zero)
        {
            _targetForward = transform.forward;
            return;
        }

        _targetForward = targetForward.normalized;
    }

    public virtual void Jump()
    {
        if (isKinematic)
        {
            return;
        }

        AddForce(_compositeUp * _jumpVelocity, ForceMode.VelocityChange);
        UngroundActor();
    }

    protected virtual RaycastHit CheckGround()
    {
        Vector3 sweepVector = (subjectiveVelocity + GetCurrentForceAccumulator()) * Time.fixedDeltaTime - _compositeUp * (_hoverHeight + _hoverSnap);
        Physics.Raycast(transform.position + transform.rotation * _relativeCastOrigin, sweepVector, out RaycastHit hit, sweepVector.magnitude, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore);
        Debug.DrawLine(transform.position + transform.rotation * _relativeCastOrigin, transform.position + transform.rotation * _relativeCastOrigin + sweepVector);
        return hit;
    }

    protected void GroundActor(Vector3 normal, PhysicMaterial physicMaterial) 
    {
        _groundNormal = normal;
        _groundPhysicMaterial = physicMaterial;

        Vector3 gravityUp = -gravity.normalized;
        _compositeUp = Vector3.Lerp(gravityUp, _groundNormal, Math.Clamp(_groundAngleInfluence.Evaluate(Vector3.Angle(gravityUp, _groundNormal) / 180f), 0f, 1f)).normalized;
    }

    protected void UngroundActor()
    {
        _groundNormal = default;
        _groundPhysicMaterial = null;
        _compositeUp = -gravity.normalized;
    }

    private Vector3 CalculateMoveForce(Vector3 flattenVector, float moveControl, float friction)
    {
        Vector3 moveVector = _targetMove.FlattenAgainstAxis(flattenVector) * _moveMaxSpeed;
        Vector3 predictedVelocityOnPlane = (subjectiveVelocity + GetCurrentForceAccumulator()).RemoveComponentAlongAxis(flattenVector);
        Vector3 predictedVelocityParallel = predictedVelocityOnPlane.ComponentAlongAxis(moveVector);
        Vector3 predictedVelocityPerpendicular = predictedVelocityOnPlane - predictedVelocityParallel;

        float simMoveMaxAcceleration = _moveMaxAcceleration * experiencedFixedDeltaTime * friction;
        float simMoveMaxDeceleration = _moveMaxDeceleration * experiencedFixedDeltaTime * friction;

        Vector3 accelerationForce = Vector3.ClampMagnitude(Vector3.ClampMagnitude(predictedVelocityOnPlane + moveVector, moveMaxSpeed) - predictedVelocityOnPlane, simMoveMaxAcceleration);

        Vector3 velocityChangeForce;
        if (predictedVelocityOnPlane.SignedMagnitudeInDirection(moveVector) >= 0f) //accelerating in moveDirection
        {
            velocityChangeForce = Vector3.ClampMagnitude(Vector3.ClampMagnitude(moveVector - predictedVelocityParallel, simMoveMaxAcceleration) - predictedVelocityPerpendicular, simMoveMaxDeceleration);
        }
        else
        {
            velocityChangeForce = Vector3.ClampMagnitude(Vector3.ClampMagnitude(moveVector + predictedVelocityParallel, simMoveMaxAcceleration) - Vector3.ClampMagnitude(predictedVelocityParallel, simMoveMaxDeceleration) - predictedVelocityPerpendicular, simMoveMaxDeceleration);
        }

        return Vector3.Lerp(accelerationForce, velocityChangeForce, moveControl);
    }

    protected virtual float GetFrictionForPhysicMaterial(PhysicMaterial physicMaterial, bool useStaticFriction)
    {
        return useStaticFriction ? physicMaterial.staticFriction : physicMaterial.dynamicFriction;
    }

    protected void ActionUpdate()
    {
        if (isKinematic)
        {
            return;
        }

        //Floating and ground check
        if (!forceUngrounded)
        {
            RaycastHit hit = CheckGround();

            if (hit.collider == null)
            {
                UngroundActor();
            }
            else
            {
                float hoverError = _hoverHeight - hit.distance;
                float subjectiveSpeedFromSurface = subjectiveVelocity.SignedMagnitudeInDirection(hit.normal);

                if (hoverError == 0f)
                {
                    GroundActor(hit.normal, hit.collider.material);
                }
                else if (hoverError > 0f) //Actor is too close to the surface
                {
                    GroundActor(hit.normal, hit.collider.material);
                    AddForce((_hoverStrength * hoverError - GetCurrentForceAccumulator().SignedMagnitudeInDirection(hit.normal) - subjectiveSpeedFromSurface * _hoverDamp) * hit.normal, ForceMode.VelocityChange); //FIX - groundphysicsmat
                }
                else //Actor is too far from the surface, but within snap range
                {
                }
            }
        }

        //Movement
        if (grounded)
        {
            float friction = GetFrictionForPhysicMaterial(_groundPhysicMaterial, subjectiveVelocity.RemoveComponentAlongAxis(_groundNormal).magnitude < _minimumDynamicSpeed);
            
            AddForce(CalculateMoveForce(_groundNormal, _moveControlGrounded, friction), ForceMode.VelocityChange);
        }
        else
        {
            AddForce(CalculateMoveForce(gravity, _moveControlAerial, _moveStrengthAerial), ForceMode.VelocityChange);
        }
        
        //Orientation
        Vector3 lookForward = _targetForward == default ? transform.forward : _targetForward;
        transform.rotation.ShortestRotation(Quaternion.LookRotation(lookForward.RemoveComponentAlongAxis(_compositeUp), _compositeUp)).ToAngleAxis(out float rotDegrees, out Vector3 rotAxis);
        Vector3 rotationForce = rotAxis * rotDegrees * Math.Min(1f, (grounded ? _orientStrengthGrounded : _orientStrengthAerial) * experiencedTime) - GetCurrentTorqueAccumulator() - subjectiveAngularVelocity * _orientDamp;

        AddTorque(rotationForce, ForceMode.VelocityChange);

        ForceUpdate();
    }

    protected override void LateFixedUpdate() //Runs after physics simulate
    {
        BuiltInForceCorrection();
    }
}