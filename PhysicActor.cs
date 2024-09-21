using System;
using UnityEngine;

public class PhysicActor : PhysicObject
{
    [SerializeField] protected float _hoverHeight = 0.5f;
    [SerializeField] protected float _minimumDynamicSpeed = 0.01f; //Essentially equal to maximum static speed
    [SerializeField] protected float _moveMaxSpeed = 5f; //Must be positive
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
    [SerializeField] protected Vector3 _relativeCastOrigin = Vector3.zero;
    [SerializeField] protected int _maxNormalChange = 45; //In cases where this value is vital, set 5 or so below actual desired value to account for imprecisions
    [Space]
    [SerializeField] protected float _orientStrengthGrounded = 0.95f;
    [SerializeField] protected float _orientStrengthAerial = 0.75f;
    [SerializeField] protected float _orientDamp = 0.5f;
    [Space]
    [SerializeField] protected float _jumpVelocity = 5f;
    [Space]
    [SerializeField] protected AnimationCurve _groundAngleInfluence;

    private Vector3 _compositeUp = default;
    protected Vector3 _groundNormal = default;
    protected PhysicMaterial _groundPhysicMaterial;

    //getters and setters
    public bool grounded { get { return _groundNormal != Vector3.zero; } }
    public float moveMaxSpeed { get { return _moveMaxSpeed; } }
    public float moveMaxAcceleration { get { return _moveMaxAcceleration; } }
    public float moveMaxDeceleration { get { return _moveMaxDeceleration; } }
    public Vector3 compositeUp { get { return _compositeUp.normalized; } }
    public override Vector3 groundNormal { get { return _groundNormal; } }
    public virtual bool forceUngrounded { get { return false; } }

    public virtual void Jump()
    {
        if (isKinematic)
        {
            return;
        }

        AddForce(_compositeUp * _jumpVelocity, ForceMode.VelocityChange);
        UngroundActor();
    }

    protected virtual float GetFrictionForPhysicMaterial(PhysicMaterial physicMaterial, bool useStaticFriction)
    {
        return useStaticFriction ? physicMaterial.staticFriction : physicMaterial.dynamicFriction;
    }

    protected virtual RaycastHit GetSurfaceHit(RaycastHit hit)
    {
        if (Vector3.Angle(hit.normal, groundNormal == default ? -gravity : groundNormal) > _maxNormalChange)
        {
            return default;
        }

        return hit;
    }

    protected RaycastHit CheckGround()
    {
        Vector3 sweepVector = (subjectiveVelocity + GetCurrentForceAccumulator()) * 0.05f - _compositeUp * (_hoverHeight + _hoverSnap); //0.05f is an arbitary constant for how the time step this function considers; it must be a constant to maintain predictability
        Vector3 castOrigin = transform.position + transform.rotation * _relativeCastOrigin;

        Collider thisCollider = transform.GetComponent<Collider>();
        bool colliderActive = thisCollider.enabled;
        thisCollider.enabled = false;

        RaycastHit[] hits = Physics.RaycastAll(castOrigin, sweepVector, sweepVector.magnitude, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore);

        if (hits.Length == 0)
        {
            thisCollider.enabled = colliderActive;
            return default;
        }

        thisCollider.enabled = colliderActive;

        foreach (RaycastHit hit in hits)
        {
            RaycastHit surfaceHit = GetSurfaceHit(hit);

            if (surfaceHit.normal != default)
            {
                return surfaceHit;
            }    
        }

        return default;
    }

    protected void GroundActor(Vector3 normal, PhysicMaterial physicMaterial) 
    {
        _groundNormal = normal;
        _groundPhysicMaterial = physicMaterial;

        Vector3 gravityUp = -gravity.normalized;
        _compositeUp = Vector3.Lerp(gravityUp, normal, _groundAngleInfluence.Evaluate(Vector3.Angle(gravityUp, normal) / 180f)).normalized;
    }

    protected void UngroundActor()
    {
        _groundNormal = default;
        _groundPhysicMaterial = null;
        _compositeUp = -gravity.normalized;
    }

    private Vector3 CalculateMoveForce(Vector3 targetMove, Vector3 flattenVector, float moveControl, float friction)
    {
        Vector3 moveVector = targetMove.FlattenAgainstAxis(flattenVector) * _moveMaxSpeed;
        Vector3 predictedVelocityOnPlane = (subjectiveVelocity + GetCurrentForceAccumulator()).RemoveComponentAlongAxis(flattenVector);
        Vector3 predictedVelocityParallel = predictedVelocityOnPlane.ComponentAlongAxis(moveVector);
        Vector3 predictedVelocityPerpendicular = predictedVelocityOnPlane - predictedVelocityParallel;

        float simMoveMaxAcceleration = _moveMaxAcceleration * experiencedSimulationDeltaTime * friction;
        float simMoveMaxDeceleration = _moveMaxDeceleration * experiencedSimulationDeltaTime * friction;

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

    public void ActionUpdate(Vector3 targetMove, Vector3 targetForward)
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

                if (hoverError < 0f) //Actor is too far from the surface, but within snap range
                {
                    GroundActor(hit.normal, hit.collider.material);
                    //FIX
                }
                else //Actor is too close to the surface or at the correct height
                {
                    GroundActor(hit.normal, hit.collider.material);
                    float dampingForce = (Math.Max(0f, GetCurrentForceAccumulator().SignedMagnitudeInDirection(-hit.normal)) - subjectiveSpeedFromSurface * _hoverDamp) * Math.Min(_moveMaxSpeed, subjectiveVelocity.magnitude) / _moveMaxSpeed;
                    AddForce((_hoverStrength * hoverError + dampingForce) * hit.normal, ForceMode.VelocityChange); //FIX - groundphysicsmat
                }
            }    
        }

        //Movement
        if (grounded)
        {
            float friction = GetFrictionForPhysicMaterial(_groundPhysicMaterial, subjectiveVelocity.RemoveComponentAlongAxis(_groundNormal).magnitude < _minimumDynamicSpeed);
            
            AddForce(CalculateMoveForce(targetMove, _groundNormal, _moveControlGrounded, friction), ForceMode.VelocityChange);
        }
        else
        {
            AddForce(CalculateMoveForce(targetMove, gravity, _moveControlAerial, _moveStrengthAerial), ForceMode.VelocityChange);
        }
        
        //Orientation
        Vector3 lookForward = targetForward == default ? transform.forward : targetForward;
        transform.rotation.ShortestRotation(Quaternion.LookRotation(lookForward.RemoveComponentAlongAxis(_compositeUp), _compositeUp)).ToAngleAxis(out float rotDegrees, out Vector3 rotAxis);
        Vector3 rotationForce = (grounded ? _orientStrengthGrounded : _orientStrengthAerial) * rotDegrees * rotAxis - GetCurrentTorqueAccumulator() - _orientDamp * globalTime * relativeTime * subjectiveAngularVelocity; //FIX -something is(n't) time scaled

        AddTorque(rotationForce, ForceMode.VelocityChange);

        ForceUpdate();
    }

    public override void FixedUpdate()
    {
        if (_simulationModeAsScript)
        {
            return;
        }

        ActionUpdate(default, transform.forward);
    }

    public override void LateFixedUpdate() //Runs after physics simulate
    {
        BuiltInForceCorrection();
    }
}