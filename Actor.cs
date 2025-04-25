using System;
using UnityEngine;

[DefaultExecutionOrder(110)]
public abstract class Actor : BasicEntity
{
    [SerializeField] protected float _minimumDynamicSpeed = 0.01f; //Essentially equal to maximum static speed
    [SerializeField] protected float _moveMaxSpeed = 5f; //Must be positive
    [SerializeField] protected float _moveMaxAcceleration = 5f; //Must be positive
    [SerializeField] protected float _moveMaxDeceleration = 10f; //Must be positive and not less than _moveMaxAcceleration
    [SerializeField] protected float _moveControlGrounded = 1f;
    [SerializeField] protected float _moveControlAerial = 0.5f;
    [SerializeField] protected float _moveStrengthAerial = 0.5f;
    [Space]
    [SerializeField] protected float _hoverHeight = 0.5f;
    [SerializeField] protected float _hoverStrength = 0.75f;
    [SerializeField] protected float _hoverDamp = 0.95f;
    [SerializeField] protected float _hoverError = 0.1f;
    [Space]
    [SerializeField] protected Vector3 _relativeCastOrigin = Vector3.zero;
    [SerializeField] protected int _maxNormalChange = 45;
    [Space]
    [SerializeField] protected float _orientStrengthGrounded = 0.95f;
    [SerializeField] protected float _orientStrengthAerial = 0.75f;
    [SerializeField] protected float _orientDamp = 0.5f;
    [Space]
    [SerializeField] protected AnimationCurve _groundAngleInfluence;

    private Vector3 _up = default;
    private RaycastHitInfoVerbose _surfaceInfo;


    //getters and setters
    public bool grounded { get { return _surfaceInfo != null; } }
    public float moveMaxSpeed { get { return _moveMaxSpeed; } }
    public float moveMaxAcceleration { get { return _moveMaxAcceleration; } }
    public float moveMaxDeceleration { get { return _moveMaxDeceleration; } }
    public override Vector3 up { get { return _up; } }

    protected override void ModifyTrueVelocities(ref Vector3 trueVelocity, ref Vector3 trueAngularVelocity, float deltaTime)
    {
        Vector3 normal;
        float moveControl;
        float friction;
        float orientStrength;
        float scaledDeltaTime = deltaTime * relativeTimeScale * SimulationController.globalTimeScale;
        Vector3 surfaceVelocity = Vector3.zero;

        if (grounded)
        {
            float hoverError;
            float speedFromSurface;

            Entity surfaceEntity = _surfaceInfo.transform.GetComponent<Entity>();
            if (surfaceEntity != null)
            {
                RaycastHitInfoVerbose adjustedSurfaceInfo = surfaceEntity.GetPredictedTransformedSurfaceInfo(_surfaceInfo, deltaTime);

                Vector3 surfaceTrueVelocity = surfaceEntity.velocity;// * surfaceEntity.relativeTimeScale * SimulationController.globalTimeScale;
                Vector3 surfaceNextTrueVelocity = (adjustedSurfaceInfo.point - _surfaceInfo.point) / deltaTime;
                Debug.Log(surfaceTrueVelocity);
                Debug.Log(surfaceNextTrueVelocity);
                trueVelocity += (surfaceNextTrueVelocity - surfaceTrueVelocity).RemoveComponentAlongAxis(adjustedSurfaceInfo.normal) * surfaceEntity.collider.material.staticFriction; //Apply surface acceleration

                hoverError = _hoverHeight - Vector3.Distance(transform.position + transform.rotation * _relativeCastOrigin, adjustedSurfaceInfo.point);
                speedFromSurface = (trueVelocity - surfaceNextTrueVelocity).SignedMagnitudeInDirection(adjustedSurfaceInfo.normal);
                normal = adjustedSurfaceInfo.normal;
            }
            else //Static surface
            {
                hoverError = _hoverHeight - _surfaceInfo.distance;
                speedFromSurface = trueVelocity.SignedMagnitudeInDirection(_surfaceInfo.normal);
                normal = _surfaceInfo.normal;
            }

            if (hoverError > 0f) //Actor is too close to the surface
            {
                float dampingForce = Math.Max(-speedFromSurface * _hoverDamp, 0f);  
                trueVelocity += (_hoverStrength * hoverError + dampingForce) * _surfaceInfo.normal;
            }
            
            moveControl = _moveControlGrounded;
            friction = velocity.magnitude / relativeTimeScale > _minimumDynamicSpeed ? _surfaceInfo.collider.material.staticFriction : _surfaceInfo.collider.material.dynamicFriction;
            orientStrength = _orientStrengthGrounded;
        }
        else
        {
            normal = _up;
            moveControl = _moveControlAerial;
            friction = _moveStrengthAerial;
            orientStrength = _orientStrengthAerial;
        }

        transform.rotation.ShortestRotation(Quaternion.LookRotation(GetTargetForward().RemoveComponentAlongAxis(_up), _up)).ToAngleAxis(out float rotDegrees, out Vector3 rotAxis);
        trueAngularVelocity += orientStrength * rotDegrees * rotAxis - _orientDamp * SimulationController.globalTimeScale * relativeTimeScale * trueAngularVelocity;

        Vector3 moveVector = GetTargetMove().FlattenAgainstAxis(normal) * _moveMaxSpeed; //FIX ALL THIS WHAT IS EVEN HAPPENING
        Vector3 predictedVelocityAlongSurfacePlane = (trueVelocity - surfaceVelocity).RemoveComponentAlongAxis(normal);
        Vector3 predictedVelocityParallel = predictedVelocityAlongSurfacePlane.ComponentAlongAxis(moveVector);
        Vector3 predictedVelocityPerpendicular = predictedVelocityAlongSurfacePlane - predictedVelocityParallel;

        float stepMaxAccelerationVelocityChange = _moveMaxAcceleration * scaledDeltaTime * friction;
        float stepMaxDecelerationVelocityChange = _moveMaxDeceleration * scaledDeltaTime * friction;

        Vector3 accelerationForce = Vector3.ClampMagnitude(Vector3.ClampMagnitude(predictedVelocityAlongSurfacePlane + moveVector, moveMaxSpeed) - predictedVelocityAlongSurfacePlane, stepMaxAccelerationVelocityChange);

        Vector3 velocityChangeForce;
        if (predictedVelocityAlongSurfacePlane.SignedMagnitudeInDirection(moveVector) >= 0f) //accelerating in moveDirection
        {
            velocityChangeForce = Vector3.ClampMagnitude(Vector3.ClampMagnitude(moveVector - predictedVelocityParallel, stepMaxAccelerationVelocityChange) - predictedVelocityPerpendicular, stepMaxDecelerationVelocityChange);
        }
        else
        {
            velocityChangeForce = Vector3.ClampMagnitude(Vector3.ClampMagnitude(moveVector + predictedVelocityParallel, stepMaxAccelerationVelocityChange) - Vector3.ClampMagnitude(predictedVelocityParallel, stepMaxDecelerationVelocityChange) - predictedVelocityPerpendicular, stepMaxDecelerationVelocityChange);
        }

        trueVelocity += Vector3.Lerp(accelerationForce, velocityChangeForce, moveControl);
    }

    public override void LateSimulationUpdate(float deltaTime)
    {
        Vector3 sweepVector = velocity / relativeTimeScale * 0.05f - _up * (_hoverHeight + _hoverError); //0.05f is an arbitary constant for how long the time step this function considers is; it must be a constant to maintain predictability
        Vector3 castOrigin = transform.position + transform.rotation * _relativeCastOrigin;
        RaycastHit[] hits = Physics.RaycastAll(castOrigin, sweepVector, sweepVector.magnitude, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore);

        Vector3 prevNormal;
        if (_surfaceInfo == null)
        {
            prevNormal = -gravity;
        }
        else
        {
            prevNormal = _surfaceInfo.normal;
        }

        _surfaceInfo = null;
        _up = -gravity.normalized;

        foreach (RaycastHit hit in hits)
        {
            if (hit.transform == transform)
            {
                continue;
            }

            float normalChange = Vector3.Angle(hit.normal, prevNormal);
            if (normalChange <= _maxNormalChange)
            {
                _surfaceInfo = new RaycastHitInfoVerbose(hit, transform.forward);
                _up = Vector3.Lerp(_up, hit.normal, _groundAngleInfluence.Evaluate(normalChange)).normalized;
                break;
            }
        }

        base.LateSimulationUpdate(deltaTime);
    }

    protected abstract Vector3 GetTargetMove();
    protected abstract Vector3 GetTargetForward();
}