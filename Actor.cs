using System;
using UnityEngine;

[DefaultExecutionOrder(110)]
public abstract class Actor : BasicEntity
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
    private RaycastHitInfo _surfaceInfo;


    //getters and setters
    public bool grounded { get { return _surfaceInfo != null; } }
    public float moveMaxSpeed { get { return _moveMaxSpeed; } }
    public float moveMaxAcceleration { get { return _moveMaxAcceleration; } }
    public float moveMaxDeceleration { get { return _moveMaxDeceleration; } }
    public override Vector3 up { get { return _up; } }

    private Vector3 CalculateMoveForce(Vector3 targetMove, Vector3 flattenVector, float moveControl, float friction, float deltaTime)
    {
        Vector3 moveVector = targetMove.FlattenAgainstAxis(flattenVector) * _moveMaxSpeed;
        Vector3 predictedVelocityOnPlane = velocity.RemoveComponentAlongAxis(flattenVector);
        Vector3 predictedVelocityParallel = predictedVelocityOnPlane.ComponentAlongAxis(moveVector);
        Vector3 predictedVelocityPerpendicular = predictedVelocityOnPlane - predictedVelocityParallel;

        float simMoveMaxAcceleration = _moveMaxAcceleration * deltaTime * relativeTimeScale * SimulationController.globalTimeScale * friction;
        float simMoveMaxDeceleration = _moveMaxDeceleration * deltaTime * relativeTimeScale * SimulationController.globalTimeScale * friction;

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

    public override void SimulationUpdate(float deltaTime) //FIX - velocities should be relative sometimes...?
    {
        Vector3 targetMove = GetTargetMove();
        Vector3 targetForward = GetTargetForward();

        if (grounded)
        {
            float hoverError = _hoverHeight - _surfaceInfo.distance;
            float speedFromSurface = velocity.SignedMagnitudeInDirection(_surfaceInfo.normal);

            if (hoverError > 0f) //Actor is too close to the surface
            {
                float dampingForce = -speedFromSurface * _hoverDamp * Math.Min(_moveMaxSpeed, velocity.magnitude) / _moveMaxSpeed;
                AddForce((_hoverStrength * hoverError + dampingForce) * _surfaceInfo.normal, ForceMode.VelocityChange);
            }

            float friction = velocity.magnitude / relativeTimeScale > _minimumDynamicSpeed ? _surfaceInfo.collider.material.staticFriction : _surfaceInfo.collider.material.dynamicFriction;
            AddForce(CalculateMoveForce(targetMove, _surfaceInfo.normal, _moveControlGrounded, friction, deltaTime), ForceMode.VelocityChange);

            transform.rotation.ShortestRotation(Quaternion.LookRotation(targetForward.RemoveComponentAlongAxis(_up), _up)).ToAngleAxis(out float rotDegrees, out Vector3 rotAxis);
            AddTorque(_orientStrengthGrounded * rotDegrees * rotAxis - _orientDamp * SimulationController.globalTimeScale * relativeTimeScale * angularVelocity, ForceMode.VelocityChange);
        }
        else
        {
            AddForce(CalculateMoveForce(targetMove, gravity, _moveControlAerial, _moveStrengthAerial, deltaTime), ForceMode.VelocityChange);

            transform.rotation.ShortestRotation(Quaternion.LookRotation(targetForward.RemoveComponentAlongAxis(_up), _up)).ToAngleAxis(out float rotDegrees, out Vector3 rotAxis);
            AddTorque(_orientStrengthAerial * rotDegrees * rotAxis - _orientDamp * SimulationController.globalTimeScale * relativeTimeScale * angularVelocity, ForceMode.VelocityChange);
        }

        base.SimulationUpdate(deltaTime);
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