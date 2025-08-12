using System;
using UnityEngine;

[System.Serializable]
public class ActorGrounded : ActorState
{
    [SerializeField] private float _minimumDynamicSpeed = 0.1f; //Essentially equal to maximum static speed
    [SerializeField] private float _maxSpeed = 10f; //Must be positive
    [SerializeField] private float _maxAcceleration = 20f; //Must be positive
    [SerializeField] private float _maxDeceleration = 40f; //Must be positive and not less than _maxAcceleration
    [SerializeField] private float _hoverError = 0.1f;
    [SerializeField] private int _maxNormalChange = 50;
    [SerializeField] private float _orientStrength = 0.9f;
    [SerializeField] private float _orientDamp = 0.95f;
    [SerializeField] private AnimationCurve _groundAngleInfluence;
    [SerializeField] private Jump _jump = new Jump();

    private Vector3 _up;
    private Vector3 _move;
    private RaycastHitInfoVerbose _surfaceInfo;

    //Getters and setters
    public override string stateID { get { return "grounded"; } }
    public override Vector3 up { get { return _up; } }
    public Vector3 move { get { return _move; } set { _move = value; } }
    public Jump jump { get { return _jump; } }

    public override bool TryStart()
    {
        Vector3 sweepVector = _actor.linearVelocity / _actor.relativeTimeScale * 0.05f - _up * (_actor.baseHeight + _hoverError); //0.05f is an arbitary constant for how long the time step this function considers is; it must be a constant to maintain predictability
        RaycastHit[] hits = Physics.RaycastAll(_actor.basePosition, sweepVector, sweepVector.magnitude, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore);

        Vector3 prevNormal = -_actor.gravity;
        _surfaceInfo = null;
        _up = -_actor.gravity.normalized;

        foreach (RaycastHit hit in hits)
        {
            if (hit.transform == _actor.transform)
            {
                continue;
            }

            float normalChange = Vector3.Angle(hit.normal, prevNormal);
            if (normalChange <= _maxNormalChange)
            {
                _surfaceInfo = new RaycastHitInfoVerbose(hit, _actor.transform.forward);
                _up = Vector3.Lerp(_up, hit.normal, _groundAngleInfluence.Evaluate(normalChange)).normalized;
                return true;
            }
        }

        return false;
    }

    public override void SimulationUpdate(ref Vector3 actualLinearVelocity, ref Vector3 actualAngularVelocity, float deltaTime)
    {
        float scaledDeltaTime = deltaTime * _actor.relativeTimeScale * SimulationController.globalTimeScale;

        Vector3 targetForward = (_actor.linearVelocity * _actor.relativeTimeScale * SimulationController.globalTimeScale).magnitude > _minimumDynamicSpeed ? _actor.linearVelocity.normalized : _actor.transform.forward;

        float hoverError;
        float speedFromSurface;
        Vector3 surfaceVelocity = default;
        Vector3 normal;

        IEntity surfaceEntity = _surfaceInfo.transform.GetComponent<IEntity>();
        if (surfaceEntity != null)
        {
            RaycastHitInfoVerbose adjustedSurfaceInfo = surfaceEntity.GetPredictedTransformedSurfaceInfo(_surfaceInfo, deltaTime);

            Vector3 surfaceTrueVelocity = surfaceEntity.linearVelocity * surfaceEntity.relativeTimeScale * SimulationController.globalTimeScale;
            Vector3 surfaceNextTrueVelocity = (adjustedSurfaceInfo.point - _surfaceInfo.point) / deltaTime;

            actualLinearVelocity += (surfaceNextTrueVelocity - surfaceTrueVelocity).RemoveComponentAlongAxis(adjustedSurfaceInfo.normal) * surfaceEntity.collider.material.staticFriction; //Apply surface acceleration

            hoverError = _actor.baseHeight - (_actor.basePosition - adjustedSurfaceInfo.point).ComponentAlongAxis(_up).magnitude;
            speedFromSurface = (actualLinearVelocity - surfaceNextTrueVelocity).SignedMagnitudeInDirection(adjustedSurfaceInfo.normal);
            normal = adjustedSurfaceInfo.normal;
            surfaceVelocity = surfaceNextTrueVelocity;
        }
        else //Static surface
        {
            hoverError = _actor.baseHeight - _surfaceInfo.distance;
            speedFromSurface = actualLinearVelocity.SignedMagnitudeInDirection(_surfaceInfo.normal);
            normal = _surfaceInfo.normal;
        }

        if (hoverError > 0f) //Actor is too close to the surface
        {
            float dampingForce = Math.Max(-speedFromSurface, 0f);
            actualLinearVelocity += (hoverError + dampingForce) * _surfaceInfo.normal;
        }

        _actor.transform.rotation.ShortestRotation(Quaternion.LookRotation(targetForward.RemoveComponentAlongAxis(_up), _up)).ToAngleAxis(out float rotDegrees, out Vector3 rotAxis);
        actualAngularVelocity += _orientStrength * rotDegrees * rotAxis - _orientDamp * SimulationController.globalTimeScale * _actor.relativeTimeScale * actualAngularVelocity;

        Vector3 moveVector = _move.FlattenAgainstAxis(normal) * _maxSpeed;
        Vector3 predictedVelocityAlongSurfacePlane = (actualLinearVelocity - surfaceVelocity).RemoveComponentAlongAxis(normal);
        Vector3 predictedVelocityParallel = predictedVelocityAlongSurfacePlane.ComponentAlongAxis(moveVector);
        Vector3 predictedVelocityPerpendicular = predictedVelocityAlongSurfacePlane - predictedVelocityParallel;

        float friction = actualLinearVelocity.magnitude / _actor.relativeTimeScale > _minimumDynamicSpeed ? _surfaceInfo.collider.material.staticFriction : _surfaceInfo.collider.material.dynamicFriction;
        float stepMaxAccelerationVelocityChange = _maxAcceleration * scaledDeltaTime * friction;
        float stepMaxDecelerationVelocityChange = _maxDeceleration * scaledDeltaTime * friction;

        if (predictedVelocityAlongSurfacePlane.SignedMagnitudeInDirection(moveVector) >= 0f) //accelerating in moveDirection
        {
            actualLinearVelocity += Vector3.ClampMagnitude(Vector3.ClampMagnitude(moveVector - predictedVelocityParallel, stepMaxAccelerationVelocityChange) - predictedVelocityPerpendicular, stepMaxDecelerationVelocityChange);
        }
        else
        {
            actualLinearVelocity += Vector3.ClampMagnitude(Vector3.ClampMagnitude(moveVector + predictedVelocityParallel, stepMaxAccelerationVelocityChange) - Vector3.ClampMagnitude(predictedVelocityParallel, stepMaxDecelerationVelocityChange) - predictedVelocityPerpendicular, stepMaxDecelerationVelocityChange);
        }
    }

    public override void LateSimulationUpdate(ref Vector3 actualLinearVelocity, ref Vector3 actualAngularVelocity, float deltaTime)
    {
        Vector3 sweepVector = actualLinearVelocity / _actor.relativeTimeScale * 0.05f - _up * (_actor.baseHeight + _hoverError); //0.05f is an arbitary constant for how long the time step this function considers is; it must be a constant to maintain predictability
        RaycastHit[] hits = Physics.RaycastAll(_actor.basePosition, sweepVector, sweepVector.magnitude, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore);

        Vector3 prevNormal;
        if (_surfaceInfo == null)
        {
            prevNormal = -_actor.gravity;
        }
        else
        {
            prevNormal = _surfaceInfo.normal;
        }

        _surfaceInfo = null;
        _up = -_actor.gravity.normalized;

        foreach (RaycastHit hit in hits)
        {
            if (hit.transform == _actor.transform)
            {
                continue;
            }

            float normalChange = Vector3.Angle(hit.normal, prevNormal);
            if (normalChange <= _maxNormalChange)
            {
                _surfaceInfo = new RaycastHitInfoVerbose(hit, _actor.transform.forward);
                _up = Vector3.Lerp(_up, hit.normal, _groundAngleInfluence.Evaluate(normalChange)).normalized;
                return;
            }
        }

        _actor.EndCurrentState();
    }

    public void SetTargetMovement(Vector3 targetMovement)
    {
        _move = targetMovement;
    }

    [System.Serializable]
    public class Jump : ActorAction
    {
        [SerializeField] private float _jumpSpeed = 20f;

        private float _startTime = -1f;
        
        public override void Start()
        {
            _startTime = Time.time;
        }

        public override void Cancel()
        {
            if (_startTime == -1f)
            {
                return;
            }

            float duration = Time.time - _startTime;
            _startTime = -1f;
            _actor.AddForce(_jumpSpeed * (0.5f + Math.Clamp(duration, 0f, 1f) * 0.5f) * _actor.up, ForceMode.VelocityChange);
        }
    }
}