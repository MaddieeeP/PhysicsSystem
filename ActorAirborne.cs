using System;
using UnityEngine;

[System.Serializable]
public class ActorAirborne : ActorState
{
    [SerializeField] private float _maxSpeed = 10f; //Must be positive
    [SerializeField] private float _maxAcceleration = 20f; //Must be positive
    [SerializeField] private float _maxDeceleration = 40f; //Must be positive and not less than _maxAcceleration
    [SerializeField] private float _moveAerialFriction = 0.2f;
    [SerializeField] private float _orientStrength = 0.8f;
    [SerializeField] private float _orientDamp = 0.95f;
    [SerializeField] private Jump _jump = new Jump();

    private Vector3 _move;

    //Getters and setters
    public override string stateID { get { return "airborne"; } }
    public Vector3 move { get { return _move; } set { _move = value; } }
    public Jump jump { get { return _jump; } }

    public override bool TryStart()
    {
        _jump.ResetRemainingJumps();
        return true;
    }

    public override void SimulationUpdate(ref Vector3 actualLinearVelocity, ref Vector3 actualAngularVelocity)
    {
        Vector3 actorUp = -_actor.gravity.normalized;
        Vector3 planarVelocityDirection = _actor.linearVelocity.RemoveComponentAlongAxis(actorUp).normalized;
        Vector3 targetForward = planarVelocityDirection != default ? planarVelocityDirection : _actor.transform.forward.RemoveComponentAlongAxis(actorUp).normalized;

        _actor.transform.rotation.ShortestRotation(Quaternion.LookRotation(targetForward, actorUp)).ToAngleAxis(out float rotDegrees, out Vector3 rotAxis);
        actualAngularVelocity += _orientStrength * rotDegrees * rotAxis - _orientDamp * actualAngularVelocity;

        Vector3 moveVector = _move.FlattenAgainstAxis(actorUp) * _maxSpeed;
        Vector3 predictedVelocityAlongSurfacePlane = actualLinearVelocity.RemoveComponentAlongAxis(actorUp);
        Vector3 predictedVelocityParallel = predictedVelocityAlongSurfacePlane.ComponentAlongAxis(moveVector);
        Vector3 predictedVelocityPerpendicular = predictedVelocityAlongSurfacePlane - predictedVelocityParallel;

        float stepMaxAccelerationVelocityChange = _maxAcceleration * _actor.simulationController.simulationDeltaTime * _moveAerialFriction;
        float stepMaxDecelerationVelocityChange = _maxDeceleration * _actor.simulationController.simulationDeltaTime * _moveAerialFriction;

        actualLinearVelocity += Vector3.ClampMagnitude(Vector3.ClampMagnitude(predictedVelocityAlongSurfacePlane + moveVector, _maxSpeed) - predictedVelocityAlongSurfacePlane, stepMaxAccelerationVelocityChange);
    }

    [System.Serializable]
    public class Jump : ActorAction
    {
        [SerializeField] private float _jumpSpeed = 20f;
        [SerializeField] private int _maxJumps = 1;
        [SerializeField] private int _remainingJumps = 0;

        private float _startTime = -1f;
        public override void Start()
        {
            _startTime = Time.time;
        }

        public override void Cancel()
        {
            if (_remainingJumps < 1 || _startTime == -1f)
            {
                return;
            }

            float duration = Time.time - _startTime;
            _remainingJumps--;
            _startTime = -1f;
            _actor.AddForce(_jumpSpeed * (0.5f + Math.Clamp(duration, 0f, 1f) * 0.5f) * -_actor.gravity, ForceMode.VelocityChange);
        }

        public void ResetRemainingJumps()
        {
            _remainingJumps = _maxJumps;
        }
    }
}