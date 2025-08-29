using System;
using UnityEngine;

[System.Serializable]
public class ActorRailGrind : ActorState
{
    [SerializeField] private float _testDistance = 0.5f;
    [SerializeField] private float _breakSpeed = 1f;
    [SerializeField] private Jump _jump = new Jump();

    private Rail _rail;
    private float _t = default;
    private Vector3 _splinePosition = default;
    private Vector3 _splineTangent = default;

    //getters and setters
    public override string stateID { get { return "railGrind"; } }
    public Jump jump { get { return _jump; } }

    public override bool TryStart()
    {
        Rail[] worldRails = UnityEngine.Object.FindObjectsByType<Rail>(FindObjectsSortMode.None); //FIX
        Vector3 testPosition = _actor.basePosition;
        foreach (Rail worldRail in worldRails)
        {
            if (Vector3.Distance(worldRail.bounds.ClosestPoint(testPosition), testPosition) <= _testDistance)
            {
                Vector3 closestPoint = worldRail.GetNearestPointInWorldSpace(testPosition, out float t, out float distance);
                if (distance > _testDistance)
                {
                    continue;
                }
                worldRail.EvaluateInWorldSpace(_t, out _, out Vector3 closestTangent, out _);
                Vector3 breakingVelocity = _actor.linearVelocity.RemoveComponentAlongAxis(_splineTangent);
                if (!breakingVelocity.IsComponentInDirectionPositive(closestPoint - testPosition))
                {
                    continue;
                }

                _rail = worldRail;
                _t = t;
                _splinePosition = closestPoint;
                _splineTangent = closestTangent;
                return true;
            }
        }

        return false;
    }

    public override void SimulationUpdate(ref Vector3 actualLinearVelocity, ref Vector3 actualAngularVelocity)
    {
        Vector3 velocityParallel = actualLinearVelocity.ComponentAlongAxis(_splineTangent);
        Vector3 velocityPerpendicular = actualLinearVelocity - velocityParallel;

        float distance = velocityParallel.magnitude * _actor.simulationController.simulationDeltaTime;
        float direction = Mathf.Sign(Vector3.Dot(velocityParallel, _splineTangent));
        if (velocityParallel == Vector3.zero)
        {
            direction = 0f;
        }

        _t += direction * distance / _rail.splineLength;
        float clampedT = Mathf.Clamp01(_t);
        _actor.MoveBasePosition(_splinePosition);
        _rail.EvaluateInWorldSpace(clampedT, out _splinePosition, out _splineTangent, out Vector3 splineNormal);
        
        Quaternion rotationError = _actor.transform.rotation.ShortestRotation(Quaternion.LookRotation(_splineTangent, _actor.transform.up));
        rotationError.ToAngleAxis(out float rotDegrees, out Vector3 rotAxis);

        actualLinearVelocity = direction * velocityParallel.magnitude * _splineTangent;
        actualAngularVelocity = Mathf.Deg2Rad * rotDegrees * rotAxis; //FIX - damping + consistent direction (negation of tangent)

        if (!velocityPerpendicular.IsComponentInDirectionPositive(_actor.gravity))
        {
            actualLinearVelocity += velocityPerpendicular;
        }

        if (_t != clampedT)
        {
            _t = -1f;
        }
    }

    public override void LateSimulationUpdate(ref Vector3 actualLinearVelocity, ref Vector3 actualAngularVelocity)
    {
        if (_t == -1f)
        {
            _actor.EndCurrentState();
        }

        Vector3 breakingVelocity = actualLinearVelocity.RemoveComponentAlongAxis(_splineTangent);
        if (breakingVelocity.magnitude > _breakSpeed)
        {
            _actor.EndCurrentState();
        }
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
            _actor.AddForce(_jumpSpeed * (0.5f + Math.Clamp(duration, 0f, 1f) * 0.5f) * _actor.transform.up, ForceMode.VelocityChange);
        }
    }
}