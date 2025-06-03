using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Splines;

[DefaultExecutionOrder(110)]
public class RailSlider : BasicEntity
{
    [SerializeField] private Rail _rail;
    private float _t = default;
    private Vector3 _splinePosition = default;
    private Vector3 _splineTangent = default;

    public Rail rail { get { return _rail; } }

    protected override void ModifyTrueVelocities(ref Vector3 trueVelocity, ref Vector3 trueAngularVelocity, float deltaTime)
    {
        Vector3 velocityInTangentDirection = trueVelocity.ComponentAlongAxis(_splineTangent);
        float distance = velocityInTangentDirection.magnitude * deltaTime;
        float direction = Mathf.Sign(Vector3.Dot(velocityInTangentDirection, _splineTangent));
        if (velocityInTangentDirection == Vector3.zero)
        {
            direction = 0f;
        }
        _t += direction * distance / _rail.splineLength;
        float clampedT = Mathf.Clamp01(_t);
        MovePosition(_splinePosition);
        _rail.splineContainer.Spline.Evaluate(clampedT, out _splinePosition, out _splineTangent, out Vector3 splineNormal);
        trueVelocity = direction * velocityInTangentDirection.magnitude * _splineTangent;

        trueAngularVelocity = Mathf.Deg2Rad * Vector3.SignedAngle(transform.forward, _splineTangent, splineNormal) * splineNormal;

        if (_t != clampedT)
        {
            _t = clampedT;
            trueVelocity = Vector3.zero;
        }
    }
}