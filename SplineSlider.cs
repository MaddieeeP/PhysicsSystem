using UnityEngine;
using UnityEngine.Splines;

[DefaultExecutionOrder(110)]
public class SplineSlider : BasicEntity
{
    [SerializeField] private SplineContainer _splineContainer;
    private Vector3 _lastSplinePosition = default;
    private Vector3 _lastSplineForward = default;
    private Vector3 _lastSplineUp = default;

    public SplineContainer splineContainer { set { _splineContainer = value; } }

    protected override void ModifyTrueVelocities(ref Vector3 trueVelocity, ref Vector3 trueAngularVelocity, float deltaTime)
    {
        _splineContainer.GetClosestPoint(transform.position + trueVelocity * deltaTime, out float t, out float distance, 10, 4);
        _splineContainer.Spline.Evaluate(t, out _lastSplinePosition, out _lastSplineForward, out _lastSplineUp);
        trueVelocity = (_lastSplinePosition - transform.position) / deltaTime;
        //angularVelocity = up * Vector3.SignedAngle(transform.forward, nextForward, up) * Mathf.Deg2Rad;
    }
}