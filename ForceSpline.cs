using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;

public class ForceSpline : ForceField
{
    [SerializeField] protected Vector3 _overlapSphereCenter;
    [SerializeField] protected float _radius;
    [SerializeField] protected float _maxClosestPointInaccuracy = 0.05f;
    [SerializeField] protected SplineContainer splineContainer;
    [SerializeField] protected float _sampleMaxDistance = float.PositiveInfinity;
    [SerializeField] protected int _sampleResolution = 4;
    [SerializeField] protected int _sampleIterations = 2;
    [SerializeField] protected bool _repel = true;

    protected int _repelMultiplier { get { return _repel ? -1 : 1; } }

    void FixedUpdate()
    {
        Collider[] colliders = Physics.OverlapSphere(_overlapSphereCenter, _radius);
        Tick(colliders);
    }

    public override Vector3 GetForce(Entity entity)
    {
        Vector3 splinePoint = splineContainer.GetClosestPoint(entity.transform.position, out float t, out float distance, _sampleResolution, _sampleIterations);
        splineContainer.Spline.Evaluate(t, out Vector3 position, out Vector3 forward, out Vector3 up);

        if (distance > _sampleMaxDistance) 
        { 
            return default;
        }

        Vector3 positionDifference = splinePoint - entity.transform.position;

        if (positionDifference.ComponentAlongAxis(forward).magnitude > _maxClosestPointInaccuracy)
        {
            return default;
        }

        return positionDifference.normalized * magnitude * Math.Clamp(_fallOff.Evaluate(distance / _sampleMaxDistance), 0f, 1f) * _repelMultiplier;
    }
}