using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;

public class GravitySpline : GravitationalField
{
    [SerializeField] protected SplineContainer splineContainer;
    [SerializeField] protected float _sampleMaxDistance = 10f;
    [SerializeField] protected int _sampleResolution = 4;
    [SerializeField] protected int _sampleIterations = 2;

    public override Vector3 GetGravity(PhysicsObject obj)
    {
        Vector3 splinePoint = splineContainer.GetClosestPoint(obj.transform.position, out float t, out float distance, _sampleResolution, _sampleIterations);
        return GetGravityAtPoint(obj, splinePoint, distance);
    }

    public override Vector3 GetGravityOnEnter(PhysicsObject obj)
    {
        return GetGravity(obj);
    }

    public override Vector3 GetGravityOnExit(PhysicsObject obj)
    {
        return GetGravity(obj);
    }

    protected Vector3 GetGravityAtPoint(PhysicsObject obj, Vector3 splinePoint, float distance)
    {
        if (distance > _sampleMaxDistance) 
        { 
            return default;
        }

        Vector3 positionDifference = splinePoint - obj.transform.position;
        return positionDifference.normalized * magnitude * Math.Clamp(_fallOff.Evaluate(distance / _sampleMaxDistance), 0f, 1f) * _invertMultiplier;
    }
}