using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;

//invert does not apply because the direction is set manually using the spline

public class GravityTwist : GravitationalField
{
    [SerializeField] protected SplineContainer splineContainer;
    [SerializeField] protected int _gravityDirectionCount;
    [SerializeField] protected float _sampleMaxDistance = 10f;
    [SerializeField] protected int _sampleResolution = 3;
    [SerializeField] protected int _sampleIterations = 5;

    public override Vector3 GetGravity(PhysicObject obj)
    {
        splineContainer.GetClosestPoint(obj.transform.position, out float t, out float distance, _sampleResolution, _sampleIterations);
        return GetGravityAtPoint(obj, 0, t, distance);
    }

    public override Vector3 GetGravityOnEnter(PhysicObject obj)
    {
        return GetGravity(obj);
    }

    public override Vector3 GetGravityOnExit(PhysicObject obj)
    {
        splineContainer.GetClosestEndPoint(obj.transform.position, out float t, out float distance);
        return GetGravityAtPoint(obj, 0, t, distance);
    }

    protected Vector3 GetGravityAtPoint(PhysicObject obj, int splineIndex, float t, float distance)
    {
        if (distance > _sampleMaxDistance || _sampleMaxDistance == 0f)
        {
            return default;
        }

        splineContainer.GetPoint(splineIndex, t, out Vector3 forward, out Vector3 down);
        down *= -1f;

        List<Quaternion> gravityDirections = Quaternion.LookRotation(down).GetRotationsAroundAxis(forward, _gravityDirectionCount);

        Vector3 objDown = obj.transform.up * -1f;
        if (obj.gravity != Vector3.zero)
        {
            objDown = obj.gravity;
        }

        Quaternion direction = Quaternion.LookRotation(objDown, obj.transform.forward).FindClosest(gravityDirections);
        
        return direction * Vector3.forward * magnitude * Math.Clamp(_fallOff.Evaluate(distance / _sampleMaxDistance), 0f, 1f);
    }
}
