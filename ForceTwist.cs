using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;

public class ForceTwist : ForceField
{
    [SerializeField] protected Vector3 _overlapSphereCenter;
    [SerializeField] protected float _radius;
    [SerializeField] protected float _maxClosestPointInaccuracy = 0.05f;
    [SerializeField] protected SplineContainer splineContainer;
    [SerializeField] protected int _gravityDirectionCount;
    [SerializeField] protected float _sampleMaxDistance = 10f;
    [SerializeField] protected int _sampleResolution = 3;
    [SerializeField] protected int _sampleIterations = 5;

    void FixedUpdate()
    {
        Collider[] colliders = Physics.OverlapSphere(_overlapSphereCenter, _radius);
        Tick(colliders);
    }

    public override Vector3 GetForce(PhysicObject physicObject)
    {
        splineContainer.GetClosestPoint(physicObject.transform.position, out float t, out float distance, _sampleResolution, _sampleIterations);

        if (distance > _sampleMaxDistance)
        {
            return default;
        }

        Vector3 splinePoint = splineContainer.GetPoint(0, t, out Vector3 forward, out Vector3 up);
        Vector3 positionDifference = splinePoint - physicObject.transform.position;

        if (positionDifference.ComponentAlongAxis(forward).magnitude > _maxClosestPointInaccuracy)
        {
            return default;
        }

        List<Quaternion> gravityDirections = Quaternion.LookRotation(-up).GetRotationsAroundAxis(forward, _gravityDirectionCount);

        Vector3 physicObjectDown = (physicObject.gravity == Vector3.zero) ? -physicObject.transform.up : physicObject.gravity;
        Quaternion direction = Quaternion.LookRotation(physicObjectDown, physicObject.transform.forward).FindClosest(gravityDirections);

        return direction * Vector3.forward * magnitude * Math.Clamp(_fallOff.Evaluate(distance / _sampleMaxDistance), 0f, 1f);
    }
}
