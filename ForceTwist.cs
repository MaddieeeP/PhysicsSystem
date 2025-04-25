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

    protected override List<Entity> GetCollidingEntities()
    {
        Collider[] colliders = Physics.OverlapSphere(_overlapSphereCenter, _radius);
        List<Entity> entities = new List<Entity>();
        foreach (Collider collider in colliders)
        {
            Entity entity = collider.GetComponent<Entity>();
            if (entity != null)
            {
                entities.Add(entity);
            }
        }
        return entities;
    }

    public override Vector3 GetForce(Entity entity)
    {
        splineContainer.GetClosestPoint(entity.transform.position, out float t, out float distance, _sampleResolution, _sampleIterations);

        if (distance > _sampleMaxDistance)
        {
            return default;
        }

        splineContainer.Spline.Evaluate(t, out Vector3 splinePoint, out Vector3 forward, out Vector3 up);
        Vector3 positionDifference = splinePoint - entity.transform.position;

        if (positionDifference.ComponentAlongAxis(forward).magnitude > _maxClosestPointInaccuracy)
        {
            return default;
        }

        List<Quaternion> gravityDirections = Quaternion.LookRotation(-up).GetRotationsAroundAxis(forward, _gravityDirectionCount);

        Vector3 entityDown = (entity.gravity == Vector3.zero) ? -entity.transform.up : entity.gravity;
        Quaternion direction = Quaternion.LookRotation(entityDown, entity.transform.forward).FindClosest(gravityDirections);

        return direction * Vector3.forward * magnitude * Math.Clamp(_fallOff.Evaluate(distance / _sampleMaxDistance), 0f, 1f);
    }
}
