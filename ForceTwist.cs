using System;
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

    protected override List<IEntity> GetCollidingEntities()
    {
        Collider[] colliders = Physics.OverlapSphere(_overlapSphereCenter, _radius);
        List<IEntity> entities = new List<IEntity>();
        foreach (Collider collider in colliders)
        {
            IEntity entity = collider.GetComponent<IEntity>();
            if (entity != null)
            {
                entities.Add(entity);
            }
        }
        return entities;
    }

    public override Vector3 GetForce(IEntity entity)
    {
        splineContainer.GetNearestPointInWorldSpace(entity.transform.position, out float t, out float distance, _sampleResolution, _sampleIterations);

        if (distance > _sampleMaxDistance)
        {
            return default;
        }

        splineContainer.EvaluateInWorldSpace(t, out Vector3 splinePoint, out Vector3 forward, out Vector3 up);
        Vector3 positionDifference = splinePoint - entity.transform.position;

        if (positionDifference.ComponentAlongAxis(forward).magnitude > _maxClosestPointInaccuracy)
        {
            return default;
        }

        List<Quaternion> gravityDirections = Quaternion.LookRotation(-up).GetRotationsAroundAxis(forward, _gravityDirectionCount);

        Vector3 entityDown = (entity.gravity == Vector3.zero) ? -entity.transform.up : entity.gravity;
        Quaternion direction = Quaternion.LookRotation(entityDown, entity.transform.forward).FindClosest(gravityDirections);

        return direction * Vector3.forward * _signedMagnitude;
    }
}
