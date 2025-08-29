using System;
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
        Vector3 splinePoint = splineContainer.GetNearestPointInWorldSpace(entity.transform.position, out float t, out float distance, _sampleResolution, _sampleIterations);
        splineContainer.EvaluateInWorldSpace(t, out Vector3 position, out Vector3 forward, out Vector3 up);

        if (distance > _sampleMaxDistance) 
        { 
            return default;
        }

        Vector3 positionDifference = splinePoint - entity.transform.position;

        if (positionDifference.ComponentAlongAxis(forward).magnitude > _maxClosestPointInaccuracy)
        {
            return default;
        }

        return positionDifference.normalized * _signedMagnitude;
    }
}