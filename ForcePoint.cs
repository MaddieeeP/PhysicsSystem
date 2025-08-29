using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ForcePoint : ForceField
{
    [SerializeField] protected float _radius = 1f;

    protected override List<IEntity> GetCollidingEntities()
    {
        Collider[] colliders = Physics.OverlapSphere(transform.position, _radius);
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
        Vector3 positionDifference = transform.position - entity.transform.position;
        return positionDifference.normalized * _signedMagnitude;
    }
}
