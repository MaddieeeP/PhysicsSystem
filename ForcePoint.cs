using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ForcePoint : ForceField
{
    [SerializeField] protected float _radius = 1f;
    [SerializeField] protected bool _repel = false;
    protected int _repelMultiplier { get { return _repel ? -1 : 1; } }

    protected override List<Entity> GetCollidingEntities()
    {
        Collider[] colliders = Physics.OverlapSphere(transform.position, _radius);
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
        Vector3 positionDifference = transform.position - entity.transform.position;
        float falloffMultiplier = Math.Clamp(_fallOff.Evaluate(positionDifference.magnitude / _radius), 0f, 1f);
        return positionDifference.normalized * magnitude * falloffMultiplier * _repelMultiplier;
    }
}
