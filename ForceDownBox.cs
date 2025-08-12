using System.Collections.Generic;
using UnityEngine;

public class ForceDownBox : ForceField
{
    [SerializeField] protected Vector3 _halfExtents;

    protected override List<IEntity> GetCollidingEntities()
    {
        Collider[] colliders = Physics.OverlapBox(transform.position, _halfExtents, transform.rotation);
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
        float falloffMultiplier = 1f; //FIX - use relative distance along relative y axis
        return transform.up * -magnitude * falloffMultiplier;
    }
}