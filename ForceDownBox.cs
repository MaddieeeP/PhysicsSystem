using System.Collections.Generic;
using UnityEngine;

public class ForceDownBox : ForceField
{
    [SerializeField] protected Vector3 _halfExtents;

    protected override List<Entity> GetCollidingEntities()
    {
        Collider[] colliders = Physics.OverlapBox(transform.position, _halfExtents, transform.rotation);
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
        float falloffMultiplier = 1f; //FIX - use relative distance along relative y axis
        return transform.up * -magnitude * falloffMultiplier;
    }
}