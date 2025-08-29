using System;
using System.Collections.Generic;
using UnityEngine;

public abstract class ForceField : SimulationObject
{
    [SerializeField] protected float _signedMagnitude = -9.8f;
    [SerializeField] protected int _gravityPriority = 0; //0 means not a gravity field
    [SerializeField] protected ForceMode forceMode = ForceMode.Force;

    protected abstract List<IEntity> GetCollidingEntities();
    public abstract Vector3 GetForce(IEntity IEntity);

    public override void PreSimulationUpdate()
    {
        List<IEntity> entities = GetCollidingEntities();
        foreach (IEntity IEntity in entities)
        {
            if (_gravityPriority == 0)
            {
                IEntity.AddForce(GetForce(IEntity), forceMode);
                continue;
            }
            IEntity.TrySetGravity(GetForce(IEntity), _gravityPriority);
        }
    }
}