using System;
using System.Collections.Generic;
using UnityEngine;

public abstract class ForceField : SimulationObject
{
    [SerializeField] protected bool _isGravityForce = false;
    [SerializeField] protected float _magnitude = 9.8f;
    [SerializeField] protected AnimationCurve _fallOff = new AnimationCurve(new Keyframe(0, 1), new Keyframe(1, 1));
    [SerializeField] protected ForceMode forceMode = ForceMode.Force;

    //getters and setters
    public bool isGravityForce { get { return _isGravityForce; } }
    public float magnitude { get { return Math.Abs(_magnitude); } }

    protected abstract List<IEntity> GetCollidingEntities();
    public abstract Vector3 GetForce(IEntity IEntity);

    public override void SimulationUpdate(float deltaTime)
    {
        List<IEntity> entities = GetCollidingEntities();
        foreach (IEntity IEntity in entities)
        {
            if (_isGravityForce)
            {
                IEntity.AddGravityForce(GetForce(IEntity));
                continue;
            }
            IEntity.AddForce(GetForce(IEntity), forceMode);
        }
    }
}