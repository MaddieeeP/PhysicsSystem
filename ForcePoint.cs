using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ForcePoint : ForceField
{
    [SerializeField] protected float _radius = 1f;
    [SerializeField] protected bool _repel = false;
    protected int _repelMultiplier { get { return _repel ? -1 : 1; } }

    void FixedUpdate()
    {
        Collider[] colliders = Physics.OverlapSphere(transform.position, _radius);
        Tick(colliders);
    }

    public override Vector3 GetForce(PhysicObject physicObject)
    {
        Vector3 positionDifference = transform.position - physicObject.transform.position;
        float falloffMultiplier = Math.Clamp(_fallOff.Evaluate(positionDifference.magnitude / _radius), 0f, 1f);
        return positionDifference.normalized * magnitude * falloffMultiplier * _repelMultiplier;
    }
}
