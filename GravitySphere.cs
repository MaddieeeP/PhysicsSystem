using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(SphereCollider))]

public class GravitySphere : GravitationalField
{
    public new SphereCollider collider { get { return transform.GetComponent<SphereCollider>(); } }

    public override Vector3 GetGravity(PhysicObject obj)
    {
        Vector3 positionDifference = transform.TransformPoint(collider.center) - obj.transform.position;
        return positionDifference.normalized * magnitude * Math.Clamp(_fallOff.Evaluate(positionDifference.magnitude / collider.radius), 0f, 1f) * _invertMultiplier;
    }

    public override Vector3 GetGravityOnEnter(PhysicObject obj)
    {
        return GetGravity(obj);
    }

    public override Vector3 GetGravityOnExit(PhysicObject obj)
    {
        return GetGravity(obj);
    }
}
