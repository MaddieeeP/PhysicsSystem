using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GravityFlat : GravitationalField
{
    public override Vector3 GetGravity(PhysicsObject obj)
    {
        return transform.up * -1f * magnitude;
    }

    public override Vector3 GetGravityOnEnter(PhysicsObject obj)
    {
        return GetGravity(obj);
    }

    public override Vector3 GetGravityOnExit(PhysicsObject obj)
    {
        return GetGravity(obj);
    }
}
