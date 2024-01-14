using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GravityFlat : GravitationalField
{
    public override Vector3 GetGravity(PhysicObject obj)
    {
        return transform.up * -1f * magnitude;
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
