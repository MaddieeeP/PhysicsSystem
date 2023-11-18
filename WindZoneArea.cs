using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(WindZone))]

public class WindZoneArea : WindArea
{
    public Collider collider { get { return transform.GetComponent<Collider>(); } }
    public WindZone windZone { get { return transform.GetComponent<WindZone>(); } }
    new private float _magnitude = 1f;
    new public float magnitude { get { return _magnitude * windZone.windMain; } }

    public Vector3 GetWind(PhysicsObject obj)
    {
        return transform.forward * magnitude;
    }
}