using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(WindZone))]

public class WindZoneArea : WindArea
{
    public WindZone windZone { get { return transform.GetComponent<WindZone>(); } }
    [SerializeField] protected new float _magnitude = 1f;
    public new float magnitude { get { return _magnitude * windZone.windMain; } }

    public new Vector3 GetWind(PhysicObject obj)
    {
        return transform.forward * magnitude;
    }
}