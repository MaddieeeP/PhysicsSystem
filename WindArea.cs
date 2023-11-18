using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Collider))]

public class WindArea : MonoBehaviour
{
    public Collider collider { get { return transform.GetComponent<Collider>(); } }
    [SerializeField] private float _magnitude = 1f;
    public float magnitude { get { return Math.Abs(_magnitude); } set { _magnitude = Math.Abs(value); } }

    public Vector3 GetWind(PhysicsObject obj)
    {
        return transform.forward * magnitude;
    }
}