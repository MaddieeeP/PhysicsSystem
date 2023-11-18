using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Collider))]

public class GravitationalField : MonoBehaviour
{
    [SerializeField] private bool _overwrite;
    public bool overwrite { get { return _overwrite; } }

    [SerializeField] private float _magnitude = 9.8f;
    public float magnitude { get { return Math.Abs(_magnitude); } set { _magnitude = Math.Abs(value); } }

    public Collider collider { get { return transform.GetComponent<Collider>(); } }

    public Vector3 GetGravity(PhysicsObject obj) 
    { 
        return transform.forward * magnitude;
    }
}