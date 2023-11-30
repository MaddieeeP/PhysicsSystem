using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Collider))]

public class GravitationalField : MonoBehaviour
{
    [SerializeField] private bool _applyAfterExit = false;
    public bool applyAfterExit { get { return _applyAfterExit; } }

    [SerializeField] private float _magnitude = 9.8f;
    public float magnitude { get { return Math.Abs(_magnitude); } set { _magnitude = Math.Abs(value); } }

    public Collider collider { get { return transform.GetComponent<Collider>(); } }

    public Vector3 GetGravity(PhysicsObject obj) 
    {
        return transform.up * -1f * magnitude;
    }

    public Vector3 GetGravityOnEnter(PhysicsObject obj)
    {
        return GetGravity(obj);
    }

    public Vector3 GetGravityOnExit(PhysicsObject obj)
    {
        return GetGravity(obj);
    }
}