using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Collider))]

public abstract class GravitationalField : MonoBehaviour
{
    [SerializeField] private bool _invert = false;
    [SerializeField] protected bool _applyAfterExit = false;
    [SerializeField] protected float _magnitude = 9.8f;
    [SerializeField] protected AnimationCurve _fallOff = new AnimationCurve(new Keyframe(0, 1), new Keyframe(1, 1));

    protected int _invertMultiplier { get { return _invert ? -1 : 1; } }

    public bool applyAfterExit { get { return _applyAfterExit; } }
    public float magnitude { get { return Math.Abs(_magnitude); } set { _magnitude = Math.Abs(value); } }

    public Collider collider { get { return transform.GetComponent<Collider>(); } }

    public abstract Vector3 GetGravity(PhysicsObject obj);
    public abstract Vector3 GetGravityOnEnter(PhysicsObject obj);
    public abstract Vector3 GetGravityOnExit(PhysicsObject obj);
} 