using System;
using UnityEngine;

public abstract class ForceField : MonoBehaviour
{
    [SerializeField] protected bool _isGravityForce = false;
    [SerializeField] protected float _magnitude = 9.8f;
    [SerializeField] protected AnimationCurve _fallOff = new AnimationCurve(new Keyframe(0, 1), new Keyframe(1, 1));
    [SerializeField] protected ForceMode forceMode = ForceMode.Force;

    //getters and setters
    public bool isGravityForce { get { return _isGravityForce; } }
    public float magnitude { get { return Math.Abs(_magnitude); } }

    public abstract Vector3 GetForce(PhysicObject physicObject);

    public void Tick(Collider[] colliders)
    {
        foreach (Collider collider in colliders)
        {
            PhysicObject physicObject = collider.transform.GetComponent<PhysicObject>();
            if (physicObject != null)
            {
                if (_isGravityForce)
                {
                    physicObject.AddGravityForce(GetForce(physicObject));
                    continue;
                }
                physicObject.AddForce(GetForce(physicObject), forceMode);
            }
        }
    }
}