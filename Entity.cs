using UnityEngine;

public abstract class Entity : SimulationObject
{
    public abstract bool isKinematic { get; set; }
    public abstract Collider collider { get; }
    public abstract Vector3 gravity { get; }
    public virtual Vector3 up { get { return -gravity; } }

    public abstract void AddGravityForce(Vector3 force);
    public abstract void AddForce(Vector3 force, ForceMode forceMode = ForceMode.Force);
    public abstract void AddTorque(Vector3 torque, ForceMode forceMode = ForceMode.Force);
    public abstract void AddForceAtPosition(Vector3 force, Vector3 position, ForceMode forceMode = ForceMode.Force);
}