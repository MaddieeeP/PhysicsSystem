using UnityEngine;

public interface IEntity
{
    public abstract bool isKinematic { get; set; }
    public abstract float relativeTimeScale { get; }
    public abstract Transform transform { get; }
    public abstract Collider collider { get; }
    public abstract Vector3 gravity { get; }
    public abstract Vector3 up { get; }
    public abstract Vector3 linearVelocity { get; } //velocity is relative
    public abstract Vector3 angularVelocity { get; } //angularVelocity is relative

    public abstract RaycastHitInfoVerbose GetPredictedTransformedSurfaceInfo(RaycastHitInfoVerbose surfaceInfo, float deltaTime); //Modify surfaceInfo to reflect predicted motion in next simulation step
    public abstract void AddGravityForce(Vector3 force);
    public abstract void AddForce(Vector3 force, ForceMode forceMode = ForceMode.Force);
    public abstract void AddTorque(Vector3 torque, ForceMode forceMode = ForceMode.Force);
    public abstract void AddForceAtPosition(Vector3 force, Vector3 position, ForceMode forceMode = ForceMode.Force);
}