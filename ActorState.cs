using UnityEngine;

[System.Serializable]

public abstract class ActorState
{
    [SerializeField] protected Actor _actor;

    //Getters and setters
    public abstract string stateID { get; }
    public abstract Vector3 up { get; }

    public abstract bool TryStart();

    public virtual void SimulationUpdate(ref Vector3 actualLinearVelocity, ref Vector3 actualAngularVelocity, float deltaTime) { }
    public virtual void LateSimulationUpdate(ref Vector3 actualLinearVelocity, ref Vector3 actualAngularVelocity, float deltaTime) { }
}