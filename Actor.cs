using UnityEngine;

[DefaultExecutionOrder(110)]
public abstract class Actor : BasicEntity
{
    protected ActorState _currentState;

    //getters and setters
    public abstract Vector3 basePosition { get; }
    public abstract float baseHeight { get; }

    public abstract void UpdateCurrentState();
    public abstract void EndCurrentState();

    public abstract void MoveBasePosition(Vector3 position);

    public override void PreSimulationUpdate()
    {
        base.PreSimulationUpdate();

        Vector3 linearVelocityCopy = linearVelocity;
        Vector3 angularVelocityCopy = angularVelocity;
        _currentState.SimulationUpdate(ref linearVelocityCopy, ref angularVelocityCopy);
        linearVelocity = linearVelocityCopy;
        angularVelocity = angularVelocityCopy;
    }

    public override void PostSimulationUpdate()
    {
        base.PostSimulationUpdate();

        Vector3 linearVelocityCopy = linearVelocity;
        Vector3 angularVelocityCopy = angularVelocity;
        _currentState.LateSimulationUpdate(ref linearVelocityCopy, ref angularVelocityCopy);
        linearVelocity = linearVelocityCopy;
        angularVelocity = angularVelocityCopy;

        UpdateCurrentState();
    }
}