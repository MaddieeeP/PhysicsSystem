using UnityEngine;

[DefaultExecutionOrder(110)]
public abstract class Actor : BasicEntity
{
    protected ActorState _currentState;

    //getters and setters
    public override Vector3 up { get { return _currentState.up; } }
    public abstract Vector3 basePosition { get; }
    public abstract float baseHeight { get; }

    public abstract void UpdateCurrentState();
    public abstract void EndCurrentState();

    public abstract void MoveBasePosition(Vector3 position);

    public override void SimulationUpdate(float deltaTime)
    {
        base.SimulationUpdate(deltaTime);

        Vector3 actualLinearVelocityCopy = actualLinearVelocity;
        Vector3 actualAngularVelocityCopy = actualAngularVelocity;
        _currentState.SimulationUpdate(ref actualLinearVelocityCopy, ref actualAngularVelocityCopy, deltaTime);
        actualLinearVelocity = actualLinearVelocityCopy;
        actualAngularVelocity = actualAngularVelocityCopy;
    }

    public override void LateSimulationUpdate(float deltaTime)
    {
        base.LateSimulationUpdate(deltaTime);

        Vector3 actualLinearVelocityCopy = actualLinearVelocity;
        Vector3 actualAngularVelocityCopy = actualAngularVelocity;
        _currentState.LateSimulationUpdate(ref actualLinearVelocityCopy, ref actualAngularVelocityCopy, deltaTime);
        actualLinearVelocity = actualLinearVelocityCopy;
        actualAngularVelocity = actualAngularVelocityCopy;

        UpdateCurrentState();
    }
}