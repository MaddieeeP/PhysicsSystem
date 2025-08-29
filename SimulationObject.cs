using UnityEngine;

public abstract class SimulationObject : MonoBehaviour
{
    private SimulationController _simulationController;

    public SimulationController simulationController { get { return _simulationController; } }

    public virtual void OnEnable()
    {
        _simulationController = SimulationController.Instances[gameObject.scene.GetPhysicsScene()];
        _simulationController.PreSimulationUpdate += PreSimulationUpdate;
        _simulationController.PostSimulationUpdate += PostSimulationUpdate;
    }

    public virtual void OnDisable()
    {
        _simulationController.PreSimulationUpdate -= PreSimulationUpdate;
        _simulationController.PostSimulationUpdate -= PostSimulationUpdate;
    }

    public virtual void PreSimulationUpdate() { }

    public virtual void PostSimulationUpdate() { }
}