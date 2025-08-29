using System;
using System.Collections.Generic;
using UnityEngine;

[DefaultExecutionOrder(-100)]
public class SimulationController : MonoBehaviour
{
    private static Dictionary<PhysicsScene, SimulationController> _Instances;

    [SerializeField] private bool _simulateOnFixedUpdate = false;
    [SerializeField] private float _simulationDeltaTime = 0.2f;
    private float _stepSimulationDeltaTime; //Step size must be consistent across each step even if simulationDeltaTime is changed

    public event Action PreSimulationUpdate;
    public event Action PostSimulationUpdate;
    
    public static Dictionary<PhysicsScene, SimulationController> Instances { get { return _Instances; } }
    public float simulationDeltaTime { get { return _stepSimulationDeltaTime; } set { _simulationDeltaTime = value; } }

    public void OnEnable()
    {
        PhysicsScene physicsScene = gameObject.scene.GetPhysicsScene();

        if (_Instances == null)
        {
            _Instances = new Dictionary<PhysicsScene, SimulationController>();
        }

        if (_Instances.ContainsKey(physicsScene) && _Instances[physicsScene] != this)
        {
            Destroy(gameObject);
            return;
        }

        _Instances[physicsScene] = this;
        DontDestroyOnLoad(gameObject);
    }

    public void OnDestroy()
    {
        foreach (PhysicsScene physicsScene in _Instances.Keys) 
        {
            if (physicsScene.IsValid())
            {
                continue;
            }
            _Instances.Remove(physicsScene);
        }
    }

    public void Simulate() //Collision and Trigger events are scheduled by Physics.Simulate so will not behave properly if Simulate is called multiple times consecutively
    {
        _stepSimulationDeltaTime = _simulationDeltaTime;

        PreSimulationUpdate?.Invoke();
        Physics.Simulate(_stepSimulationDeltaTime);
        PostSimulationUpdate?.Invoke();
    }

    public void FixedUpdate()
    {
        if (_simulateOnFixedUpdate)
        {
            Simulate();
        }
    }
}