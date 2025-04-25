using UnityEngine;

public abstract class SimulationObject : MonoBehaviour
{
    [SerializeField] private SimulationMode _simulationMode = SimulationMode.FixedUpdate;
    private SimulationMode _simulationModeBuffer = SimulationMode.FixedUpdate;
    private float _relativeTimeScale = 1f;
    private float _relativeTimeScaleBuffer = 1f;

    
    public SimulationMode simulationMode { get { return _simulationMode; } }
    public float relativeTimeScale { get { return _relativeTimeScale; } }

    public void SetRelativeTimeScale(float timeScale)
    {
        _relativeTimeScaleBuffer = timeScale;
    }

    public void SetSimulationMode(SimulationMode simulationMode)
    {
        _simulationModeBuffer = simulationMode;
    }

    public virtual void ApplyValueChanges()
    {
        _relativeTimeScale = _relativeTimeScaleBuffer;
        if (_simulationMode != _simulationModeBuffer)
        {
            if (gameObject.activeInHierarchy)
            {
                StopListeningForSimulationAction();
            }

            _simulationMode = _simulationModeBuffer;
            ListenForSimulationAction();
        }
    }

    public virtual void OnEnable()
    {
        ListenForSimulationAction();
    }

    public virtual void OnDisable()
    {
        StopListeningForSimulationAction();
    }

    private void ListenForSimulationAction()
    {
        if (_simulationMode == SimulationMode.FixedUpdate)
        {
            SimulationController.UnityFixedUpdate += SimulationUpdate;
            SimulationController.UnityLateFixedUpdate += LateSimulationUpdate;
        }
        else if (_simulationMode == SimulationMode.Update)
        {
            SimulationController.UnityUpdate += SimulationUpdate;
            SimulationController.UnityLateUpdate += LateSimulationUpdate;
        }
    }

    private void StopListeningForSimulationAction()
    {
        if (_simulationMode == SimulationMode.FixedUpdate)
        {
            SimulationController.UnityFixedUpdate -= SimulationUpdate;
            SimulationController.UnityLateFixedUpdate -= LateSimulationUpdate;
        }
        else if (_simulationMode == SimulationMode.Update)
        {
            SimulationController.UnityUpdate -= SimulationUpdate;
            SimulationController.UnityLateUpdate -= LateSimulationUpdate;
        }
    }

    public virtual void SimulationUpdate(float deltaTime) { } //Data from other SimulationObjects can be read

    public virtual void LateSimulationUpdate(float deltaTime) //Data about self can be updated
    { 
        ApplyValueChanges();
    }
}