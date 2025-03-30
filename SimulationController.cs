using System;
using UnityEngine;

public class SimulationController : MonoBehaviour
{
    private static float _globalTimeScale = 1f; //Use for slowdown and speed up effect and pausing
    private static float _globalTimeScaleBuffer = 1f;
    private GameObject _intersecter;

    private static SimulationController Instance;
    public static event Action<float> UnityUpdate;
    public static event Action<float> UnityLateUpdate;
    public static event Action<float> UnityFixedUpdate;
    public static event Action<float> UnityLateFixedUpdate;

    public static float globalTimeScale { get { return _globalTimeScale; } }

    public static void SetGlobalTimeScale(float timeScale)
    {
        _globalTimeScaleBuffer = timeScale;
    }

    void Start()
    {
        if (Instance != null)
        {
            Destroy(gameObject);
            return;
        }

        Instance = this;
        DontDestroyOnLoad(gameObject);

        //Create a couple of colliders to trigger OnTriggerStay after Unity simulates physics collisions
        Collider collider = gameObject.AddComponent<SphereCollider>();
        collider.isTrigger = true;

        _intersecter = new GameObject();
        _intersecter.AddComponent<SphereCollider>();
        Rigidbody rb = _intersecter.AddComponent<Rigidbody>();
        rb.useGravity = false;
        _intersecter.transform.parent = transform;
        _intersecter.transform.position = transform.position;
        _intersecter.name = "Intersecter";
    }
    
    void OnDestroy()
    {
        Destroy(_intersecter);
        if (Instance == this)
        {
            Instance = null;
        }
    }

    void Update()
    {
        UnityUpdate?.Invoke(Time.deltaTime);
    }

    void LateUpdate()
    {
        UnityLateUpdate?.Invoke(Time.deltaTime);
        _globalTimeScale = _globalTimeScaleBuffer;
    }

    void FixedUpdate()
    {
        UnityFixedUpdate?.Invoke(Time.fixedDeltaTime);
    }

    void OnTriggerStay()
    {
        UnityLateFixedUpdate?.Invoke(Time.fixedDeltaTime);
        _globalTimeScale = _globalTimeScaleBuffer;
        _intersecter.transform.position = transform.position;
    }
}