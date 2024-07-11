using System;
using UnityEngine;

public class LateFixedUpdateBroadcaster : MonoBehaviour
{
    public static LateFixedUpdateBroadcaster Instance;
    public static event Action LateFixedUpdate;
    private bool _fixedUpdate = false;

    void Start()
    {
        if (Instance != null)
        {
            Destroy(gameObject);
            return;
        }

        Instance = this;

        //Create a couple of colliders to trigger OnTriggerStay each fixed update frame after the physics run
        SphereCollider thisCollider = gameObject.AddComponent<SphereCollider>();
        thisCollider.isTrigger = true;

        GameObject other = new GameObject();
        other.AddComponent<SphereCollider>();
        Rigidbody otherRigidbody = other.AddComponent<Rigidbody>();
        otherRigidbody.isKinematic = true;
        other.transform.parent = transform;
        other.transform.position = transform.position;
        other.name = "Intersecting Collider";
    }
    
    void OnDestroy()
    {
        if (Instance == this)
        {
            Instance = null;
        }
    }

    void FixedUpdate()
    {
        _fixedUpdate = true;
    }

    void OnTriggerStay()
    {
        if (!_fixedUpdate) //LateFixedUpdate could be invoked multiple times if there is another collider so it is limited by FixedUpdate
        {
            return;
        }

        _fixedUpdate = false;
        LateFixedUpdate?.Invoke();
    }
}