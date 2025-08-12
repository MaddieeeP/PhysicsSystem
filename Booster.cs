using UnityEngine;

[RequireComponent(typeof(Collider))]
public class Booster : SimulationObject
{
    [SerializeField] private float _speed = 10f;

    public void OnTriggerEnter(Collider other)
    {
        if (other.TryGetComponent(out BasicEntity entity))
        {
            entity.SetVelocities(transform.forward * _speed, entity.angularVelocity);
            entity.MovePosition(transform.position);
        }
    }
}