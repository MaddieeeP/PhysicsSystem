using UnityEngine;

public class ForceDownBox : ForceField
{
    [SerializeField] protected Vector3 _halfExtents;

    void FixedUpdate()
    {
        Collider[] colliders = Physics.OverlapBox(transform.position, _halfExtents, transform.rotation);
        Tick(colliders);
    }

    public override Vector3 GetForce(Entity entity)
    {
        float falloffMultiplier = 1f; //FIX - use relative distance along relative y axis
        return transform.up * -magnitude * falloffMultiplier;
    }
}