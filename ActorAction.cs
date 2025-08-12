using UnityEngine;

[System.Serializable]
public abstract class ActorAction
{
    [SerializeField] protected Actor _actor;
    public virtual void Start() { }
    public virtual void Perform() { }
    public virtual void Cancel() { }
}