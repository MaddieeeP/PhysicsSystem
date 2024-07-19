using UnityEngine;

public enum FloorBehaviour { None, FollowNormal, FollowGravity }

public class ObjectProperties : MonoBehaviour
{
    [SerializeField] private FloorBehaviour _floorBehaviour = FloorBehaviour.None;

    //getters and setters
    public FloorBehaviour floorBehaviour { get { return _floorBehaviour; } }
}