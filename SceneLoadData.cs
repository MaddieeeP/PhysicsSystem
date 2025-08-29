using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class SceneLoadData
{
    [SerializeField] private List<GameObject> _unloadBlockers = new List<GameObject>();
    [SerializeField] private Bounds _loadBounds;
    [SerializeField] private Bounds _unloadBounds;
    private bool _isLoaded = false;

    public List<GameObject> unloadBlockers { get { return _unloadBlockers; } }
    public Bounds loadBounds { get { return _loadBounds; } }
    public Bounds unloadBounds { get { return _unloadBounds; } }
    public bool isLoaded { get { return _isLoaded; } set { _isLoaded = value; } }
}