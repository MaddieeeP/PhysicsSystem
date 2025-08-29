using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class SceneObjectData : MonoBehaviour
{
    private static Dictionary<Scene, SceneObjectData> _Instances;

    [SerializeField] string _levelName;
    [SerializeField] Dictionary<Type, List<SimulationObject>> _objects = new Dictionary<Type, List<SimulationObject>>();

    public static Dictionary<Scene, SceneObjectData> Instances { get { return _Instances; } }
    public string levelName { get { return _levelName; } }

    public void OnEnable()
    {
        Scene scene = gameObject.scene;

        if (_Instances == null)
        { 
            _Instances = new Dictionary<Scene, SceneObjectData>();
        }

        if (_Instances.ContainsKey(scene) && _Instances[scene] != this)
        {
            Destroy(gameObject);
            return;
        }

        _Instances[scene] = this;
    }

    public void OnDestroy()
    {
        foreach (Scene scene in _Instances.Keys)
        {
            if (scene.IsValid())
            {
                continue;
            }
            _Instances.Remove(scene);
        }
    }
}