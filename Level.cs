using AYellowpaper.SerializedCollections;
using System.Collections.Generic;
using UnityEngine;

[DefaultExecutionOrder(-90)]
public class Level : SimulationObject
{
    private static Dictionary<string, Level> _Instances;
    public static Dictionary<string, Level> Instances { get { return _Instances; } }

    [SerializeField] private string _levelName;
    [SerializeField] private List<Transform> _loadOrigins;
    [SerializeField] private SerializedDictionary<string, SceneLoadData> _sceneLoadDatas = new SerializedDictionary<string, SceneLoadData>();

    public string levelName { get { return _levelName; } }

    public override void OnEnable()
    {
        if (_Instances == null)
        {
            _Instances = new Dictionary<string, Level>();
        }

        if (_Instances.ContainsKey(name) && _Instances[name] != this)
        {
            Destroy(gameObject);
            return;
        }

        _Instances[name] = this;
        base.OnEnable();
    }

    public void OnDestroy()
    {
        _Instances.Remove(name);
    }
    
    public override void PreSimulationUpdate()
    {
        foreach ((string sceneName, SceneLoadData sceneLoadData) in _sceneLoadDatas)
        {
            if (sceneLoadData.unloadBlockers.Count > 0)
            {
                continue;
            }

            if (sceneLoadData.isLoaded)
            {
                bool stayLoaded = false;
                foreach (Transform loadOrigin in _loadOrigins)
                {
                    if (sceneLoadData.unloadBounds.Contains(loadOrigin.position))
                    {
                        stayLoaded = true;
                        break;
                    }
                }
                if (stayLoaded)
                {
                    continue;
                }

                UnityEngine.SceneManagement.SceneManager.UnloadSceneAsync(sceneName);
                sceneLoadData.isLoaded = false;
                continue;
            }
            foreach (Transform loadOrigin in _loadOrigins)
            {
                if (sceneLoadData.loadBounds.Contains(loadOrigin.position))
                {
                    UnityEngine.SceneManagement.SceneManager.LoadSceneAsync(sceneName, UnityEngine.SceneManagement.LoadSceneMode.Additive);
                    sceneLoadData.isLoaded = true;
                    break;
                }
            }
        }
    }
}