using System;
using UnityEngine;
using UnityEngine.Splines;

[RequireComponent(typeof(SplineContainer))]
public class Rail : MonoBehaviour
{
    [SerializeField] private int _resolution;
    [SerializeField] private int _iterations;
    [SerializeField] private PhysicsMaterial _sharedMaterial;

    private SplineContainer _splineContainer;
    private float _splineLength;

    public SplineContainer splineContainer { get { return _splineContainer; } }
    public int resolution { get { return _resolution; } }
    public int iterations { get { return _iterations; } }
    public PhysicsMaterial sharedMaterial { get { return _sharedMaterial; } }
    public float splineLength { get { return _splineLength; } }

    public void OnEnable()
    {
        _splineContainer = GetComponent<SplineContainer>();
        _splineLength = _splineContainer.Spline.GetLength();
    }
}
