using UnityEngine;

[System.Serializable]
public class TransformInfo
{
    private Vector3 _position = Vector3.zero;
    private Quaternion _rotation = Quaternion.identity;
    private Vector3 _scale = Vector3.one;

    //getters and setters
    public Vector3 position { get { return _position; } set { _position = value; } }
    public Quaternion rotation { get { return _rotation; } set { _rotation = value; } }
    public Vector3 scale { get { return _scale; } set { _scale = value; } }

    public TransformInfo() { }

    public TransformInfo(Vector3 position, Quaternion rotation, Vector3 scale)
    {
        _position = position;
        _rotation = rotation;
        _scale = scale;
    }

    public TransformInfo(Transform transform) => new TransformInfo(transform.position, transform.rotation, transform.lossyScale);

    public void CopyTo(Transform transform)
    {
        transform.position = _position;
        transform.rotation = _rotation;
        transform.SetScale(scale);
    }
}