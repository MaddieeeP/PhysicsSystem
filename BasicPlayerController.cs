using UnityEngine;
using UnityEngine.InputSystem;

public class BasicPlayerController : Actor
{
    [SerializeField] private float _jumpVelocity = 5f;

    [SerializeField] private Transform _camera;
    [SerializeField] private float _cameraDistance = 5f;
    [SerializeField] private float _cameraMinXRotation = -50f;
    [SerializeField] private float _cameraMaxXRotation = 70f;
    [SerializeField] private int _cameraMaxRotationSpeed = 360;

    [SerializeField] private InputAction _move;
    [SerializeField] private InputAction _jump;
    [SerializeField] private InputAction _look;

    public override void OnEnable()
    {
        _move.Enable();
        _jump.Enable();
        _jump.performed += context => OnJumpPerformed();
        _look.Enable();
        base.OnEnable();
    }

    public override void OnDisable()
    {
        _move.Disable();
        _jump.Disable();
        _look.Disable();
        base.OnDisable();
    }

    public void OnJumpPerformed()
    {
        if (!grounded || isKinematic)
        {
            return;
        }

        AddForce(up * _jumpVelocity, ForceMode.VelocityChange);
    }

    protected override Vector3 GetTargetMove()
    {
        Vector2 moveInput = _move.ReadValue<Vector2>();
        return _camera.rotation * new Vector3(moveInput.x, 0f, moveInput.y);
    }

    protected override Vector3 GetTargetForward()
    {
        Vector3 targetMove = GetTargetMove();
        Vector3 planarVelocity = velocity.RemoveComponentAlongAxis(up);
        return targetMove.magnitude > 0.1f ? targetMove : planarVelocity.magnitude > 0.1f ? planarVelocity.normalized : transform.forward.RemoveComponentAlongAxis(up).normalized;
    }

    public override void SimulationUpdate(float deltaTime)
    {
        base.SimulationUpdate(deltaTime);

        Vector2 look = _look.ReadValue<Vector2>() * 0.5f;

        Vector3 xAxis = Vector3.Cross(up, transform.position - _camera.position);

        Quaternion deltaRotation = Quaternion.AngleAxis(look.x, up) * Quaternion.AngleAxis(look.y, xAxis);

        _camera.rotation = Quaternion.RotateTowards(_camera.rotation, Quaternion.LookRotation(deltaRotation * _camera.forward, up), _cameraMaxRotationSpeed * deltaTime);

        float xRotation = Vector3.Angle(up, _camera.forward) - 90f;

        if (xRotation < _cameraMinXRotation)
        {
            _camera.RotateAround(transform.position, xAxis, _cameraMinXRotation - xRotation);
        }
        else if (xRotation > _cameraMaxXRotation)
        {
            _camera.RotateAround(transform.position, xAxis, _cameraMaxXRotation - xRotation);
        }

        _camera.position = transform.position + _camera.rotation * new Vector3(0f, 0f, -_cameraDistance);
    }
}