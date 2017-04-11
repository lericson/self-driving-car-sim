using UnityEngine;
using System.Collections.Generic;
using System.Collections;
using SocketIO;
using UnityStandardAssets.Vehicles.Car;
using UnityEngine.SceneManagement;
using System;
using System.Security.AccessControl;

public class CommandServer : MonoBehaviour
{
	public CarRemoteControl CarRemoteControl;
	public Camera FrontFacingCamera;
	private SocketIOComponent _socket;
	private CarController _carController;

	// Use this for initialization
	void Start()
	{
		_socket = GameObject.Find("SocketIO").GetComponent<SocketIOComponent>();
		_socket.On("open", OnOpen);
		_socket.On("steer", OnSteer);
		_socket.On("reset_level", OnResetLevel);
		_carController = CarRemoteControl.GetComponent<CarController>();
		StartCoroutine(EmitTelemetry());
	}

	// Update is called once per frame
	void Update()
	{
	}

	void OnOpen(SocketIOEvent obj)
	{
		Debug.Log("Connection Open");
	}

	void OnSteer(SocketIOEvent obj)
	{
		JSONObject jsonObject = obj.data;
		//    print(float.Parse(jsonObject.GetField("steering_angle").str));
		CarRemoteControl.SteeringAngle = float.Parse(jsonObject.GetField("steering_angle").str);
		CarRemoteControl.Acceleration = float.Parse(jsonObject.GetField("throttle").str);
	}

	void OnResetLevel(SocketIOEvent obj)
	{
		string sceneName = SceneManager.GetActiveScene ().name;
		Debug.Log ($"resetting level {sceneName}");
		SceneManager.LoadScene (sceneName);
	}

	IEnumerator EmitTelemetry()
	{
		yield return new WaitForSeconds(0.0666666666666667f);

		// send only if it's not being manually driven
		if ((Input.GetKey(KeyCode.W)) || (Input.GetKey(KeyCode.S))) {
			_socket.Emit("telemetry", new JSONObject());
		}
		else {
			// Collect Data from the 
			Dictionary<string, string> data = new Dictionary<string, string>();
			data["steering_angle"] = _carController.CurrentSteerAngle.ToString("N4");
			data["throttle"] = _carController.AccelInput.ToString("N4");
			data["speed"] = _carController.CurrentSpeed.ToString("N4");
			data["num_wheels_on_road"] = _carController.NumWheelsOnRoad.ToString("N4");
			data["image"] = Convert.ToBase64String(CameraHelper.CaptureFrame(FrontFacingCamera));
			_socket.Emit("telemetry", new JSONObject(data));
		}

		StartCoroutine(EmitTelemetry());
	}
}