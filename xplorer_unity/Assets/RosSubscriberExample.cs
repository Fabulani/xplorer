using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.UI;

using RosColor = RosMessageTypes.UnityRoboticsDemo.PosRotMsg;

public class RosSubscriberExample : MonoBehaviour
{
    public GameObject cube;
    public Text txt;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<RosColor>("color", ColorChange);
    }

    void ColorChange(RosColor colorMessage)
    {
        Debug.Log(colorMessage.pos_x.ToString());
    }
}