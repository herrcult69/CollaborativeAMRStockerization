using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

public class RosTestBridge : MonoBehaviour
{
    private ROSConnection ros;
    public string heartbeatTopic = "/unity_heartbeat";
    public string cmdVelTopic = "/cmd_vel";
    private float timer = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        
        // 1. Publisher: Unity -> ROS 1
        ros.RegisterPublisher<StringMsg>(heartbeatTopic);
        
        // 2. Subscriber: ROS 1 -> Unity (/cmd_vel)
        ros.Subscribe<TwistMsg>(cmdVelTopic, OnCmdVelReceived);
        
        Debug.Log("[ROS Bridge] Ready! Publishing on " + heartbeatTopic + " and listening on " + cmdVelTopic);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer >= 1.0f)
        {
            timer = 0f;
            StringMsg msg = new StringMsg("Heartbeat from Unity 6 on Windows!");
            ros.Publish(heartbeatTopic, msg);
        }
    }

    // Callback when ROS 1 sends velocity commands
    void OnCmdVelReceived(TwistMsg cmd)
    {
        Debug.Log($"<color=green><b>[ROS Bridge] RECEIVED /cmd_vel from ROS 1!</b> Linear X: {cmd.linear.x:F2} m/s, Angular Z: {cmd.angular.z:F2} rad/s</color>");
        
        // Move this GameObject forward based on linear speed!
        transform.Translate(Vector3.forward * (float)cmd.linear.x * Time.deltaTime);
    }
}
