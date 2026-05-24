using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSubscriber : MonoBehaviour
{
    [Serializable]
    public class JointBinding
    {
        public string rosJointName;
        public string unityBodyName;
        public bool invert;
        public float offsetDegrees;
    }

    [Header("ROS2 Config")]
    public string topicName = "/joint_states";

    [Header("Robot Setup")]
    public GameObject robotRoot;

    [Header("Joint Mapping")]
    public List<JointBinding> jointBindings = new List<JointBinding>
    {
        new JointBinding { rosJointName = "joint_1", unityBodyName = "link_1" },
        new JointBinding { rosJointName = "joint_2", unityBodyName = "link_2" },
        new JointBinding { rosJointName = "joint_3", unityBodyName = "link_3" },
        new JointBinding { rosJointName = "joint_4", unityBodyName = "link_4" },
        new JointBinding { rosJointName = "joint_5", unityBodyName = "link_5" },
        new JointBinding { rosJointName = "joint_6", unityBodyName = "link_6" },
        new JointBinding { rosJointName = "rh_r1", unityBodyName = "rh_p12_rn_r1" },
        new JointBinding { rosJointName = "rh_r2", unityBodyName = "rh_p12_rn_r2" },
        new JointBinding { rosJointName = "rh_l1", unityBodyName = "rh_p12_rn_l1" },
        new JointBinding { rosJointName = "rh_l2", unityBodyName = "rh_p12_rn_l2" }
    };

    [Header("Visualization Drive")]
    public bool configureDriveOnStart = true;
    public bool disableGravityOnMappedBodies = true;
    public float stiffness = 100000.0f;
    public float damping = 10000.0f;
    public float forceLimit = 100000.0f;

    private readonly Dictionary<string, ArticulationBody> bodyByRosJoint = new Dictionary<string, ArticulationBody>();
    private readonly Dictionary<string, JointBinding> bindingByRosJoint = new Dictionary<string, JointBinding>();
    private readonly HashSet<string> warnedUnmappedRosJoints = new HashSet<string>();
    private readonly object messageLock = new object();

    private JointStateMsg latestMessage;
    private bool hasNewMessage;

    void Start()
    {
        if (robotRoot == null)
        {
            robotRoot = gameObject;
        }

        BuildJointMap();

        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(topicName, OnJointStateReceived);
        Debug.Log($"[JointStateSubscriber] Subscribed to {topicName}");
    }

    void Update()
    {
        JointStateMsg msgToProcess = null;

        lock (messageLock)
        {
            if (hasNewMessage)
            {
                msgToProcess = latestMessage;
                hasNewMessage = false;
            }
        }

        if (msgToProcess != null)
        {
            ApplyJointState(msgToProcess);
        }
    }

    private void BuildJointMap()
    {
        bodyByRosJoint.Clear();
        bindingByRosJoint.Clear();
        warnedUnmappedRosJoints.Clear();

        ArticulationBody[] bodies = robotRoot.GetComponentsInChildren<ArticulationBody>(true);
        Dictionary<string, ArticulationBody> bodyByUnityName = new Dictionary<string, ArticulationBody>();

        foreach (ArticulationBody body in bodies)
        {
            if (bodyByUnityName.ContainsKey(body.name))
            {
                Debug.LogWarning($"[JointStateSubscriber] Duplicate ArticulationBody name ignored: {body.name}");
                continue;
            }

            bodyByUnityName.Add(body.name, body);
        }

        foreach (JointBinding binding in jointBindings)
        {
            if (string.IsNullOrWhiteSpace(binding.rosJointName) || string.IsNullOrWhiteSpace(binding.unityBodyName))
            {
                Debug.LogWarning("[JointStateSubscriber] Empty joint binding ignored.");
                continue;
            }

            if (bindingByRosJoint.ContainsKey(binding.rosJointName))
            {
                Debug.LogWarning($"[JointStateSubscriber] Duplicate ROS joint binding ignored: {binding.rosJointName}");
                continue;
            }

            if (!bodyByUnityName.TryGetValue(binding.unityBodyName, out ArticulationBody body))
            {
                Debug.LogWarning($"[JointStateSubscriber] Unity body not found: {binding.unityBodyName} for ROS joint {binding.rosJointName}");
                continue;
            }

            bodyByRosJoint.Add(binding.rosJointName, body);
            bindingByRosJoint.Add(binding.rosJointName, binding);

            if (configureDriveOnStart)
            {
                ConfigureDrive(body);
            }

            if (disableGravityOnMappedBodies)
            {
                body.useGravity = false;
            }

            Debug.Log($"[JointStateSubscriber] Mapped {binding.rosJointName} -> {binding.unityBodyName}");
        }
    }

    private void ConfigureDrive(ArticulationBody body)
    {
        ArticulationDrive drive = body.xDrive;
        drive.stiffness = stiffness;
        drive.damping = damping;
        drive.forceLimit = forceLimit;
        body.xDrive = drive;
    }

    private void OnJointStateReceived(JointStateMsg msg)
    {
        lock (messageLock)
        {
            latestMessage = msg;
            hasNewMessage = true;
        }
    }

    private void ApplyJointState(JointStateMsg msg)
    {
        if (msg.name == null || msg.position == null)
        {
            return;
        }

        int count = Mathf.Min(msg.name.Length, msg.position.Length);

        for (int i = 0; i < count; i++)
        {
            string rosJointName = msg.name[i];

            if (!bodyByRosJoint.TryGetValue(rosJointName, out ArticulationBody body))
            {
                if (rosJointName.StartsWith("rh_") && !warnedUnmappedRosJoints.Contains(rosJointName))
                {
                    warnedUnmappedRosJoints.Add(rosJointName);
                    Debug.LogWarning($"[JointStateSubscriber] Received gripper joint without Unity mapping: {rosJointName}");
                }

                continue;
            }

            JointBinding binding = bindingByRosJoint[rosJointName];
            float targetDegrees = (float)(msg.position[i] * Mathf.Rad2Deg);

            if (binding.invert)
            {
                targetDegrees = -targetDegrees;
            }

            targetDegrees += binding.offsetDegrees;

            ArticulationDrive drive = body.xDrive;
            drive.target = targetDegrees;
            body.xDrive = drive;
        }
    }
}
