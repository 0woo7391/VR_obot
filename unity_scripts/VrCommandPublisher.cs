using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class VrCommandPublisher : MonoBehaviour
{
    [Header("ROS2 Topics")]
    public string poseTopic = "/vr/pose_goal";
    public string gripperTopic = "/vr/gripper_goal";

    [Header("Publishing")]
    public bool publishPoseContinuously = false;
    public float publishRateHz = 10f;
    public float positionThresholdMeters = 0.01f;
    public float rotationThresholdDegrees = 2f;
    public bool publishPoseOnce;
    public Transform targetTransform;
    public string frameId = "base_link";

    [Header("Gripper")]
    [Range(0f, 0.8f)]
    public float gripperGoalValue = 0f;

    private const float GripperPublishThreshold = 0.01f;

    private ROSConnection ros;
    private float nextPosePublishTime;
    private Vector3 lastPublishedPosition;
    private Quaternion lastPublishedRotation;
    private bool hasPublishedPose;
    private float lastPublishedGripperValue = float.NaN;
    private bool warnedMissingTarget;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(poseTopic);
        ros.RegisterPublisher<Float64Msg>(gripperTopic);
    }

    void Update()
    {
        if (publishPoseContinuously)
        {
            PublishPoseIfDue();
        }
        else if (publishPoseOnce)
        {
            publishPoseOnce = false;
            PublishCurrentPose();
        }

        PublishGripperIfChanged();
    }

    public void SetGripperOpen()
    {
        gripperGoalValue = 0f;
    }

    public void SetGripperClose()
    {
        gripperGoalValue = 0.8f;
    }

    public void PublishCurrentPose()
    {
        if (targetTransform == null)
        {
            Debug.LogWarning("[VrCommandPublisher] Target Transform is not assigned.");
            return;
        }

        lastPublishedPosition = targetTransform.position;
        lastPublishedRotation = targetTransform.rotation;
        hasPublishedPose = true;

        ros.Publish(poseTopic, BuildPoseStampedMessage());
        Debug.Log("[VrCommandPublisher] Published current pose goal.");
    }

    private void PublishPoseIfDue()
    {
        if (targetTransform == null)
        {
            if (!warnedMissingTarget)
            {
                Debug.LogWarning("[VrCommandPublisher] Target Transform is not assigned.");
                warnedMissingTarget = true;
            }

            return;
        }

        float publishInterval = 1f / Mathf.Max(0.1f, publishRateHz);
        if (Time.time < nextPosePublishTime)
        {
            return;
        }

        if (!ShouldPublishPose())
        {
            return;
        }

        nextPosePublishTime = Time.time + publishInterval;
        lastPublishedPosition = targetTransform.position;
        lastPublishedRotation = targetTransform.rotation;
        hasPublishedPose = true;

        ros.Publish(poseTopic, BuildPoseStampedMessage());
    }

    private bool ShouldPublishPose()
    {
        if (!hasPublishedPose)
        {
            return true;
        }

        float positionDelta = Vector3.Distance(targetTransform.position, lastPublishedPosition);
        float rotationDelta = Quaternion.Angle(targetTransform.rotation, lastPublishedRotation);

        return positionDelta >= positionThresholdMeters ||
            rotationDelta >= rotationThresholdDegrees;
    }

    private void PublishGripperIfChanged()
    {
        float clampedValue = Mathf.Clamp(gripperGoalValue, 0f, 0.8f);
        if (!Mathf.Approximately(gripperGoalValue, clampedValue))
        {
            gripperGoalValue = clampedValue;
        }

        if (!float.IsNaN(lastPublishedGripperValue) &&
            Mathf.Abs(clampedValue - lastPublishedGripperValue) < GripperPublishThreshold)
        {
            return;
        }

        lastPublishedGripperValue = clampedValue;
        ros.Publish(gripperTopic, new Float64Msg(clampedValue));
    }

    private PoseStampedMsg BuildPoseStampedMessage()
    {
        Vector3 unityPosition = targetTransform.position;
        Quaternion unityRotation = targetTransform.rotation;

        PointMsg rosPosition = new PointMsg(
            unityPosition.z,
            -unityPosition.x,
            unityPosition.y
        );

        QuaternionMsg rosRotation = new QuaternionMsg(
            unityRotation.z,
            -unityRotation.x,
            unityRotation.y,
            -unityRotation.w
        );

        PoseMsg pose = new PoseMsg(rosPosition, rosRotation);
        HeaderMsg header = new HeaderMsg(GetCurrentRosTime(), frameId);

        return new PoseStampedMsg(header, pose);
    }

    private TimeMsg GetCurrentRosTime()
    {
        double time = Time.timeAsDouble;
        int sec = Mathf.FloorToInt((float)time);
        uint nanosec = (uint)((time - sec) * 1e9);

        return new TimeMsg(sec, nanosec);
    }
}
