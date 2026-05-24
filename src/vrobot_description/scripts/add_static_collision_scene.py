#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class StaticCollisionScene(Node):
    def __init__(self):
        super().__init__("static_collision_scene")
        self._client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def apply_floor(self) -> bool:
        if not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("/apply_planning_scene service is not available yet")
            return False

        floor = CollisionObject()
        floor.header.frame_id = "base_link"
        floor.id = "floor"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [4.0, 4.0, 0.1]

        pose = Pose()
        pose.position.z = -0.06
        pose.orientation.w = 1.0

        floor.primitives = [primitive]
        floor.primitive_poses = [pose]
        floor.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [floor]

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error("Failed to apply floor collision object")
            return False

        if not future.result().success:
            self.get_logger().error("MoveIt rejected floor collision object")
            return False

        self.get_logger().info("Applied static floor collision object")
        return True


def main():
    rclpy.init()
    node = StaticCollisionScene()

    success = False
    for _ in range(30):
        if node.apply_floor():
            success = True
            break
        time.sleep(1.0)

    if not success:
        node.get_logger().error("Unable to apply static collision scene")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
