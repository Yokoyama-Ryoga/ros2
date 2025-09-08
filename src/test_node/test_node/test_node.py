import json
import re

import ollama
import rclpy
from geometry_msgs.msg import Twist
from pydantic import BaseModel
from rclpy.node import Node
from turtlesim.msg import Pose

MODEL = "gemma3:12b-it-q4_K_M"


class CommandRequest(BaseModel):
    linear: float
    angular: float


class TurtleCommander(Node):
    def __init__(self):
        super().__init__("turtle_commander")
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.subscription = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        self.current_pose = None

    def pose_callback(self, msg: Pose):
        self.current_pose = msg
        self.get_logger().info(
            f"Pose -> x: {msg.x:.2f}, y: {msg.y:.2f}, theta: {msg.theta:.2f}, "
            f"linear_vel: {msg.linear_velocity:.2f}, angular_vel: {msg.angular_velocity:.2f}"
        )

    def command(self, text: str):
        response = ollama.generate(
            model=MODEL,
            format=CommandRequest.model_json_schema(),
            prompt=f"""
            instructions: {text}
            """,
        )
        raw = response["response"]
        self.get_logger().info(f"Raw response: {raw}")

        try:
            # {} で囲まれた最初の部分だけ抽出
            match = re.search(r"\{.*?\}", raw, re.DOTALL)
            if not match:
                raise ValueError("No JSON found in response")

            json_str = match.group().replace(
                "'", '"'
            )  # シングルクオート→ダブルクオート
            data = json.loads(json_str)

            msg = Twist()
            msg.linear.x = float(data.get("linear", 0))
            msg.angular.z = float(data.get("angular", 0))
            self.publisher_.publish(msg)
            self.get_logger().info(
                f"Command: {text} → linear={msg.linear.x} angular={msg.angular.z}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to parse response: {raw}\n{e}")


def hit_wall(pose, x_min=0, x_max=11, y_min=0, y_max=11):
    if pose.x <= x_min or pose.x >= x_max or pose.y <= y_min or pose.y >= y_max:
        return True
    return False


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()

    try:
        text = input("指示を入力してください: ")
        while rclpy.ok():
            rclpy.spin_once(node)
            if hit_wall(node.current_pose):
                node.get_logger().info("壁に衝突しました。終了します。")
                break
            node.command(text)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
