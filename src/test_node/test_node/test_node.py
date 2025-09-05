import json
import re

import ollama
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

MODEL = "gemma3:12b-it-q4_K_M"


class TurtleCommander(Node):
    def __init__(self):
        super().__init__("turtle_commander")
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    def command(self, text: str):
        response = ollama.generate(
            model=MODEL,
            prompt=f"""
            Convert the following instructions to ROS2 Twist JSON:
            
            Example:
                Instruction: 'move forward'
                Output: {{"linear": 1.0, "angular": 0.0}}

                Instruction: 'move backward'
                Output: {{"linear": -1.0, "angular": 0.0}}

                Instruction: 'turn left'
                Output: {{"linear": 0.0, "angular": 1.0}}

                Instruction: 'turn right'
                Output: {{"linear": 0.0, "angular": -1.0}}

                Instruction: '前に進んで'
                Output: {{"linear": 1.0, "angular": 0.0}}

                Instruction: '後ろに進んで'
                Output: {{"linear": -1.0, "angular": 0.0}}

                Instruction: '左に曲がれ'
                Output: {{"linear": 0.0, "angular": 1.0}}   # 左は正

                Instruction: '右に曲がれ'
                Output: {{"linear": 0.0, "angular": -1.0}}  # 右は負

            Instruction: '{text}'
            Output:
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


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()

    try:
        while rclpy.ok():
            text = input("指示を入力してください: ")
            node.command(text)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
