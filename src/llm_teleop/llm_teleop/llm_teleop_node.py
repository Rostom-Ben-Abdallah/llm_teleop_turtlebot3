import json
import re
import threading
import time

import requests
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class LlmTeleop(Node):
    def __init__(self):
        super().__init__('llm_teleop')

        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Subscribe to natural-language commands
        self.sub = self.create_subscription(
            String, '/llm_text', self.on_text, 10
        )

        # Parameters
        self.declare_parameter('ollama_url', 'http://127.0.0.1:11434')
        self.declare_parameter('model', 'llama3.1:8b')  # adapt if needed
        self.declare_parameter('max_lin', 0.25)  # m/s
        self.declare_parameter('max_ang', 1.0)   # rad/s

        self.get_logger().info(
            'LlmTeleop ready. Publish std_msgs/String to /llm_text, '
            'e.g. "go forward a little".'
        )

    # ======================================================================

    def on_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"Received text command: {text}")

        try:
            twist, duration = self.query_llm(text)
        except Exception as e:
            self.get_logger().error(f"LLM/query error: {e}")
            return

        self.get_logger().info(
            f"Executing: v={twist.linear.x:.2f} m/s, "
            f"w={twist.angular.z:.2f} rad/s for {duration:.1f} s"
        )

        # Run motion in a small background thread so callback returns quickly
        threading.Thread(
            target=self.execute_motion,
            args=(twist, duration),
            daemon=True
        ).start()

    # ======================================================================

    def query_llm(self, user_text: str):
        ollama_url = self.get_parameter(
            'ollama_url').get_parameter_value().string_value
        model = self.get_parameter(
            'model').get_parameter_value().string_value
        max_lin = float(self.get_parameter(
            'max_lin').get_parameter_value().double_value)
        max_ang = float(self.get_parameter(
            'max_ang').get_parameter_value().double_value)

        system_prompt = (
            "You control a small differential-drive TurtleBot3 robot in Gazebo.\n"
            "The user gives a natural language command like:\n"
            "  - 'go forward a little'\n"
            "  - 'back up and turn left'\n"
            "  - 'rotate in place to the right'\n"
            "You must respond with ONLY a single JSON object, no explanations.\n"
            "The JSON schema is exactly:\n"
            "{\n"
            '  "v": <float linear velocity in m/s>,\n'
            '  "w": <float angular velocity in rad/s>,\n'
            '  "duration": <float duration in seconds>\n'
            "}\n"
            f"Use safe small values, with |v| <= {max_lin} and |w| <= {max_ang}.\n"
        )

        payload = {
            "model": model,
            "stream": False,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_text},
            ],
        }

        url = ollama_url.rstrip('/') + '/api/chat'
        r = requests.post(url, json=payload, timeout=20)
        r.raise_for_status()
        data = r.json()
        content = data["message"]["content"]
        self.get_logger().info(f"LLM raw reply: {content}")

        # Extract the first {...} as JSON
        m = re.search(r"\{.*\}", content, re.DOTALL)
        if not m:
            raise RuntimeError("No JSON object found in LLM reply")

        obj = json.loads(m.group(0))
        v = float(obj.get("v", 0.0))
        w = float(obj.get("w", 0.0))
        duration = float(obj.get("duration", 1.5))

        # Clamp
        v = max(-max_lin, min(max_lin, v))
        w = max(-max_ang, min(max_ang, w))
        if duration <= 0.0:
            duration = 1.0

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist, duration

    # ======================================================================

    def execute_motion(self, twist: Twist, duration: float):
        end = time.time() + duration
        rate_hz = 10.0
        dt = 1.0 / rate_hz
        while time.time() < end and rclpy.ok():
            self.cmd_pub.publish(twist)
            time.sleep(dt)

        # Stop at the end
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = LlmTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
