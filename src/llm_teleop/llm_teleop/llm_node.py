import json
import requests

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class LlmTeleopNode(Node):
    def __init__(self):
        super().__init__("llm_teleop")

        # Parameters
        self.declare_parameter("prompt", "go forward slowly for 2 seconds")
        self.declare_parameter("model", "llama3.1:8b")
        self.declare_parameter("ollama_url", "http://127.0.0.1:11434")

        self.prompt = self.get_parameter("prompt").get_parameter_value().string_value
        self.model = self.get_parameter("model").get_parameter_value().string_value
        self.ollama_url = self.get_parameter("ollama_url").get_parameter_value().string_value

        self.get_logger().info(f'[llm_teleop] Using model: "{self.model}" at {self.ollama_url}')
        self.get_logger().info(f'[llm_teleop] User prompt: "{self.prompt}"')

        # Publisher to cmd_vel
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Service client to vision node
        self.describe_client = self.create_client(Trigger, "describe_scene")

        # Run once when node starts
        self.started = False
        self.timer = self.create_timer(0.1, self._run_once)

    def _run_once(self):
        if self.started:
            return
        self.started = True

        # 1) Query vision (YOLO) for scene description
        scene_json = self._get_scene_description()

        # 2) Build LLM prompt
        full_prompt = self._build_llm_prompt(self.prompt, scene_json)

        # 3) Ask Ollama
        motion = self._ask_ollama(full_prompt)

        # 4) Execute motion
        self._execute_motion(motion)

        # 5) Exit cleanly
        self.get_logger().info("[llm_teleop] Done. Shutting down node.")
        rclpy.shutdown()

    def _get_scene_description(self) -> str:
        """Call /describe_scene Trigger service and return JSON string."""
        if not self.describe_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("[llm_teleop] describe_scene service not available, proceeding without vision.")
            return ""

        req = Trigger.Request()
        future = self.describe_client.call_async(req)
        self.get_logger().info("[llm_teleop] Calling /describe_scene ...")
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            self.get_logger().warn("[llm_teleop] describe_scene call timed out, proceeding without vision.")
            return ""

        resp = future.result()
        if not resp.success:
            self.get_logger().warn(f"[llm_teleop] describe_scene failed: {resp.message}")
            return ""

        self.get_logger().info(f"[llm_teleop] Scene description: {resp.message}")
        return resp.message

    def _build_llm_prompt(self, user_prompt: str, scene_json: str) -> str:
        vision_part = scene_json if scene_json else '{"objects": []}'

        template = f"""
You control a differential-drive mobile robot in a lab.
You receive:
- A natural language command from the user.
- A JSON description of objects detected by a YOLO model in the camera image.

### Vision JSON (YOLO detections)
{vision_part}

Each object has:
- "label": class name (e.g., "person", "bottle", "chair")
- "pos": "left", "center", or "right" relative to the camera
- "range": "near", "mid", or "far"

### Your job
Based on the user's command and the vision JSON,
output ONLY a JSON object (no extra text) with keys:
- "linear": linear velocity in m/s (float, e.g., 0.1)
- "angular": angular velocity in rad/s (float, e.g., 0.5, negative means turn right)
- "duration": how long to apply this command in seconds (float, e.g., 2.0)

### Rules
- If the user asks to approach an object that exists in the vision JSON,
  choose a forward linear velocity (e.g., 0.1) and a small angular velocity
  to turn towards its "pos".
- If the object is "left": use positive angular (e.g., 0.5).
- If "right": use negative angular (e.g., -0.5).
- If "center": angular ~ 0.
- If no relevant object is detected, choose linear = 0.0 and angular = 0.0, duration = 0.0.
- NEVER output anything except a single valid JSON object.

### User command
"{user_prompt}"
"""
        return template

    def _ask_ollama(self, full_prompt: str):
        """Send prompt to Ollama /api/chat and parse JSON reply."""
        url = f"{self.ollama_url}/api/chat"
        self.get_logger().info(f"[llm_teleop] Calling Ollama /api/chat at {url} with model '{self.model}'...")

        try:
            payload = {
                "model": self.model,
                "messages": [
                    {"role": "user", "content": full_prompt}
                ],
                "stream": False,
            }
            r = requests.post(url, json=payload, timeout=60)
            r.raise_for_status()
            data = r.json()
            content = data["message"]["content"]
            self.get_logger().info(f"[llm_teleop] Ollama raw reply: {content}")

            # Try to parse JSON from the reply
            motion = json.loads(content)
            return {
                "linear": float(motion.get("linear", 0.0)),
                "angular": float(motion.get("angular", 0.0)),
                "duration": float(motion.get("duration", 0.0)),
            }

        except Exception as e:
            self.get_logger().error(f"[llm_teleop] LLM call failed: {e}; using safe stop motion.")
            return {"linear": 0.0, "angular": 0.0, "duration": 0.0}

    def _execute_motion(self, motion):
        linear = motion.get("linear", 0.0)
        angular = motion.get("angular", 0.0)
        duration = motion.get("duration", 0.0)

        self.get_logger().info(
            f"[llm_teleop] Executing motion: linear={linear:.3f} m/s, "
            f"angular={angular:.3f} rad/s, duration={duration:.2f} s"
        )

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        # Send command at 10 Hz
        if duration <= 0.0:
            self.get_logger().info("[llm_teleop] Duration <= 0.0, not sending any motion.")
            # Just ensure robot is stopped
            self.cmd_pub.publish(Twist())
            return

        rate = self.create_rate(10.0)
        steps = int(duration * 10.0)
        for _ in range(steps):
            self.cmd_pub.publish(twist)
            rate.sleep()

        # Stop the robot
        self.cmd_pub.publish(Twist())
        self.get_logger().info("[llm_teleop] Motion done; published stop command.")


def main(args=None):
    rclpy.init(args=args)
    node = LlmTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    # rclpy.shutdown() is called inside _run_once when finished.

