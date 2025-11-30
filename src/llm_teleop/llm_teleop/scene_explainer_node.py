import json
import requests

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class SceneExplainer(Node):
    def __init__(self):
        super().__init__('scene_explainer')

        # Parameters
        self.declare_parameter('describe_service', 'describe_scene')
        self.declare_parameter('ollama_url', 'http://127.0.0.1:11434')
        self.declare_parameter('model', 'llama3.1:8b')

        self.describe_service_name = (
            self.get_parameter('describe_service').get_parameter_value().string_value
        )
        self.ollama_url = self.get_parameter('ollama_url').get_parameter_value().string_value
        self.model = self.get_parameter('model').get_parameter_value().string_value

        # Client for YOLO scene description
        self.describe_cli = self.create_client(Trigger, self.describe_service_name)

        # Service that user will call to get NL summary
        self.explain_srv = self.create_service(
            Trigger, 'explain_scene', self.explain_scene_cb
        )

        self.get_logger().info(
            f"[scene_explainer] Using describe_service='{self.describe_service_name}', "
            f"Ollama='{self.ollama_url}', model='{self.model}'"
        )

    def explain_scene_cb(self, request, response):
        # 1) Call describe_scene from yolo_scene_node
        if not self.describe_cli.wait_for_service(timeout_sec=3.0):
            msg = f"Service '{self.describe_service_name}' not available."
            self.get_logger().error(msg)
            response.success = False
            response.message = msg
            return response

        self.get_logger().info(
            f"[scene_explainer] Calling {self.describe_service_name}..."
        )
        future = self.describe_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            msg = "Failed to call describe_scene."
            self.get_logger().error(msg)
            response.success = False
            response.message = msg
            return response

        scene_resp = future.result()
        scene_json_str = scene_resp.message  # yolo_scene returns JSON here

        # 2) Try to parse JSON (for safety)
        try:
            scene_data = json.loads(scene_json_str)
        except json.JSONDecodeError:
            # Fallback if something unexpected
            self.get_logger().warn(
                "[scene_explainer] describe_scene did not return valid JSON; using raw string."
            )
            scene_data = {"raw": scene_json_str}

        # 3) Build prompt for Ollama
        sys_prompt = (
            "You are a robot assistant describing what the camera sees. "
            "You will receive JSON with detected objects. "
            "Respond with 1–2 short sentences, friendly and clear."
        )

        user_prompt = (
            "Here is the current scene as JSON detections:\n\n"
            f"```json\n{json.dumps(scene_data, indent=2)}\n```\n\n"
            "Describe what the robot is seeing."
        )

        nl_summary = None

        # 4) Call Ollama /api/chat
        try:
            url = f"{self.ollama_url}/api/chat"
            self.get_logger().info(
                f"[scene_explainer] Calling Ollama at {url} with model '{self.model}'..."
            )

            payload = {
                "model": self.model,
                "messages": [
                    {"role": "system", "content": sys_prompt},
                    {"role": "user", "content": user_prompt},
                ],
                "stream": False,
            }

            r = requests.post(url, json=payload, timeout=30)
            r.raise_for_status()
            j = r.json()
            nl_summary = j.get("message", {}).get("content", "").strip()
            self.get_logger().info(f"[scene_explainer] LLM summary: {nl_summary!r}")
        except Exception as e:
            self.get_logger().error(f"[scene_explainer] Ollama call failed: {e}")

        # 5) Build final response
        if not nl_summary:
            # Fallback: just reuse YOLO summary field if present
            summary = scene_data.get("summary", "No natural language description available.")
        else:
            summary = nl_summary

        response.success = True
        response.message = summary
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SceneExplainer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
