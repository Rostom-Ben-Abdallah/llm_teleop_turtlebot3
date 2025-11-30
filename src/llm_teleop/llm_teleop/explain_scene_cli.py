import json
import requests

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class ExplainSceneNode(Node):
    def __init__(self):
        super().__init__('explain_scene_cli')

        # Parameters (can be overridden with --ros-args -p ...)
        self.declare_parameter('describe_service', 'describe_scene')
        self.declare_parameter('ollama_url', 'http://127.0.0.1:11434')
        self.declare_parameter('model', 'llama3.1:8b')

        self.describe_service_name = (
            self.get_parameter('describe_service').get_parameter_value().string_value
        )
        self.ollama_url = self.get_parameter('ollama_url').get_parameter_value().string_value
        self.model = self.get_parameter('model').get_parameter_value().string_value

        self.get_logger().info(f'Using describe_service="{self.describe_service_name}"')
        self.get_logger().info(f'Ollama URL="{self.ollama_url}", model="{self.model}"')

        # Create client to YOLO scene service
        self.cli = self.create_client(Trigger, self.describe_service_name)
        self.req = Trigger.Request()

    def call_describe(self):
        # Wait for /describe_scene to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for service {self.describe_service_name}...')

        self.get_logger().info('Calling /describe_scene...')
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            self.get_logger().error('Failed to call describe_scene')
            return None

        resp = future.result()
        self.get_logger().info(f'/describe_scene raw message:\n{resp.message}')
        return resp.message

    def call_ollama(self, scene_json_str: str) -> str:
        """Send scene JSON to Ollama and get a natural-language explanation."""
        try:
            scene = json.loads(scene_json_str)
        except Exception:
            # If it isn't valid JSON, wrap it
            scene = {"raw": scene_json_str}

        prompt = (
            "You are a robot perception assistant.\n"
            "You receive a JSON description of detected objects from a YOLO model.\n"
            "Explain in clear, short natural language what the robot sees.\n\n"
            f"JSON:\n{json.dumps(scene, indent=2)}\n\n"
            "Now write a concise explanation for a human operator."
        )

        payload = {
            "model": self.model,
            "messages": [
                {"role": "user", "content": prompt}
            ],
            "stream": False,
        }

        url = f"{self.ollama_url.rstrip('/')}/api/chat"
        self.get_logger().info(f'Calling Ollama at {url} ...')
        r = requests.post(url, json=payload, timeout=60)
        r.raise_for_status()
        data = r.json()
        content = data["message"]["content"]
        return content

    def run(self):
        scene_str = self.call_describe()
        if scene_str is None:
            return

        try:
            explanation = self.call_ollama(scene_str)
            self.get_logger().info("\n=== LLM Explanation ===\n" + explanation + "\n")
            print("\n=== LLM Explanation ===\n")
            print(explanation)
            print("\n=======================\n")
        except Exception as e:
            self.get_logger().error(f'Failed to call Ollama: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ExplainSceneNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
