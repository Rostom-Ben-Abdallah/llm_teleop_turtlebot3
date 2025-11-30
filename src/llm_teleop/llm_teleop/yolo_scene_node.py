import json
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from cv_bridge import CvBridge, CvBridgeError
import cv2

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class YoloSceneNode(Node):
    def __init__(self):
        super().__init__('yolo_scene')

        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf', 0.4)
        self.declare_parameter('device', 'cpu')          # 'cpu' or 'cuda:0'
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('debug_image_topic', '/camera/yolo_debug')

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf = float(self.get_parameter('conf').get_parameter_value().double_value)
        self.device = self.get_parameter('device').get_parameter_value().string_value or 'cpu'
        self.imgsz = int(self.get_parameter('imgsz').get_parameter_value().integer_value)
        debug_image_topic = self.get_parameter('debug_image_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.last_scene = {"objects": [], "ts": 0.0}

        # Load YOLO
        if YOLO is None:
            self.get_logger().error("Ultralytics YOLO not available. Run: pip install ultralytics")
            self.model = None
        else:
            self.get_logger().info(
                f"[yolo_scene] Loading YOLO model '{model_path}' on device '{self.device}' "
                f"with conf={self.conf}, imgsz={self.imgsz}"
            )
            try:
                self.model = YOLO(model_path)
            except Exception as e:
                self.get_logger().error(f"[yolo_scene] Failed to load YOLO model {model_path}: {e}")
                self.model = None

        # Subscriber to camera
        self.sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        # Debug image publisher
        self.debug_pub = self.create_publisher(Image, debug_image_topic, 10)

        # Service to get last scene summary
        self.srv = self.create_service(Trigger, 'describe_scene', self.describe_scene_cb)

        self.get_logger().info(
            f"[yolo_scene] Subscribed to {camera_topic}, debug images on {debug_image_topic}, "
            "service /describe_scene ready."
        )

        # Rate-limit detection
        self.last_infer_time = 0.0
        self.infer_interval = 0.5  # seconds

    def image_callback(self, msg: Image):
        # throttle
        now = time.time()
        if now - self.last_infer_time < self.infer_interval:
            return
        self.last_infer_time = now

        if self.model is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f"[yolo_scene] CvBridge error: {e}")
            return

        # Run YOLO
        try:
            results = self.model(
                frame,
                imgsz=self.imgsz,
                conf=self.conf,
                device=self.device
            )
        except Exception as e:
            self.get_logger().error(f"[yolo_scene] YOLO inference error: {e}")
            return

        objects = []
        vis = frame.copy()

        for r in results:
            if not hasattr(r, 'boxes') or r.boxes is None:
                continue
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                xyxy = box.xyxy[0].tolist()
                label = self.model.names.get(cls_id, str(cls_id)) if hasattr(self.model, 'names') else str(cls_id)

                x1, y1, x2, y2 = map(int, xyxy)
                objects.append({
                    "label": label,
                    "conf": conf,
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                })

                # draw on debug image
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(vis, f"{label} {conf:.2f}", (x1, max(0, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        self.last_scene = {"objects": objects, "ts": now}

        if objects:
            self.get_logger().info(
                "[yolo_scene] Detected: " +
                ", ".join(f"{o['label']} ({o['conf']:.2f})" for o in objects)
            )
        else:
            self.get_logger().info("[yolo_scene] No objects above conf threshold.")

        # publish debug image
        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            dbg_msg.header = msg.header
            self.debug_pub.publish(dbg_msg)
        except CvBridgeError as e:
            self.get_logger().warn(f"[yolo_scene] Failed to publish debug image: {e}")

    def describe_scene_cb(self, request, response):
        scene = self.last_scene
        objs = scene["objects"]
        if not objs:
            summary = "No objects currently detected."
        else:
            parts = [f"{o['label']} ({o['conf']:.2f})" for o in objs]
            summary = "Detected: " + ", ".join(parts)

        payload = {
            "summary": summary,
            "objects": objs,
            "ts": scene["ts"],
        }
        response.success = True
        response.message = json.dumps(payload, indent=2)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloSceneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
