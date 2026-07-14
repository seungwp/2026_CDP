import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import numpy as np
import cv2
import hailo

import time
import queue
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

# ROS2 통신 규격(QoS) 모듈
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Override to use parse_known_args() instead of parse_args()
from hailo_apps_infra.hailo_rpi_common import get_default_parser as original_get_default_parser
import hailo_apps_infra.hailo_rpi_common as common
def get_default_parser_ros2(*args, **kwargs):
    parser = original_get_default_parser(*args, **kwargs)
    parser.parse_args = lambda: parser.parse_known_args()[0]
    return parser
common.get_default_parser = get_default_parser_ros2

from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp

from hailo_apps_infra.gstreamer_helper_pipelines import (
    INFERENCE_PIPELINE,
    INFERENCE_PIPELINE_WRAPPER,
    TRACKER_PIPELINE,
    DISPLAY_PIPELINE,
    USER_CALLBACK_PIPELINE,
)
from hailo_apps_infra.gstreamer_app import disable_qos
from stella_hailo_rpi5_ros2_examples.color_config import get_bbox_color

processed_frame_queue = queue.Queue(maxsize=10)

OBSTACLE_LABELS = {"person", "car", "truck", "bus", "bicycle", "motorcycle"}
OBSTACLE_CONFIDENCE_THRESHOLD = 0.5
obstacle_state = {"detected": False}

class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.new_variable = 42

    def new_function(self):
        return "The meaning of life is:"

def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    user_data.increment()
    format, width, height = get_caps_from_pad(pad)
    frame = None
    
    if format is not None and width is not None and height is not None:
        frame = get_numpy_from_buffer(buffer, format, width, height)

    if frame is not None:
        roi = hailo.get_roi_from_buffer(buffer)
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

        obstacle_found = False
        for detection in detections:
            label = detection.get_label()
            bbox = detection.get_bbox()
            confidence = detection.get_confidence()
            color = get_bbox_color(label)

            if label in OBSTACLE_LABELS and confidence >= OBSTACLE_CONFIDENCE_THRESHOLD:
                obstacle_found = True

            try:
                x1 = round(bbox.xmin() * width)
                y1 = round(bbox.ymin() * height)
                x2 = round(bbox.xmax() * width)
                y2 = round(bbox.ymax() * height)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 1)
                cv2.putText(frame, f"{label}", ((int(x1)+5), (int(y1)+10)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            except AttributeError:
                continue

        obstacle_state["detected"] = obstacle_found

        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        try:
            processed_frame_queue.put_nowait(frame)
        except queue.Full:
            processed_frame_queue.get_nowait()
            processed_frame_queue.put_nowait(frame)

    return Gst.PadProbeReturn.OK

# [복구됨] 스레드 주입을 모두 제거하고 100% 원본 안전한 상태로 되돌림
class ROS2DetectionApp(GStreamerDetectionApp):
    def get_pipeline_string(self):
        source_pipeline = (
            "appsrc name=app_source is-live=true format=GST_FORMAT_TIME "
            "! videoconvert ! video/x-raw, format=RGB, width=640, height=480 "
            "! queue name=source_queue ! videoscale "
        )

        detection_pipeline = INFERENCE_PIPELINE(
            hef_path=self.hef_path,
            post_process_so=self.post_process_so,
            batch_size=self.batch_size,
            config_json=self.labels_json,
            additional_params=self.thresholds_str
        )
        detection_pipeline_wrapper = INFERENCE_PIPELINE_WRAPPER(detection_pipeline)
        tracker_pipeline = TRACKER_PIPELINE(class_id=1)
        user_callback_pipeline = USER_CALLBACK_PIPELINE()
        display_pipeline = DISPLAY_PIPELINE(video_sink="fakesink", sync=self.sync, show_fps=self.show_fps)

        pipeline_string = (
            f"{source_pipeline} ! "
            f"{detection_pipeline_wrapper} ! "
            f"{tracker_pipeline} ! "
            f"{user_callback_pipeline} ! "
            f"{display_pipeline}"
        )
        return pipeline_string

    def shutdown(self, signum=None, frame=None):
        self.pipeline.set_state(Gst.State.NULL)
        GLib.idle_add(self.loop.quit)

    def run(self):
        self.source_type = "ros2"
        Gst.init(None)
        pipeline_string = self.get_pipeline_string()
        try:
            self.pipeline = Gst.parse_launch(pipeline_string)
        except Exception as e:
            print(f"[ERROR] Pipeline parsing failed: {e}")
            exit(1)
        
        self.loop = GLib.MainLoop()
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.bus_call, self.loop)
        
        if not self.options_menu.disable_callback:
            identity = self.pipeline.get_by_name("identity_callback")
            if identity is not None:
                identity_pad = identity.get_static_pad("src")
                identity_pad.add_probe(Gst.PadProbeType.BUFFER, self.app_callback, self.user_data)
        
        disable_qos(self.pipeline)
        self.pipeline.set_state(Gst.State.PAUSED)
        new_latency = self.pipeline_latency * Gst.MSECOND
        self.pipeline.set_latency(new_latency)
        self.pipeline.set_state(Gst.State.PLAYING)

class HailoDetectionNode(Node):
    def __init__(self):
        super().__init__("hailo_ros2_detection_node")
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            qos_profile 
        )
        
        self.publisher_ = self.create_publisher(Image, "detection_image", 10)
        self.obstacle_pub = self.create_publisher(Bool, "/perception/obstacle_detected", 10)
        
        # 10Hz로 무조건 상태값 발행 (False든 True든)
        self.create_timer(0.1, self.publish_obstacle_state)
        self.bridge = CvBridge()
        self.get_logger().info("Hailo Detection Node starting...")

        self.frame_count = 0
        self.last_frame_time = time.time()
        self.MAX_FRAME_COUNT = 1000000

        class DummyArgs:
            def __init__(self):
                self.input = ""
                self.use_frame = True
                self.show_fps = False
                self.arch = None
                self.hef_path = None
                self.disable_sync = True
                self.disable_callback = False
                self.dump_dot = False
                self.labels_json = None
        dummy_args = DummyArgs()

        if "TAPPAS_POST_PROC_DIR" not in os.environ:
            os.environ["TAPPAS_POST_PROC_DIR"] = os.getcwd()

        self.user_data = user_app_callback_class()
        self.detection_app = ROS2DetectionApp(app_callback, self.user_data)
        self.detection_app.options_menu = dummy_args
        self.detection_app.video_width = 640
        self.detection_app.video_height = 480
        self.detection_app.video_source = "ros2"
        
        # 순정 앱 실행
        self.detection_app.run()

        self.appsrc = self.detection_app.pipeline.get_by_name("app_source")
        if self.appsrc is not None:
            caps = Gst.Caps.from_string("video/x-raw, format=RGB, width=640, height=480, framerate=30/1")
            self.appsrc.set_property("caps", caps)

        self.create_timer(0.1, self.publish_processed_frame)

    def image_callback(self, msg):
        # [핵심 방어] 무한 대기(Deadlock) 원천 차단
        # NPU로 가는 영상을 1초에 약 10장으로 제한하여, 버퍼가 터지거나 막히는 것을 방지합니다.
        current_time = time.time()
        if current_time - self.last_frame_time < 0.1:
            return  
        self.last_frame_time = current_time

        if self.appsrc is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception:
            return

        self.frame_count += 1
        if self.frame_count >= self.MAX_FRAME_COUNT:
            self.frame_count = 1

        cv_image = cv2.resize(cv_image, (640, 480))

        data = cv_image.tobytes()
        buf = Gst.Buffer.new_wrapped(data)
        buf.pts = self.frame_count * Gst.util_uint64_scale_int(1, Gst.SECOND, 30)
        buf.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, 30)
        
        try:
            self.appsrc.emit("push-buffer", buf)
        except Exception:
            pass

    def publish_obstacle_state(self):
        msg = Bool()
        msg.data = obstacle_state["detected"]
        self.obstacle_pub.publish(msg)

    def publish_processed_frame(self):
        try:
            frame = processed_frame_queue.get_nowait()
        except queue.Empty:
            return

        if frame is not None:
            try:
                ros_img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher_.publish(ros_img)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = HailoDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == "__main__":
    main()