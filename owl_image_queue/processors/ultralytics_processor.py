# ~/ros2_ws/src/owl_image_queue/owl_image_queue/processors/ultralytics_processor.py
import requests
from .base import BaseProcessor

class UltralyticsProcessor(BaseProcessor):
    name = "ultralytics"


    def setup(self, node):
        gp = node.declare_parameter
        self.api_url = gp("ultra_api_url", "http://127.0.0.1:8000/predict").get_parameter_value().string_value
        self.model   = gp("ultra_model",   "/models/finetuned.pt").get_parameter_value().string_value
        self.conf    = float(gp("ultra_conf", 0.25).get_parameter_value().double_value)
        self.iou     = float(gp("ultra_iou",  0.70).get_parameter_value().double_value)
        self.imgsz   = int(gp("ultra_imgsz", 640).get_parameter_value().integer_value)
        # "0" (string) for GPU 0, "" or None for CPU
        dev = gp("ultra_device", "").get_parameter_value().string_value
        self.device  = dev if dev else None
        self.timeout = float(gp("ultra_timeout_sec", 120.0).get_parameter_value().double_value)
        self.session = requests.Session()

    def process(self, image_path: str) -> dict:
        payload = {
            "image_path": image_path,
            "model": self.model,
            "conf": self.conf,
            "iou": self.iou,
            "imgsz": self.imgsz,
            "device": self.device,
        }
        r = self.session.post(self.api_url, json=payload, timeout=self.timeout)
        r.raise_for_status()
        return r.json()
