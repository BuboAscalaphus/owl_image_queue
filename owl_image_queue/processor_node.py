import rclpy, os, json, csv, math, datetime
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

# Try owl_msgs/BoolStamped first (owl_sense convention), fallback to std_msgs/Bool
try:
    from owl_msgs.msg import BoolStamped as StatusMsg
except Exception:
    from std_msgs.msg import Bool as StatusMsg  # type: ignore

from .sqlite_queue import ImageQueue
from .processors import REGISTRY

def _abs_speed(msg: TwistStamped) -> float:
    v = msg.twist.linear
    return float(math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z))

class ProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor')
        gp = self.declare_parameter
        self.base_dir = gp('base_dir', '/home/dev/bags').get_parameter_value().string_value
        self.status_topic = gp('status_topic', '/enabled').get_parameter_value().string_value
        self.process_when = gp('process_when', 'false').get_parameter_value().string_value.lower().strip()
        self.velocity_topic = gp('velocity_topic', '/velocity').get_parameter_value().string_value
        self.min_speed_mps = float(gp('min_speed_mps', -1.0).get_parameter_value().double_value)
        self.processor_name = gp('processor_name', 'noop').get_parameter_value().string_value
        self.results_path = gp('results_path', '').get_parameter_value().string_value
        self.results_format = gp('results_format', 'jsonl').get_parameter_value().string_value
        self.max_batch = int(gp('max_batch', 25).get_parameter_value().integer_value)

        if not self.results_path:
            out_dir = os.path.join(self.base_dir, 'processed')
            os.makedirs(out_dir, exist_ok=True)
            ext = 'jsonl' if self.results_format == 'jsonl' else 'csv'
            self.results_path = os.path.join(out_dir, f'results.{ext}')
        else:
            os.makedirs(os.path.dirname(self.results_path) or '.', exist_ok=True)

        db_path = os.path.join(self.base_dir, 'queue.db')
        self.q = ImageQueue(db_path)

        if self.processor_name not in REGISTRY:
            raise RuntimeError(f"Unknown processor '{self.processor_name}'. Available: {list(REGISTRY.keys())}")
        self.processor = REGISTRY[self.processor_name]()
        self.processor.setup(self)

        self._status_ok = (self.process_when == 'true')  # default until first message
        self._speed_ok = (self.min_speed_mps < 0)

        # Subscriptions
        self.create_subscription(StatusMsg, self.status_topic, self._on_status, 10)
        if self.min_speed_mps >= 0.0:
            self.create_subscription(TwistStamped, self.velocity_topic, self._on_vel, 10)

        self.timer = self.create_timer(0.5, self._tick)
        self._csv_writer = None
        self._csv_file = None

        self.get_logger().info(
            f"Processor='{self.processor_name}' results={self.results_path} queue_db={db_path} "
            f"status_topic={self.status_topic} process_when={self.process_when} "
            f"velocity_topic={self.velocity_topic} min_speed_mps={self.min_speed_mps}"
        )

    def _on_status(self, msg):
        # BoolStamped or Bool
        val = bool(msg.data) if hasattr(msg, 'data') else bool(getattr(msg, 'data', False))
        self._status_ok = (val and self.process_when == 'true') or ((not val) and self.process_when == 'false')

    def _on_vel(self, msg: TwistStamped):
        self._speed_ok = (_abs_speed(msg) >= self.min_speed_mps)

    def _open_csv(self):
        if self._csv_writer is None:
            exists = os.path.exists(self.results_path)
            self._csv_file = open(self.results_path, 'a', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            if not exists:
                self._csv_writer.writerow(['image_path', 'processed_at', 'processor', 'result_json'])

    def _write_result(self, rec: dict):
        if self.results_format == 'jsonl':
            with open(self.results_path, 'a') as f:
                f.write(json.dumps(rec) + '\n')
        else:
            self._open_csv()
            self._csv_writer.writerow([rec['image_path'], rec['processed_at'], rec['processor'], json.dumps(rec['data'])])
            self._csv_file.flush()

    def _tick(self):
        if not (self._status_ok and self._speed_ok):
            return
        batch = self.q.dequeue_batch(self.max_batch)
        if not batch:
            return
        processed = 0
        for path in batch:
            try:
                data = self.processor.process(path)
                rec = {
                    'image_path': path,
                    'processed_at': datetime.datetime.utcnow().isoformat() + 'Z',
                    'processor': self.processor.name,
                    'data': data,
                }
                self._write_result(rec)
                processed += 1
            except Exception as e:
                self.get_logger().warn(f"processing failed for {path}: {e}")
                try:
                    self.q.enqueue(path)
                except Exception:
                    pass
        self.get_logger().info(f"Processed {processed} images. Queue size now {self.q.size()}")

def main(args=None):
    rclpy.init(args=args)
    node = ProcessorNode()
    rclpy.spin(node)
    if node._csv_file:
        node._csv_file.close()
    node.destroy_node()
    rclpy.shutdown()
