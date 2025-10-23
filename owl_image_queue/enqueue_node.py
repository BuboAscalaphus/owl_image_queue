import rclpy, os, glob, pathlib
from rclpy.node import Node
from .sqlite_queue import ImageQueue

class EnqueueNode(Node):
    def __init__(self):
        super().__init__('image_enqueue')
        self.base_dir = self.declare_parameter('base_dir', '/home/dev/bags').get_parameter_value().string_value
        self.pattern = self.declare_parameter('acquisition_glob', '**/*.jpg').get_parameter_value().string_value
        db_path = os.path.join(self.base_dir, 'queue.db')
        self.q = ImageQueue(db_path)
        self.get_logger().info(f"Watching: {self.base_dir} pattern={self.pattern} queue_db={db_path}")
        self.timer = self.create_timer(2.0, self.scan)
        self._seen = set()

    def scan(self):
        g = os.path.join(self.base_dir, self.pattern)
        count = 0
        for p in glob.iglob(g, recursive=True):
            if not os.path.isfile(p):
                continue
            if p in self._seen:
                continue
            try:
                self.q.enqueue(p)
                self._seen.add(p)
                count += 1
            except Exception as e:
                self.get_logger().warn(f"enqueue failed for {p}: {e}")
        if count:
            self.get_logger().info(f"Enqueued {count} new images. Queue size={self.q.size()}")

def main(args=None):
    rclpy.init(args=args)
    node = EnqueueNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
