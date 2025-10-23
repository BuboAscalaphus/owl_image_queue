import rclpy, os, time, fnmatch, threading
from rclpy.node import Node
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

from .sqlite_queue import ImageQueue

def _is_final_write_event(event):
    # Some writers do tmp + rename, others write in place then close.
    # We accept both CREATED/MOVED and also detect CLOSE_WRITE-like by size-stabilization below.
    return True

class _Handler(FileSystemEventHandler):
    def __init__(self, node, base_dir, pattern, q: ImageQueue, settle_ms=120):
        super().__init__()
        self.node = node
        self.base_dir = os.path.abspath(base_dir)
        self.pattern = pattern
        self.q = q
        self.settle_ms = settle_ms

    def _maybe_enqueue(self, path):
        # match glob against filename path relative to base_dir
        rel = os.path.relpath(path, self.base_dir)
        if not fnmatch.fnmatch(rel, self.pattern):
            return
        if not os.path.isfile(path):
            return
        # wait a short “settle” time to avoid partial writes
        t0 = os.path.getsize(path)
        time.sleep(self.settle_ms / 1000.0)
        try:
            t1 = os.path.getsize(path)
        except FileNotFoundError:
            return
        if t1 != t0:
            # grew during settle window; wait once more
            time.sleep(self.settle_ms / 1000.0)
        try:
            self.q.enqueue(path)
            self.node.get_logger().info(f"Enqueued {path} (queue size={self.q.size()})")
        except Exception as e:
            self.node.get_logger().warn(f"enqueue failed for {path}: {e}")

    def on_created(self, event):
        if event.is_directory: 
            return
        self._maybe_enqueue(event.src_path)

    def on_moved(self, event):
        if event.is_directory:
            return
        # MOVED_TO is common for atomic writers
        self._maybe_enqueue(event.dest_path)

    def on_modified(self, event):
        # Some writers don't rename; they write then close.
        if event.is_directory:
            return
        self._maybe_enqueue(event.src_path)

class EnqueueNode(Node):
    def __init__(self):
        super().__init__('image_enqueue')
        self.base_dir = self.declare_parameter('base_dir', '/home/dev/bags').get_parameter_value().string_value
        self.pattern  = self.declare_parameter('acquisition_glob', '**/*.jpg').get_parameter_value().string_value
        db_path = os.path.join(self.base_dir, 'queue.db')
        self.q = ImageQueue(db_path)

        # Start watchdog observer (recursive)
        self._observer = Observer(timeout=2.0)
        handler = _Handler(self, self.base_dir, self.pattern, self.q)
        self._observer.schedule(handler, path=self.base_dir, recursive=True)
        self._observer.start()

        self.get_logger().info(f"Watching (inotify): base_dir={self.base_dir} pattern={self.pattern} queue_db={db_path}")

    def destroy_node(self):
        try:
            self._observer.stop()
            self._observer.join(timeout=3)
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EnqueueNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
