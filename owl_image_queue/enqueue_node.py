import os, time, queue, glob
from pathlib import PurePath
import rclpy
from rclpy.node import Node
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

from .sqlite_queue import ImageQueue


class _Handler(FileSystemEventHandler):
    """Watchdog handler that detects finished files and hands paths to the node thread via a Queue."""
    def __init__(self, node: Node, base_dir: str, pattern: str, inbox: queue.Queue, settle_ms=120):
        super().__init__()
        self.node = node
        self.base_dir = os.path.abspath(base_dir)
        self.pattern = pattern              # e.g., **/*.jpg
        self.inbox = inbox                  # thread-safe handoff to ROS thread
        self.settle_ms = settle_ms          # small delay to avoid partial writes

    def _maybe_enqueue(self, path: str):
        # Match '**/*.jpg' correctly across subdirectories
        rel = os.path.relpath(path, self.base_dir)
        if not PurePath(rel).match(self.pattern):
            return
        if not os.path.isfile(path):
            return
        # Wait briefly to avoid partial writes
        try:
            s0 = os.path.getsize(path)
            time.sleep(self.settle_ms / 1000.0)
            s1 = os.path.getsize(path)
            if s1 != s0:
                time.sleep(self.settle_ms / 1000.0)
        except FileNotFoundError:
            return
        # Do NOT touch SQLite here; just queue the path for the node thread
        try:
            self.inbox.put_nowait(path)
        except queue.Full:
            self.node.get_logger().warn(f"inbox full; dropping path: {path}")

    def on_created(self, event):
        if not event.is_directory:
            self._maybe_enqueue(event.src_path)

    def on_moved(self, event):
        if not event.is_directory:
            self._maybe_enqueue(event.dest_path)

    def on_modified(self, event):
        if not event.is_directory:
            self._maybe_enqueue(event.src_path)


class EnqueueNode(Node):
    def __init__(self):
        super().__init__('image_enqueue')
        self.base_dir = self.declare_parameter('base_dir', '/home/dev/bags').get_parameter_value().string_value
        self.pattern  = self.declare_parameter('acquisition_glob', '**/*.jpg').get_parameter_value().string_value

        # SQLite queue lives under base_dir
        db_path = os.path.join(self.base_dir, 'queue.db')
        self.q = ImageQueue(db_path)

        # Thread-safe inbox for paths coming from watchdog thread
        self._inbox = queue.Queue(maxsize=10000)
        self._drain_timer = self.create_timer(0.2, self._drain_inbox)

        # Start watchdog (recursive)
        self._observer = Observer(timeout=2.0)
        handler = _Handler(self, self.base_dir, self.pattern, self._inbox)
        self._observer.schedule(handler, path=self.base_dir, recursive=True)
        self._observer.start()

        self.get_logger().info(
            f"Watching (inotify): base_dir={self.base_dir} pattern={self.pattern} queue_db={db_path}"
        )

    def _drain_inbox(self):
        """Runs in the ROS node thread — safe to write to SQLite here."""
        drained = 0
        while not self._inbox.empty() and drained < 1000:
            path = self._inbox.get_nowait()
            try:
                self.q.enqueue(path)
                drained += 1
            except Exception as e:
                self.get_logger().warn(f"enqueue failed for {path}: {e}")
        if drained:
            self.get_logger().info(f"Enqueued {drained} new images. Queue size={self.q.size()}")

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


