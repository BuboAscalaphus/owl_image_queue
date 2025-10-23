# owl_image_queue

ROS 2 (Humble) Python package that:
- **Enqueues** each image file saved under your *acquisition* directory (recursively).
- **Processes** only when a **status topic** says it's time (e.g., *outside the row*).
- **Dequeues** processed images and writes results to JSONL or CSV.
- **Modular** processors via a simple plugin interface.

Designed to fit **owl_sense** conventions:
- Uses your existing folders under `/home/dev/bags`.
- Defaults to your topics:
  - `status_topic:=/enabled` *(owl_msgs/BoolStamped)*
  - `velocity_topic:=/velocity` *(geometry_msgs/TwistStamped)*

## Quick start

```bash
# inside the container
cd ~/ws/src
unzip owl_image_queue.zip -d .
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/ws/install/setup.bash

# Launch enqueue + processor
ros2 launch owl_image_queue orchard_processing.launch.py   base_dir:=/home/dev/bags   acquisition_glob:='**/*.jpg'   status_topic:=/enabled   process_when:=false \  # process when status==false (e.g., "outside row")
  velocity_topic:=/velocity   min_speed_mps:=-1.0 \  # set >=0 to also require |v|>=min_speed
  processor_name:=stats   results_format:=jsonl
```

### Key parameters

- `base_dir` (string): acquisition dir to watch. Default `/home/dev/bags`.
- `acquisition_glob` (string): glob for images. Default `'**/*.jpg'`.
- `status_topic` (string): topic with a boolean status *(owl_msgs/BoolStamped or std_msgs/Bool)*. Default `/enabled`.
- `process_when` (string): `'true'` or `'false'`. When to process based on status. Default `'false'`.
- `velocity_topic` (string): optional TwistStamped for speed gating. Default `/velocity`.
- `min_speed_mps` (double): if >=0, require `|v| >= min_speed_mps` to process. `-1` disables. Default `-1`.
- `results_path` (string): output path; default `<base_dir>/processed/results.jsonl`.
- `results_format` (string): `'jsonl'` or `'csv'`. Default `'jsonl'`.
- `processor_name` (string): plugin to use (`noop`, `stats`, or your own). Default `'noop'`.
- `max_batch` (int): max images processed per tick. Default `25`.

### Queue persistence

SQLite DB at `<base_dir>/queue.db` with table `queue(path TEXT PRIMARY KEY, mtime REAL)`.

### Processors

Implement:
```python
class BaseProcessor:
    name = "your_name"
    def setup(self, node): ...
    def process(self, image_path: str) -> dict: ...
```
Register in `processors/__init__.py`. Examples: `noop.py`, `stats.py` (Pillow).

