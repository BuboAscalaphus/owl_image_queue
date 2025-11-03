from .base import BaseProcessor
from .noop import NoopProcessor
from .stats import StatsProcessor
from .ultralytics_processor import UltralyticsProcessor  # <-- add this

REGISTRY = {
    "noop": NoopProcessor,
    "stats": StatsProcessor,
    "ultralytics": UltralyticsProcessor,  # <-- add this
}