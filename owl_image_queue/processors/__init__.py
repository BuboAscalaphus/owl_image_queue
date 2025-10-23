from .base import BaseProcessor
from .noop import NoOpProcessor
from .stats import StatsProcessor

REGISTRY = {
    'noop': NoOpProcessor,
    'stats': StatsProcessor,
}
