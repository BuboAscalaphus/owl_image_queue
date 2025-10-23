from .base import BaseProcessor
import os

class NoOpProcessor(BaseProcessor):
    name = 'noop'
    def process(self, image_path: str) -> dict:
        return {'size_bytes': os.path.getsize(image_path)}
