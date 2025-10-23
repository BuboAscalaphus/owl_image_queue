from .base import BaseProcessor
from PIL import Image, ImageStat

class StatsProcessor(BaseProcessor):
    name = 'stats'
    def process(self, image_path: str) -> dict:
        with Image.open(image_path) as im:
            im = im.convert('RGB')
            stat = ImageStat.Stat(im)
            return {
                'width': im.width,
                'height': im.height,
                'mean_rgb': [float(x) for x in stat.mean],
                'rms_rgb': [float(x) for x in stat.rms],
            }
