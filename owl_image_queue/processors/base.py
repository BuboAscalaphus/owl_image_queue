class BaseProcessor:
    name = 'base'
    def setup(self, node):
        self.node = node
    def process(self, image_path: str) -> dict:
        raise NotImplementedError
