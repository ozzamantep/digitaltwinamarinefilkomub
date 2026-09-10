class YOLO:
    def __init__(self, model: str = ''):
        self.names = {0: 'gate', 1: 'orange flare', 2: 'blue flare', 3: 'red flare', 4: 'yellow flare', 5: 'drum', 6: 'obstacle'}
    def __call__(self, img, *args, **kwargs):
        class BoxResult:
            boxes = []
            def plot(self): return img
        return [BoxResult()]
