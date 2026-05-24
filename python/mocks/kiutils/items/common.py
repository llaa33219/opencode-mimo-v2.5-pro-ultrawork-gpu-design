class Property:
    def __init__(self, key="", value=""):
        self.key = key
        self.value = value

class Position:
    def __init__(self, X=0.0, Y=0.0, angle=0.0):
        self.X = X
        self.Y = Y
        self.angle = angle

class Stroke:
    def __init__(self, width=0.0):
        self.width = width
