class HierarchicalSheet:
    def __init__(self):
        self.uuid = ""
        self.properties = []
        self.positions = []
        self.stroke = None
        self.fill = None
        self.pins = []

class HierarchicalSheetInstance:
    pass

class Junction:
    def __init__(self):
        self.uuid = ""
        self.position = None

class LocalLabel:
    def __init__(self):
        self.uuid = ""
        self.text = ""
        self.position = None
        self.effects = {}

class NoConnect:
    def __init__(self):
        self.uuid = ""
        self.position = None

class SchematicSymbol:
    def __init__(self):
        self.uuid = ""
        self.libId = ""
        self.properties = []
        self.position = None
        self.inBom = True
        self.onBoard = True

class Wire:
    def __init__(self):
        self.uuid = ""
        self.coords = {}
        self.stroke = None
