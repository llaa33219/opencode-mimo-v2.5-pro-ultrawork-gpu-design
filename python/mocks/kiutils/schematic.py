class Schematic:
    libSymbols = None
    sheets = None
    symbols = None
    labels = None
    wires = None
    junctions = None
    noconnects = None
    sheetInstances = None

    @classmethod
    def create_new(cls):
        inst = cls()
        inst.libSymbols = []
        inst.sheets = []
        inst.symbols = []
        inst.labels = []
        inst.wires = []
        inst.junctions = []
        inst.noconnects = []
        inst.sheetInstances = []
        return inst

    def to_file(self, filepath):
        with open(filepath, "w") as f:
            f.write("(kicad_sch (version 20211123)\n)")
