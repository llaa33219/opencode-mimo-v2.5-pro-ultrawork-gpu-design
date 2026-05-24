class Netlist:
    def __init__(self):
        self.name = ""
        self._libraries = []

    def create_library(self):
        lib = Library()
        self._libraries.append(lib)
        return lib

class Library:
    def __init__(self):
        self.name = ""
        self._cells = []

    def create_cell(self):
        cell = Cell()
        self._cells.append(cell)
        return cell

class Cell:
    def __init__(self):
        self.name = ""
        self._ports = []
        self._nets = []
        self.children = []

    def create_port(self):
        port = Port()
        self._ports.append(port)
        return port

    def create_net(self):
        net = Net()
        self._nets.append(net)
        return net

    def create_child(self):
        inst = Instance()
        self.children.append(inst)
        return inst

    def get_ports(self, name):
        return [p for p in self._ports if p.name == name]

class Port:
    def __init__(self):
        self.name = ""
        self.direction = None
        self.is_scalar = True
        self._pins = []

    def create_pins(self, count):
        for _ in range(count):
            self._pins.append(Pin())

    @property
    def pins(self):
        return self._pins

class Pin:
    def __init__(self):
        self._net = None

class Net:
    def __init__(self):
        self.name = ""
        self._pins = []

    def connect_pin(self, pin):
        self._pins.append(pin)

class Instance:
    def __init__(self):
        self.name = ""
        self.reference = None

IN = "IN"
OUT = "OUT"
INOUT = "INOUT"
UNDEFINED = "UNDEFINED"

def compose(netlist, filepath):
    with open(filepath, "w") as f:
        f.write(f"(edif {netlist.name}\n)")
