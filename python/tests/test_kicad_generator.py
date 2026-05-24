import os
import tempfile
import unittest

from netlist_gen.kicad_generator import KiCadGenerator, KiCadGeneratorError
from netlist_gen.verilog_parser import Module, Port


class TestKiCadGenerator(unittest.TestCase):
    def test_init(self):
        gen = KiCadGenerator("test_project")
        self.assertEqual(gen.project_name, "test_project")
        self.assertIsNotNone(gen.schematic)

    def test_add_component(self):
        gen = KiCadGenerator("test")
        sym = gen.add_component("U1", "Device:R", "10k", 25.4, 12.7)
        self.assertIsNotNone(sym)
        self.assertEqual(len(gen.schematic.symbols), 1)

    def test_add_module_as_sheet(self):
        gen = KiCadGenerator("test")
        mod = Module(
            name="test_mod",
            ports=[
                Port(name="clk", direction="input"),
                Port(name="rst", direction="input"),
                Port(name="out", direction="output"),
            ],
        )
        sheet = gen.add_module_as_sheet(mod, 0.0, 0.0)
        self.assertIsNotNone(sheet)
        self.assertEqual(len(gen.schematic.sheets), 1)

    def test_connect_nets(self):
        gen = KiCadGenerator("test")
        gen.connect_nets("NET1", [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)])
        self.assertEqual(len(gen.schematic.wires), 2)
        self.assertEqual(len(gen.schematic.labels), 1)

    def test_connect_nets_insufficient_points(self):
        gen = KiCadGenerator("test")
        gen.connect_nets("NET1", [(0.0, 0.0)])
        self.assertEqual(len(gen.schematic.wires), 0)

    def test_add_power_symbols(self):
        gen = KiCadGenerator("test")
        gen.add_power_symbols()
        power_syms = [s for s in gen.schematic.symbols if s.libId.startswith("power:")]
        self.assertEqual(len(power_syms), 2)

    def test_generate_netlist(self):
        gen = KiCadGenerator("test")
        gen.add_component("U1", "Device:R", "10k", 0.0, 0.0)
        netlist = gen.generate_netlist()
        self.assertIn("(export", netlist)
        self.assertIn("U1", netlist)

    def test_save(self):
        gen = KiCadGenerator("test")
        gen.add_component("U1", "Device:R", "10k", 0.0, 0.0)
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "test.kicad_sch")
            gen.save(path)
            self.assertTrue(os.path.isfile(path))


if __name__ == "__main__":
    unittest.main()
