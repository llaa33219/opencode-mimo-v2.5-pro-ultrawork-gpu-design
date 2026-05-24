import os
import tempfile
import unittest

from netlist_gen.edif_exporter import EDIFExporter, EDIFExporterError
from netlist_gen.verilog_parser import Module, Port, ModuleInstance


class TestEDIFExporter(unittest.TestCase):
    def test_init(self):
        exp = EDIFExporter("test_design")
        self.assertEqual(exp.design_name, "test_design")
        self.assertIsNotNone(exp.netlist)

    def test_add_library(self):
        exp = EDIFExporter("test")
        lib = exp.add_library("work")
        self.assertIsNotNone(lib)
        self.assertEqual(lib.name, "work")

    def test_add_cell(self):
        exp = EDIFExporter("test")
        cell = exp.add_cell("work", "and_gate")
        self.assertIsNotNone(cell)
        self.assertEqual(cell.name, "and_gate")

    def test_add_port(self):
        exp = EDIFExporter("test")
        port = exp.add_port("work", "and_gate", "a", "input")
        self.assertIsNotNone(port)
        self.assertEqual(port.name, "a")

    def test_add_instance(self):
        exp = EDIFExporter("test")
        exp.add_cell("work", "parent")
        exp.add_cell("work", "child")
        inst = exp.add_instance("work", "parent", "inst1", "work", "child")
        self.assertIsNotNone(inst)
        self.assertEqual(inst.name, "inst1")

    def test_import_module(self):
        exp = EDIFExporter("test")
        mod = Module(
            name="top",
            ports=[
                Port(name="clk", direction="input"),
                Port(name="out", direction="output"),
            ],
            instances=[
                ModuleInstance(
                    module_type="sub",
                    instance_name="u1",
                    port_connections={"in": "clk", "out": "out"},
                )
            ],
        )
        exp.import_module(mod)
        self.assertIn(("work", "top"), exp._cells)

    def test_export(self):
        exp = EDIFExporter("test")
        mod = Module(
            name="top",
            ports=[Port(name="a", direction="input")],
        )
        exp.import_module(mod)
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "test.edif")
            exp.export(path)
            self.assertTrue(os.path.isfile(path))


if __name__ == "__main__":
    unittest.main()
