import os
import tempfile
import unittest

from netlist_gen.verilog_parser import (
    Module,
    ModuleInstance,
    Port,
    VerilogParser,
    VerilogParserError,
)


SIMPLE_AND = """
module and_gate (
    input a,
    input b,
    output y
);
    assign y = a & b;
endmodule
"""

MODULE_WITH_INSTANCE = """
module half_adder (
    input a,
    input b,
    output sum,
    output cout
);
    assign sum = a ^ b;
    assign cout = a & b;
endmodule

module full_adder (
    input a,
    input b,
    input cin,
    output sum,
    output cout
);
    wire s1, c1, c2;

    half_adder ha1 (
        .a(a),
        .b(b),
        .sum(s1),
        .cout(c1)
    );

    half_adder ha2 (
        .a(s1),
        .b(cin),
        .sum(sum),
        .cout(c2)
    );

    assign cout = c1 | c2;
endmodule
"""

BUS_MODULE = """
module bus_mux (
    input [7:0] a,
    input [7:0] b,
    input sel,
    output [7:0] y
);
    assign y = sel ? a : b;
endmodule
"""

PARAMETER_MODULE = """
module counter #(parameter WIDTH = 8) (
    input clk,
    input rst,
    output [WIDTH-1:0] count
);
    reg [WIDTH-1:0] cnt;
    always @(posedge clk or posedge rst) begin
        if (rst)
            cnt <= 0;
        else
            cnt <= cnt + 1;
    end
    assign count = cnt;
endmodule
"""


class TestPort(unittest.TestCase):
    def test_scalar_port(self):
        p = Port(name="clk", direction="input")
        self.assertEqual(p.width, 1)
        self.assertIsNone(p.msb)

    def test_bus_port(self):
        p = Port(name="data", direction="output", width=8, msb=7, lsb=0)
        self.assertEqual(p.width, 8)
        self.assertEqual(p.msb, 7)


class TestVerilogParser(unittest.TestCase):
    def setUp(self):
        self.parser = VerilogParser()

    def test_parse_simple_module(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".v", delete=False) as f:
            f.write(SIMPLE_AND)
            path = f.name

        try:
            mods = self.parser.parse_file(path)
            self.assertIn("and_gate", mods)
            mod = mods["and_gate"]
            self.assertEqual(len(mod.ports), 3)
            self.assertEqual(mod.ports[0].name, "a")
            self.assertEqual(mod.ports[0].direction, "input")
        finally:
            os.unlink(path)

    def test_parse_instance(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".v", delete=False) as f:
            f.write(MODULE_WITH_INSTANCE)
            path = f.name

        try:
            self.parser.parse_file(path)
            fa = self.parser.modules.get("full_adder")
            self.assertIsNotNone(fa)
            self.assertEqual(len(fa.instances), 2)
            self.assertEqual(fa.instances[0].module_type, "half_adder")
            self.assertEqual(fa.instances[0].instance_name, "ha1")
            self.assertIn("sum", fa.instances[0].port_connections)
        finally:
            os.unlink(path)

    def test_parse_bus(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".v", delete=False) as f:
            f.write(BUS_MODULE)
            path = f.name

        try:
            mods = self.parser.parse_file(path)
            mod = mods["bus_mux"]
            data_ports = [p for p in mod.ports if p.name in ("a", "b", "y")]
            self.assertTrue(all(p.width == 8 for p in data_ports))
        finally:
            os.unlink(path)

    def test_parse_parameter(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".v", delete=False) as f:
            f.write(PARAMETER_MODULE)
            path = f.name

        try:
            mods = self.parser.parse_file(path)
            mod = mods["counter"]
            self.assertIn("WIDTH", mod.parameters)
            self.assertEqual(mod.parameters["WIDTH"], "8")
        finally:
            os.unlink(path)

    def test_build_hierarchy(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".v", delete=False) as f:
            f.write(MODULE_WITH_INSTANCE)
            path = f.name

        try:
            self.parser.parse_file(path)
            hier = self.parser.build_hierarchy("full_adder")
            self.assertEqual(hier["module"], "full_adder")
            self.assertEqual(len(hier["children"]), 2)
        finally:
            os.unlink(path)

    def test_topological_sort(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".v", delete=False) as f:
            f.write(MODULE_WITH_INSTANCE)
            path = f.name

        try:
            self.parser.parse_file(path)
            order = self.parser.topological_sort()
            self.assertIn("half_adder", order)
            self.assertIn("full_adder", order)
            self.assertLess(order.index("half_adder"), order.index("full_adder"))
        finally:
            os.unlink(path)

    def test_parse_directory(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            with open(os.path.join(tmpdir, "mod1.v"), "w") as f:
                f.write(SIMPLE_AND)
            with open(os.path.join(tmpdir, "mod2.v"), "w") as f:
                f.write(BUS_MODULE)
            mods = self.parser.parse_directory(tmpdir)
            self.assertIn("and_gate", mods)
            self.assertIn("bus_mux", mods)

    def test_missing_file(self):
        with self.assertRaises(VerilogParserError):
            self.parser.parse_file("/nonexistent/path/file.v")

    def test_missing_top_module(self):
        with self.assertRaises(VerilogParserError):
            self.parser.build_hierarchy("nonexistent")


if __name__ == "__main__":
    unittest.main()
