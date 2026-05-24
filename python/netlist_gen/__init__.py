"""
Netlist Generator Package

A Python toolchain for generating KiCad PCB netlists from Verilog module hierarchy.

This package provides tools to:
    - Parse Verilog RTL to extract module hierarchy
    - Generate KiCad 6.0+ schematic files (.kicad_sch)
    - Export EDIF 2 0 0 netlists for EDA tool compatibility
    - Perform component placement optimization

Modules:
    verilog_parser -- Parse Verilog files into module hierarchy
    kicad_generator -- Generate KiCad schematic files
    edif_exporter -- Export EDIF netlists
    placement -- Component placement algorithms

Example:
    from netlist_gen import VerilogParser, KiCadGenerator

    parser = VerilogParser()
    modules = parser.parse_directory("./verilog/rtl")
    hierarchy = parser.build_hierarchy("top_module")

    gen = KiCadGenerator("my_project")
    # ... add modules and connections ...
    gen.save("output.kicad_sch")
"""

__version__ = "0.1.0"
__author__ = "Low-Price GPU Team"
__all__ = [
    "VerilogParser",
    "Module",
    "Port",
    "ModuleInstance",
    "KiCadGenerator",
    "EDIFExporter",
    "PlacementEngine",
    "PlacementResult",
]

from netlist_gen.verilog_parser import VerilogParser, Module, Port, ModuleInstance
from netlist_gen.kicad_generator import KiCadGenerator
from netlist_gen.edif_exporter import EDIFExporter
from netlist_gen.placement import PlacementEngine, PlacementResult
