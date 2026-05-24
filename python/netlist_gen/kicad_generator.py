"""KiCad 6.0+ schematic generator using kiutils."""

from __future__ import annotations

import logging
import os
import uuid
from typing import Dict, List, Optional, Tuple

from kiutils.schematic import Schematic
from kiutils.items.schitems import (
    HierarchicalSheet,
    HierarchicalSheetInstance,
    Junction,
    LocalLabel,
    NoConnect,
    SchematicSymbol,
    Wire,
)
from kiutils.items.common import Position, Property, Stroke
from kiutils.utils import sexpr

from netlist_gen.verilog_parser import Module, Port

logger = logging.getLogger(__name__)

GRID_SIZE = 2.54


class KiCadGeneratorError(Exception):
    pass


class KiCadGenerator:
    def __init__(self, project_name: str) -> None:
        self.project_name = project_name
        self.schematic = Schematic.create_new()
        self.schematic.libSymbols = []
        self.schematic.sheets = []
        self.schematic.symbols = []
        self.schematic.labels = []
        self.schematic.wires = []
        self.schematic.junctions = []
        self.schematic.noconnects = []
        self.schematic.sheetInstances = []
        self._sheet_counter = 0
        self._component_counter = 0
        self._net_registry: Dict[str, str] = {}

    def _uuid(self) -> str:
        return str(uuid.uuid4())

    def _next_sheet_name(self) -> str:
        self._sheet_counter += 1
        return f"Sheet{self._sheet_counter:03d}"

    def _next_ref(self, prefix: str = "U") -> str:
        self._component_counter += 1
        return f"{prefix}{self._component_counter}"

    def _snap(self, value: float) -> float:
        return round(value / GRID_SIZE) * GRID_SIZE

    def add_module_as_sheet(
        self,
        module: Module,
        x: float,
        y: float,
        width: float = 50.8,
        height: float = 38.1,
    ) -> HierarchicalSheet:
        sheet_name = self._next_sheet_name()
        sheet_uuid = self._uuid()

        sheet = HierarchicalSheet()
        sheet.uuid = sheet_uuid
        sheet.properties = [
            Property(key="Sheet name", value=module.name),
            Property(key="Sheet file", value=f"{module.name}.kicad_sch"),
        ]
        sheet.positions = [
            Position(X=x, Y=y, angle=0.0),
            Position(X=x + width, Y=y, angle=0.0),
            Position(X=x + width, Y=y + height, angle=0.0),
            Position(X=x, Y=y + height, angle=0.0),
        ]
        sheet.stroke = Stroke(width=0.1524)
        sheet.fill = sexpr.SexprSerializer(str)
        sheet.fill.type = "none"

        for port in module.ports:
            pin_x = x + (width if port.direction == "output" else 0.0)
            pin_y = y + 5.08 + (module.ports.index(port) * 5.08)
            pin_y = self._snap(pin_y)
            sheet.pins.append(
                {
                    "name": port.name,
                    "position": Position(X=pin_x, Y=pin_y, angle=0.0),
                    "effects": {"font": {"size": (1.27, 1.27)}},
                }
            )

        self.schematic.sheets.append(sheet)
        logger.debug("Added sheet '%s' for module '%s' at (%.2f, %.2f)", sheet_name, module.name, x, y)
        return sheet

    def add_component(
        self,
        ref: str,
        lib_id: str,
        value: str,
        x: float,
        y: float,
        footprint: str = "",
    ) -> SchematicSymbol:
        sym = SchematicSymbol()
        sym.uuid = self._uuid()
        sym.libId = lib_id
        sym.properties = [
            Property(key="Reference", value=ref),
            Property(key="Value", value=value),
        ]
        if footprint:
            sym.properties.append(Property(key="Footprint", value=footprint))
        sym.position = Position(X=x, Y=y, angle=0.0)

        self.schematic.symbols.append(sym)
        logger.debug("Added component %s (%s) at (%.2f, %.2f)", ref, lib_id, x, y)
        return sym

    def connect_nets(
        self,
        net_name: str,
        points: List[Tuple[float, float]],
    ) -> None:
        if len(points) < 2:
            logger.warning("Need at least 2 points to connect net '%s'", net_name)
            return

        label = LocalLabel()
        label.uuid = self._uuid()
        label.text = net_name
        label.position = Position(X=points[0][0], Y=points[0][1], angle=0.0)
        label.effects = {"font": {"size": (1.27, 1.27)}, "justify": ["left", "bottom"]}
        self.schematic.labels.append(label)

        for i in range(len(points) - 1):
            wire = Wire()
            wire.uuid = self._uuid()
            wire.coords = {
                "start": Position(X=points[i][0], Y=points[i][1], angle=0.0),
                "end": Position(X=points[i + 1][0], Y=points[i + 1][1], angle=0.0),
            }
            wire.stroke = Stroke(width=0.1524)
            self.schematic.wires.append(wire)

        logger.debug("Connected net '%s' with %d segments", net_name, len(points) - 1)

    def add_power_symbols(self, x: float = 12.7, y: float = 12.7) -> None:
        for net_name, lib_id in (("VCC", "power:VCC"), ("GND", "power:GND")):
            sym = self.add_component(
                ref="#PWR" + self._uuid()[:4].upper(),
                lib_id=lib_id,
                value=net_name,
                x=x,
                y=y,
            )
            sym.inBom = False
            sym.onBoard = False
            y += 10.16
            logger.debug("Added power symbol '%s'", net_name)

    def add_junction(self, x: float, y: float) -> None:
        junction = Junction()
        junction.uuid = self._uuid()
        junction.position = Position(X=x, Y=y, angle=0.0)
        self.schematic.junctions.append(junction)

    def add_no_connect(self, x: float, y: float) -> None:
        nc = NoConnect()
        nc.uuid = self._uuid()
        nc.position = Position(X=x, Y=y, angle=0.0)
        self.schematic.noconnects.append(nc)

    def generate_netlist(self) -> str:
        nets: Dict[str, List[Tuple[str, str]]] = {}
        for sym in self.schematic.symbols:
            ref = ""
            for prop in sym.properties:
                if prop.key == "Reference":
                    ref = prop.value
                    break
            for pin in getattr(sym, "pins", []):
                net = pin.get("net", "")
                if net:
                    nets.setdefault(net, []).append((ref, pin.get("name", "")))

        lines: List[str] = [
            "(export (version \"D\")",
            f'  (design (source "{self.project_name}.kicad_sch") (date "") (tool "netlist_gen"))',
            "  (components",
        ]
        for sym in self.schematic.symbols:
            ref = ""
            val = ""
            for prop in sym.properties:
                if prop.key == "Reference":
                    ref = prop.value
                elif prop.key == "Value":
                    val = prop.value
            lines.append(f'    (comp (ref "{ref}") (value "{val}") (footprint ""))')
        lines.append("  )")
        lines.append("  (nets")
        for idx, (net_name, connections) in enumerate(nets.items(), start=1):
            lines.append(f'    (net (code "{idx}") (name "{net_name}")')
            for ref, pin in connections:
                lines.append(f'      (node (ref "{ref}") (pin "{pin}"))')
            lines.append("    )")
        lines.append("  )")
        lines.append(")")
        return "\n".join(lines)

    def save(self, filepath: str) -> None:
        directory = os.path.dirname(filepath)
        if directory and not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)
        self.schematic.to_file(filepath)
        logger.info("Saved KiCad schematic to %s", filepath)
