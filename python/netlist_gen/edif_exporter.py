"""EDIF 2 0 0 netlist exporter using SpyDrNet."""

from __future__ import annotations

import logging
from typing import Dict, List, Optional, Tuple

import spydrnet as sdn

from netlist_gen.verilog_parser import Module, Port

logger = logging.getLogger(__name__)


class EDIFExporterError(Exception):
    pass


class EDIFExporter:
    def __init__(self, design_name: str) -> None:
        self.design_name = design_name
        self.netlist: sdn.Netlist = sdn.Netlist()
        self.netlist.name = design_name
        self._libraries: Dict[str, sdn.Library] = {}
        self._cells: Dict[Tuple[str, str], sdn.Cell] = {}
        self._ports: Dict[Tuple[str, str, str], sdn.Port] = {}
        self._nets: Dict[str, sdn.Net] = {}

    def add_library(self, name: str) -> sdn.Library:
        if name in self._libraries:
            return self._libraries[name]
        library = self.netlist.create_library()
        library.name = name
        self._libraries[name] = library
        logger.debug("Created EDIF library '%s'", name)
        return library

    def add_cell(self, library_name: str, cell_name: str) -> sdn.Cell:
        key = (library_name, cell_name)
        if key in self._cells:
            return self._cells[key]

        library = self.add_library(library_name)
        cell = library.create_cell()
        cell.name = cell_name
        self._cells[key] = cell
        logger.debug("Created cell '%s' in library '%s'", cell_name, library_name)
        return cell

    def add_port(
        self,
        library_name: str,
        cell_name: str,
        port_name: str,
        direction: str,
        width: int = 1,
    ) -> sdn.Port:
        key = (library_name, cell_name, port_name)
        if key in self._ports:
            return self._ports[key]

        cell = self.add_cell(library_name, cell_name)
        port = cell.create_port()
        port.name = port_name

        if direction == "input":
            port.direction = sdn.IN
        elif direction == "output":
            port.direction = sdn.OUT
        elif direction == "inout":
            port.direction = sdn.INOUT
        else:
            port.direction = sdn.UNDEFINED

        if width > 1:
            port.is_scalar = False
            port.create_pins(width)
        else:
            port.is_scalar = True
            port.create_pins(1)

        self._ports[key] = port
        logger.debug(
            "Added port '%s' to cell '%s' (%s, width=%d)",
            port_name,
            cell_name,
            direction,
            width,
        )
        return port

    def add_instance(
        self,
        parent_lib: str,
        parent_cell: str,
        instance_name: str,
        ref_lib: str,
        ref_cell: str,
    ) -> sdn.Instance:
        parent = self.add_cell(parent_lib, parent_cell)
        ref = self.add_cell(ref_lib, ref_cell)
        inst = parent.create_child()
        inst.name = instance_name
        inst.reference = ref
        logger.debug(
            "Added instance '%s' of '%s' to cell '%s'",
            instance_name,
            ref_cell,
            parent_cell,
        )
        return inst

    def add_net(
        self,
        library_name: str,
        cell_name: str,
        net_name: str,
        connections: List[Tuple[str, str, Optional[int]]],
    ) -> sdn.Net:
        key = (library_name, cell_name, net_name)
        if key in self._nets:
            return self._nets[key]

        cell = self.add_cell(library_name, cell_name)
        net = cell.create_net()
        net.name = net_name

        for inst_name, port_name, pin_idx in connections:
            if inst_name == "__outer__":
                port = self._ports.get((library_name, cell_name, port_name))
                if port is None:
                    logger.warning(
                        "Port '%s' not found for net '%s' in cell '%s'",
                        port_name,
                        net_name,
                        cell_name,
                    )
                    continue
                pin = port.pins[pin_idx if pin_idx is not None else 0]
                net.connect_pin(pin)
            else:
                inst = next(
                    (
                        c
                        for c in cell.children
                        if c.name == inst_name
                    ),
                    None,
                )
                if inst is None:
                    logger.warning(
                        "Instance '%s' not found for net '%s' in cell '%s'",
                        inst_name,
                        net_name,
                        cell_name,
                    )
                    continue
                port = inst.reference.get_ports(port_name)
                if not port:
                    logger.warning(
                        "Port '%s' not found on instance '%s'",
                        port_name,
                        inst_name,
                    )
                    continue
                pin = port[0].pins[pin_idx if pin_idx is not None else 0]
                net.connect_pin(pin)

        self._nets[key] = net
        logger.debug("Added net '%s' to cell '%s'", net_name, cell_name)
        return net

    def import_module(self, module: Module, library_name: str = "work") -> None:
        cell = self.add_cell(library_name, module.name)

        for port in module.ports:
            self.add_port(
                library_name,
                module.name,
                port.name,
                port.direction,
                port.width,
            )

        for inst in module.instances:
            self.add_instance(
                library_name,
                module.name,
                inst.instance_name,
                library_name,
                inst.module_type,
            )

        net_map: Dict[str, List[Tuple[str, str, Optional[int]]]] = {}
        for port in module.ports:
            net_map.setdefault(port.name, []).append(
                ("__outer__", port.name, 0)
            )

        for inst in module.instances:
            for port_name, net_name in inst.port_connections.items():
                net_map.setdefault(net_name, []).append(
                    (inst.instance_name, port_name, 0)
                )

        for net_name, conns in net_map.items():
            self.add_net(library_name, module.name, net_name, conns)

    def export(self, filepath: str) -> None:
        sdn.compose(self.netlist, filepath)
        logger.info("Exported EDIF netlist to %s", filepath)
