"""Verilog module hierarchy parser using regex-based extraction."""

from __future__ import annotations

import logging
import os
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


@dataclass
class Port:
    name: str
    direction: str
    width: int = 1
    msb: Optional[int] = None
    lsb: Optional[int] = None

    def __repr__(self) -> str:
        if self.width > 1:
            return f"Port({self.name}, {self.direction}[{self.msb}:{self.lsb}])"
        return f"Port({self.name}, {self.direction})"


@dataclass
class ModuleInstance:
    module_type: str
    instance_name: str
    port_connections: Dict[str, str] = field(default_factory=dict)

    def __repr__(self) -> str:
        return f"Instance({self.instance_name} of {self.module_type})"


@dataclass
class Module:
    name: str
    ports: List[Port] = field(default_factory=list)
    instances: List[ModuleInstance] = field(default_factory=list)
    wires: Dict[str, int] = field(default_factory=dict)
    parameters: Dict[str, str] = field(default_factory=dict)

    def get_port(self, name: str) -> Optional[Port]:
        for port in self.ports:
            if port.name == name:
                return port
        return None

    def get_instance(self, name: str) -> Optional[ModuleInstance]:
        for inst in self.instances:
            if inst.instance_name == name:
                return inst
        return None


class VerilogParserError(Exception):
    pass


class VerilogParser:
    _COMMENT_RE = re.compile(r"//.*?$|/\*.*?\*/", re.MULTILINE | re.DOTALL)
    _MODULE_RE = re.compile(
        r"module\s+(\w+)\s*(?:#\s*\((.*?)\))?\s*\((.*?)\);(.*?)endmodule",
        re.DOTALL | re.MULTILINE,
    )
    _PORT_DIR_RE = re.compile(
        r"(input|output|inout)\s+(?:\[(\d+)\s*:\s*(\d+)\])?\s*(?:wire|reg|logic)?\s*([^;]+)"
    )
    _PORT_LIST_RE = re.compile(r"(input|output|inout)\s+([^,;]+)")
    _INSTANCE_RE = re.compile(
        r"(\w+)\s+(?:#\s*\((.*?)\)\s*)?(\w+)\s*\((.*?)\);",
        re.DOTALL,
    )
    _WIRE_RE = re.compile(
        r"wire\s+(?:\[(\d+)\s*:\s*(\d+)\])?\s*([^;]+);"
    )
    _REG_RE = re.compile(
        r"reg\s+(?:\[(\d+)\s*:\s*(\d+)\])?\s*([^;]+);"
    )
    _PARAM_RE = re.compile(r"parameter\s+(\w+)\s*=\s*([^,;)]+)")
    _NET_ASSIGN_RE = re.compile(r"\.\s*(\w+)\s*\(\s*([^)]+)\s*\)")

    def __init__(self) -> None:
        self.modules: Dict[str, Module] = {}

    def _remove_comments(self, text: str) -> str:
        return self._COMMENT_RE.sub("", text)

    def _parse_port_width(self, msb_str: Optional[str], lsb_str: Optional[str]) -> Tuple[int, Optional[int], Optional[int]]:
        if msb_str is None or lsb_str is None:
            return 1, None, None
        try:
            msb = int(msb_str.strip())
            lsb = int(lsb_str.strip())
            width = abs(msb - lsb) + 1
            return width, msb, lsb
        except ValueError:
            return 1, None, None

    def _parse_port_declarations(self, port_text: str) -> List[Port]:
        ports: List[Port] = []
        for match in self._PORT_DIR_RE.finditer(port_text):
            direction = match.group(1)
            msb_str, lsb_str = match.group(2), match.group(3)
            names_str = match.group(4)
            width, msb, lsb = self._parse_port_width(msb_str, lsb_str)
            for name in names_str.split(","):
                name = name.strip()
                if name:
                    ports.append(Port(name=name, direction=direction, width=width, msb=msb, lsb=lsb))
        return ports

    def _parse_inline_ports(self, header_text: str) -> List[Port]:
        ports: List[Port] = []
        for match in self._PORT_LIST_RE.finditer(header_text):
            direction = match.group(1)
            names_str = match.group(2)
            for name in names_str.split(","):
                name = name.strip()
                if name:
                    ports.append(Port(name=name, direction=direction))
        return ports

    def _parse_parameter_block(self, param_text: Optional[str]) -> Dict[str, str]:
        params: Dict[str, str] = {}
        if not param_text:
            return params
        for match in self._PARAM_RE.finditer(param_text):
            params[match.group(1).strip()] = match.group(2).strip()
        return params

    def _parse_wires(self, body_text: str) -> Dict[str, int]:
        wires: Dict[str, int] = {}
        for pattern in (self._WIRE_RE, self._REG_RE):
            for match in pattern.finditer(body_text):
                msb_str, lsb_str = match.group(1), match.group(2)
                names_str = match.group(3)
                width, _, _ = self._parse_port_width(msb_str, lsb_str)
                for name in names_str.split(","):
                    name = name.strip()
                    if name:
                        wires[name] = width
        return wires

    def _parse_instances(self, body_text: str) -> List[ModuleInstance]:
        instances: List[ModuleInstance] = []
        keywords = {"if", "else", "for", "while", "case", "always", "initial", "assign", "generate", "begin", "end"}

        for match in self._INSTANCE_RE.finditer(body_text):
            module_type = match.group(1)
            if module_type in keywords:
                continue

            instance_name = match.group(3)
            connection_text = match.group(4)

            connections: Dict[str, str] = {}
            if connection_text:
                for net_match in self._NET_ASSIGN_RE.finditer(connection_text):
                    port_name = net_match.group(1).strip()
                    net_name = net_match.group(2).strip()
                    connections[port_name] = net_name

            instances.append(
                ModuleInstance(
                    module_type=module_type,
                    instance_name=instance_name,
                    port_connections=connections,
                )
            )

        return instances

    def parse_file(self, filepath: str) -> Dict[str, Module]:
        if not os.path.isfile(filepath):
            raise VerilogParserError(f"File not found: {filepath}")

        logger.info("Parsing Verilog file: %s", filepath)
        with open(filepath, "r", encoding="utf-8") as fh:
            text = fh.read()

        text = self._remove_comments(text)
        found: Dict[str, Module] = {}

        for match in self._MODULE_RE.finditer(text):
            module_name = match.group(1)
            param_text = match.group(2)
            port_header = match.group(3)
            body = match.group(4)

            ports = self._parse_port_declarations(body)
            if not ports:
                ports = self._parse_inline_ports(port_header)

            module = Module(
                name=module_name,
                ports=ports,
                instances=self._parse_instances(body),
                wires=self._parse_wires(body),
                parameters=self._parse_parameter_block(param_text),
            )

            found[module_name] = module
            self.modules[module_name] = module
            logger.debug("Parsed module '%s' with %d ports, %d instances", module_name, len(ports), len(module.instances))

        if not found:
            logger.warning("No modules found in %s", filepath)

        return found

    def parse_directory(self, dirpath: str) -> Dict[str, Module]:
        if not os.path.isdir(dirpath):
            raise VerilogParserError(f"Directory not found: {dirpath}")

        logger.info("Scanning Verilog directory: %s", dirpath)
        for root, _dirs, files in os.walk(dirpath):
            for filename in files:
                if filename.endswith((".v", ".sv", ".vh")):
                    full_path = os.path.join(root, filename)
                    self.parse_file(full_path)

        logger.info("Total modules parsed: %d", len(self.modules))
        return dict(self.modules)

    def build_hierarchy(self, top_module: str, max_depth: int = 100) -> Dict[str, any]:
        if top_module not in self.modules:
            raise VerilogParserError(f"Top module '{top_module}' not found")

        visited: set = set()

        def _build(name: str, depth: int) -> Dict[str, any]:
            if depth > max_depth or name in visited:
                return {"module": name, "error": "cycle or max depth exceeded"}

            visited.add(name)
            module = self.modules.get(name)
            if module is None:
                return {"module": name, "error": "module not found"}

            children: List[Dict[str, any]] = []
            for inst in module.instances:
                child = _build(inst.module_type, depth + 1)
                child["instance_name"] = inst.instance_name
                child["connections"] = dict(inst.port_connections)
                children.append(child)

            visited.discard(name)
            return {
                "module": name,
                "ports": [p.__dict__ for p in module.ports],
                "parameters": dict(module.parameters),
                "children": children,
            }

        return _build(top_module, 0)

    def get_module_dependency_graph(self) -> Dict[str, List[str]]:
        graph: Dict[str, List[str]] = {}
        for name, module in self.modules.items():
            deps: List[str] = []
            for inst in module.instances:
                if inst.module_type in self.modules:
                    deps.append(inst.module_type)
            graph[name] = deps
        return graph

    def topological_sort(self) -> List[str]:
        graph = self.get_module_dependency_graph()
        in_degree: Dict[str, int] = {name: 0 for name in graph}
        for deps in graph.values():
            for dep in deps:
                in_degree[dep] = in_degree.get(dep, 0) + 1

        queue = [name for name, deg in in_degree.items() if deg == 0]
        result: List[str] = []

        while queue:
            node = queue.pop(0)
            result.append(node)
            for neighbor in graph.get(node, []):
                in_degree[neighbor] -= 1
                if in_degree[neighbor] == 0:
                    queue.append(neighbor)

        if len(result) != len(in_degree):
            raise VerilogParserError("Dependency cycle detected among modules")

        return list(reversed(result))
