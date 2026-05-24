import logging
import math
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from netlist_gen.verilog_parser import Module

logger = logging.getLogger(__name__)


@dataclass
class PlacementResult:
    component: str
    x: float
    y: float
    rotation: float = 0.0


class PlacementEngine:
    def __init__(self, board_width: float, board_height: float) -> None:
        self.board_width = board_width
        self.board_height = board_height
        self.positions: Dict[str, Tuple[float, float]] = {}
        self._temperature = 100.0
        self._cooling_rate = 0.95

    def _is_inside_board(self, x: float, y: float, margin: float = 2.54) -> bool:
        return margin <= x <= self.board_width - margin and margin <= y <= self.board_height - margin

    def _random_position(self) -> Tuple[float, float]:
        margin = 5.0
        x = random.uniform(margin, self.board_width - margin)
        y = random.uniform(margin, self.board_height - margin)
        return round(x, 2), round(y, 2)

    def _compute_wire_length(self, nets: List[Dict[str, any]]) -> float:
        total = 0.0
        for net in nets:
            pins = net.get("pins", [])
            coords = []
            for pin in pins:
                comp = pin.get("component")
                if comp and comp in self.positions:
                    ox, oy = self.positions[comp]
                    coords.append((ox + pin.get("offset_x", 0.0), oy + pin.get("offset_y", 0.0)))
            if len(coords) < 2:
                continue
            cx = sum(c[0] for c in coords) / len(coords)
            cy = sum(c[1] for c in coords) / len(coords)
            for x, y in coords:
                total += math.hypot(x - cx, y - cy)
        return total

    def _compute_overlap_penalty(self, component_sizes: Dict[str, Tuple[float, float]]) -> float:
        penalty = 0.0
        comps = list(self.positions.keys())
        for i in range(len(comps)):
            for j in range(i + 1, len(comps)):
                x1, y1 = self.positions[comps[i]]
                x2, y2 = self.positions[comps[j]]
                w1, h1 = component_sizes.get(comps[i], (5.0, 5.0))
                w2, h2 = component_sizes.get(comps[j], (5.0, 5.0))
                dx = abs(x1 - x2) - (w1 + w2) / 2.0
                dy = abs(y1 - y2) - (h1 + h2) / 2.0
                if dx < 0 and dy < 0:
                    penalty += abs(dx * dy)
        return penalty * 1000.0

    def place(
        self,
        components: List[str],
        nets: List[Dict[str, any]],
        component_sizes: Optional[Dict[str, Tuple[float, float]]] = None,
    ) -> List[PlacementResult]:
        sizes = component_sizes or {}
        self.positions.clear()

        for comp in components:
            x, y = self._random_position()
            self.positions[comp] = (x, y)

        best_positions = dict(self.positions)
        best_cost = self._compute_wire_length(nets) + self._compute_overlap_penalty(sizes)

        temp = self._temperature
        while temp > 0.1:
            comp = random.choice(components)
            old_x, old_y = self.positions[comp]
            new_x, new_y = self._random_position()
            self.positions[comp] = (new_x, new_y)

            if not self._is_inside_board(new_x, new_y):
                self.positions[comp] = (old_x, old_y)
                temp *= self._cooling_rate
                continue

            cost = self._compute_wire_length(nets) + self._compute_overlap_penalty(sizes)
            delta = cost - best_cost

            if delta < 0 or random.random() < math.exp(-delta / temp):
                best_cost = cost
                best_positions = dict(self.positions)
            else:
                self.positions[comp] = (old_x, old_y)

            temp *= self._cooling_rate

        self.positions = best_positions
        results: List[PlacementResult] = []
        for comp, (x, y) in self.positions.items():
            results.append(PlacementResult(component=comp, x=x, y=y))

        logger.info("Placement completed with %.2f mm estimated wire length", best_cost)
        return results

    def optimize(self, iterations: int = 100) -> None:
        for i in range(iterations):
            if i % 20 == 0:
                logger.debug("Placement optimization iteration %d/%d", i, iterations)

    def export_positions(self) -> Dict[str, Tuple[float, float]]:
        return dict(self.positions)

    def apply_hierarchical_placement(
        self,
        hierarchy: Dict[str, any],
        level_spacing: float = 50.0,
        sibling_spacing: float = 30.0,
    ) -> List[PlacementResult]:
        results: List[PlacementResult] = []

        def _place(node: Dict[str, any], depth: int, index: int) -> None:
            mod_name = node.get("module", "unknown")
            inst_name = node.get("instance_name", mod_name)
            x = index * sibling_spacing + 10.0
            y = depth * level_spacing + 10.0
            results.append(PlacementResult(component=inst_name, x=x, y=y))
            for idx, child in enumerate(node.get("children", [])):
                _place(child, depth + 1, index + idx)

        _place(hierarchy, 0, 0)
        for r in results:
            self.positions[r.component] = (r.x, r.y)
        return results
