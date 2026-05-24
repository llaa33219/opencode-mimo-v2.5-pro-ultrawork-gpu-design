import math
import unittest

from netlist_gen.placement import PlacementEngine, PlacementResult


class TestPlacementResult(unittest.TestCase):
    def test_defaults(self):
        r = PlacementResult(component="U1", x=10.0, y=20.0)
        self.assertEqual(r.rotation, 0.0)


class TestPlacementEngine(unittest.TestCase):
    def test_init(self):
        engine = PlacementEngine(100.0, 80.0)
        self.assertEqual(engine.board_width, 100.0)
        self.assertEqual(engine.board_height, 80.0)

    def test_place_basic(self):
        engine = PlacementEngine(100.0, 100.0)
        components = ["U1", "U2", "U3"]
        nets = [
            {
                "pins": [
                    {"component": "U1", "offset_x": 0.0, "offset_y": 0.0},
                    {"component": "U2", "offset_x": 0.0, "offset_y": 0.0},
                ]
            }
        ]
        results = engine.place(components, nets)
        self.assertEqual(len(results), 3)
        for r in results:
            self.assertTrue(0.0 <= r.x <= 100.0)
            self.assertTrue(0.0 <= r.y <= 100.0)

    def test_place_with_overlap_penalty(self):
        engine = PlacementEngine(50.0, 50.0)
        components = ["U1", "U2"]
        nets = []
        sizes = {"U1": (20.0, 20.0), "U2": (20.0, 20.0)}
        results = engine.place(components, nets, sizes)
        self.assertEqual(len(results), 2)

    def test_hierarchical_placement(self):
        engine = PlacementEngine(200.0, 200.0)
        hierarchy = {
            "module": "top",
            "children": [
                {"module": "child1", "children": []},
                {"module": "child2", "children": []},
            ],
        }
        results = engine.apply_hierarchical_placement(hierarchy)
        self.assertEqual(len(results), 3)
        names = [r.component for r in results]
        self.assertIn("top", names)
        self.assertIn("child1", names)

    def test_export_positions(self):
        engine = PlacementEngine(100.0, 100.0)
        components = ["U1", "U2"]
        nets = []
        engine.place(components, nets)
        positions = engine.export_positions()
        self.assertIn("U1", positions)
        self.assertIn("U2", positions)
        self.assertEqual(len(positions), 2)


if __name__ == "__main__":
    unittest.main()
