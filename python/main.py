import argparse
import json
import logging
import os
import sys
from typing import Tuple

from netlist_gen.verilog_parser import VerilogParser, VerilogParserError
from netlist_gen.kicad_generator import KiCadGenerator, KiCadGeneratorError
from netlist_gen.edif_exporter import EDIFExporter, EDIFExporterError
from netlist_gen.placement import PlacementEngine


def setup_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def build_kicad(
    parser: VerilogParser,
    top_module: str,
    output_path: str,
    board_size: Tuple[float, float],
) -> None:
    gen = KiCadGenerator(top_module)
    hierarchy = parser.build_hierarchy(top_module)

    modules = list(parser.modules.values())
    x_offset = 0.0
    y_offset = 0.0
    for module in modules:
        gen.add_module_as_sheet(module, x_offset, y_offset)
        x_offset += 60.0
        if x_offset > 200.0:
            x_offset = 0.0
            y_offset += 50.0

    gen.add_power_symbols()
    gen.save(output_path)


def build_edif(
    parser: VerilogParser,
    top_module: str,
    output_path: str,
) -> None:
    exporter = EDIFExporter(top_module)
    if top_module not in parser.modules:
        raise VerilogParserError(f"Top module '{top_module}' not found")

    ordered = parser.topological_sort()
    for mod_name in ordered:
        exporter.import_module(parser.modules[mod_name])

    exporter.export(output_path)


def run_placement(
    parser: VerilogParser,
    top_module: str,
    board_width: float,
    board_height: float,
) -> None:
    engine = PlacementEngine(board_width, board_height)
    hierarchy = parser.build_hierarchy(top_module)
    results = engine.apply_hierarchical_placement(hierarchy)

    data = {
        "board_size": {"width": board_width, "height": board_height},
        "placements": [
            {
                "component": r.component,
                "x": r.x,
                "y": r.y,
                "rotation": r.rotation,
            }
            for r in results
        ],
    }
    print(json.dumps(data, indent=2))


def main(argv: list = None) -> int:
    if argv is None:
        argv = sys.argv[1:]

    argparser = argparse.ArgumentParser(
        description="Generate KiCad schematics and EDIF netlists from Verilog RTL",
    )
    argparser.add_argument(
        "--verilog-dir",
        required=True,
        help="Directory containing Verilog source files",
    )
    argparser.add_argument(
        "--top-module",
        default="top",
        help="Name of the top-level module (default: top)",
    )
    argparser.add_argument(
        "--output",
        required=True,
        help="Output file path",
    )
    argparser.add_argument(
        "--format",
        choices=["kicad", "edif", "both"],
        default="both",
        help="Output format (default: both)",
    )
    argparser.add_argument(
        "--place",
        action="store_true",
        help="Run placement optimization and output JSON",
    )
    argparser.add_argument(
        "--board-size",
        default="100x100mm",
        help="Board size in WIDTHxHEIGHTmm format (default: 100x100mm)",
    )
    argparser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose debug logging",
    )

    args = argparser.parse_args(argv)
    setup_logging(args.verbose)

    if not os.path.isdir(args.verilog_dir):
        logging.error("Verilog directory not found: %s", args.verilog_dir)
        return 1

    parser = VerilogParser()
    try:
        parser.parse_directory(args.verilog_dir)
    except VerilogParserError as exc:
        logging.error("Failed to parse Verilog: %s", exc)
        return 1

    if args.top_module not in parser.modules:
        logging.error("Top module '%s' not found in parsed modules", args.top_module)
        return 1

    board_size = args.board_size.lower().replace("mm", "").split("x")
    try:
        board_width = float(board_size[0])
        board_height = float(board_size[1])
    except (IndexError, ValueError):
        logging.error("Invalid board size format: %s", args.board_size)
        return 1

    try:
        if args.format in ("kicad", "both"):
            kicad_path = args.output
            if not kicad_path.endswith(".kicad_sch"):
                kicad_path += ".kicad_sch"
            build_kicad(parser, args.top_module, kicad_path, (board_width, board_height))

        if args.format in ("edif", "both"):
            edif_path = args.output
            if not edif_path.endswith(".edif"):
                edif_path += ".edif"
            build_edif(parser, args.top_module, edif_path)

        if args.place:
            run_placement(parser, args.top_module, board_width, board_height)
    except (VerilogParserError, KiCadGeneratorError, EDIFExporterError) as exc:
        logging.error("Generation failed: %s", exc)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
