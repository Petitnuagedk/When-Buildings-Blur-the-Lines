#!/usr/bin/env python3
"""
Non-Uniform NS-3 Building Layout Generator
============================================
Generates a building layout CSV compatible with ns-3 BuildingHelper,
given an area size and a density profile.

The area is centred on the origin (matching the conference paper convention).
Buildings are placed on a regular grid but with spatially varying occupation
probability, producing a dense core and sparse periphery — or any other
density profile defined by the profile functions below.

Output columns: xmin, xmax, ymin, ymax, zmin, zmax
  (zmin is always 0; zmax is building height)

Usage:
    python building_gen.py                        # default: 1500x1500, core-heavy
    python building_gen.py --size 2000            # 2000x2000 area
    python building_gen.py --profile uniform      # uniform density (replicates conf paper)
    python building_gen.py --profile core         # dense core, sparse periphery (default)
    python building_gen.py --profile corridor     # dense corridors (street-canyon effect)
    python building_gen.py --profile random       # fully random placement
    python building_gen.py --cell 100 --bsize 80  # 100m grid, 80m wide buildings
    python building_gen.py --seed 42 --out layout.csv
    python building_gen.py --plot                 # visualise before saving
    python building_gen.py --nodes 50 --points-out nodes.csv  # generate 50 positions
    python building_gen.py --nodes 50 --clearance 10          # with buffer around buildings
"""

import argparse
import csv
import math
import random
import sys

# ------------------------------------------------------------------
# DENSITY PROFILE FUNCTIONS
# Each function takes (x, y, half_size) where x,y are the cell
# centre coordinates and half_size = area_size / 2.
# Returns a probability in [0, 1] that a building occupies the cell.
# ------------------------------------------------------------------

def profile_uniform(x, y, half):
    """Uniform 60% coverage — replicates the conference paper layout."""
    return 0.60


def profile_core(x, y, half):
    """
    Dense core (~85% coverage), sparse periphery (~15%).
    Gaussian radial falloff centred on origin.
    Models a downtown-core + suburban-periphery urban morphology.
    """
    r = math.sqrt(x**2 + y**2) / half          # normalised radius in [0, ~1.4]
    sigma = 0.45                                 # controls core radius
    p_max, p_min = 0.85, 0.15
    gauss = math.exp(-(r**2) / (2 * sigma**2))
    return p_min + (p_max - p_min) * gauss


def profile_corridor(x, y, half):
    """
    High density along X and Y axes, low density elsewhere.
    Produces street-canyon corridors crossing at the origin.
    Models a grid-street city with wide main avenues.
    """
    norm_x = abs(x) / half
    norm_y = abs(y) / half
    # Probability is high near axes, low away from them
    near_x = math.exp(-(norm_y**2) / (2 * 0.15**2))
    near_y = math.exp(-(norm_x**2) / (2 * 0.15**2))
    corridor_prob = max(near_x, near_y)
    # Invert: high density WHERE THERE ARE NO corridors
    p = 0.80 * (1.0 - 0.70 * corridor_prob) + 0.10
    return max(0.05, min(0.90, p))


def profile_random(x, y, half):
    """Fully random placement, 50% expected coverage."""
    return 0.50


PROFILES = {
    "uniform":  profile_uniform,
    "core":     profile_core,
    "corridor": profile_corridor,
    "random":   profile_random,
}

# ------------------------------------------------------------------
# GENERATOR
# ------------------------------------------------------------------

def generate_layout(
    area_size:    int   = 1500,
    cell_size:    int   = 100,
    building_size: int  = 100,
    building_height: float = 25.0,
    profile_name: str  = "core",
    min_spacing:  float = 10.0,
    seed:         int   = None,
) -> list[dict]:
    """
    Generate a list of building records.

    Parameters
    ----------
    area_size       : total side length of the square simulation area (m)
    cell_size       : grid cell size (m); buildings are centred in each cell
    building_size   : building footprint side length (m); must be <= cell_size - min_spacing
    building_height : building height (m)
    profile_name    : density profile key (see PROFILES dict)
    min_spacing     : minimum gap between adjacent buildings (m)
    seed            : random seed for reproducibility (None = random)

    Returns
    -------
    List of dicts with keys: xmin, xmax, ymin, ymax, zmin, zmax
    """
    # ensure spacing constraint
    if building_size > cell_size:
        raise ValueError("building_size must be <= cell_size")
    if building_size > cell_size - min_spacing:
        raise ValueError(
            f"building_size too large for required spacing ({min_spacing}m); "
            f"increase cell_size or reduce building_size"
        )

    if profile_name not in PROFILES:
        raise ValueError(f"Unknown profile '{profile_name}'. "
                         f"Choose from: {list(PROFILES.keys())}")

    rng      = random.Random(seed)
    profile  = PROFILES[profile_name]
    half     = area_size / 2.0
    offset   = building_size / 2.0

    # Grid: cell centres from -half + cell_size/2 to +half - cell_size/2
    steps    = int(area_size / cell_size)
    starts   = [-half + cell_size * (i + 0.5) for i in range(steps)]

    buildings = []
    for cx in starts:
        for cy in starts:
            p = profile(cx, cy, half)
            if rng.random() < p:
                buildings.append({
                    "xmin": cx - offset,
                    "xmax": cx + offset,
                    "ymin": cy - offset,
                    "ymax": cy + offset,
                    "zmin": 0.0,
                    "zmax": building_height,
                })

    return buildings


# ------------------------------------------------------------------
# NODE POSITION GENERATOR
# ------------------------------------------------------------------

def point_in_building(px: float, py: float, b: dict, clearance: float = 0.0) -> bool:
    """Return True if point lies within building footprint expanded by clearance."""
    if (px >= b["xmin"] - clearance and px <= b["xmax"] + clearance and
        py >= b["ymin"] - clearance and py <= b["ymax"] + clearance):
        return True
    return False


def generate_positions(
    count: int,
    area_size: float,
    buildings: list[dict],
    clearance: float = 0.0,
    seed: int = None,
) -> list[tuple[float, float]]:
    """Generate random positions within the area and outside buildings.

    Parameters
    ----------
    count      : number of positions to generate
    area_size  : side length of square area (m), centred on origin
    buildings  : list of building dicts (see :func:`generate_layout`)
    clearance  : additional buffer distance around buildings (m)
    seed       : random seed for reproducibility

    Returns
    -------
    List of (x, y) tuples representing valid positions.
    """
    rng = random.Random(seed)
    half = area_size / 2.0
    pts: list[tuple[float, float]] = []
    attempts = 0
    max_attempts = count * 1000  # prevent infinite loops
    while len(pts) < count and attempts < max_attempts:
        attempts += 1
        x = rng.uniform(-half, half)
        y = rng.uniform(-half, half)
        # reject if inside any building or within clearance
        if any(point_in_building(x, y, b, clearance) for b in buildings):
            continue
        pts.append((x, y))
    if len(pts) < count:
        raise RuntimeError(f"Only generated {len(pts)} of {count} positions; "
                           "consider reducing clearance or increasing area size")
    return pts


def write_positions(positions: list[tuple[float, float]], path: str):
    """Write positions to CSV with columns x,y."""
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y"])
        w.writerows(positions)
    print(f"Written {len(positions)} positions to {path}")


def write_csv(buildings: list[dict], path: str):
    fieldnames = ["xmin", "xmax", "ymin", "ymax", "zmin", "zmax"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(buildings)
    print(f"Written {len(buildings)} buildings to {path}")


def plot_layout(buildings: list[dict], area_size: int, profile_name: str, positions: list[tuple[float,float]] | None = None):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
    except ImportError:
        print("matplotlib not installed — skipping plot (pip install matplotlib)")
        return

    half = area_size / 2
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(-half, half)
    ax.set_ylim(-half, half)
    ax.set_aspect("equal")
    ax.set_facecolor("#f5f5f0")
    ax.set_title(f"Building layout — profile: {profile_name} "
                 f"({len(buildings)} buildings)", fontsize=11)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    # draw grid and central axes for orientation
    ax.grid(True, linestyle='--', linewidth=0.5, color="#cccccc")
    ax.axhline(0, color="#444444", linewidth=1)
    ax.axvline(0, color="#444444", linewidth=1)

    for b in buildings:
        w = b["xmax"] - b["xmin"]
        h = b["ymax"] - b["ymin"]
        rect = patches.Rectangle(
            (b["xmin"], b["ymin"]), w, h,
            linewidth=0.5, edgecolor="#334466",
            facecolor="#6688aa", alpha=0.75
        )
        ax.add_patch(rect)

    # draw node positions if provided
    if positions:
        xs, ys = zip(*positions)
        ax.scatter(xs, ys, c="red", s=20, marker="o", label="nodes")
        ax.legend(loc="upper right", fontsize=8)

    # Coverage stats
    cell_area     = (b["xmax"] - b["xmin"]) * (b["ymax"] - b["ymin"])
    total_bld     = len(buildings) * cell_area
    total_area    = area_size ** 2
    coverage_pct  = 100 * total_bld / total_area
    ax.text(0.02, 0.98,
            f"Buildings: {len(buildings)}\n"
            f"Coverage: {coverage_pct:.1f}%",
            transform=ax.transAxes,
            verticalalignment="top",
            fontsize=9,
            bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.8))

    plt.tight_layout()
    plt.show()


# ------------------------------------------------------------------
# NS-3 C++ SNIPPET GENERATOR (bonus)
# Prints a ready-to-paste ns-3 C++ block for the generated layout.
# ------------------------------------------------------------------

def print_ns3_snippet(buildings: list[dict], n_floors: int = 1,
                      n_rooms_x: int = 1, n_rooms_y: int = 1):
    print("\n// --- NS-3 C++ building placement snippet ---")
    print("// Paste inside your simulation function after including:")
    print("// #include \"ns3/building.h\"")
    print("// #include \"ns3/building-list.h\"\n")
    print(f"// {len(buildings)} buildings")
    print("Ptr<Building> b;")
    for i, b in enumerate(buildings):
        print(f"b = CreateObject<Building>();")
        print(f"b->SetBoundaries(Box({b['xmin']}, {b['xmax']}, "
              f"{b['ymin']}, {b['ymax']}, "
              f"{b['zmin']}, {b['zmax']}));")
        print(f"b->SetBuildingType(Building::Residential);")
        print(f"b->SetExtWallsType(Building::ConcreteWithWindows);")
        print(f"b->SetNFloors({n_floors});")
        print(f"b->SetNRoomsX({n_rooms_x});")
        print(f"b->SetNRoomsY({n_rooms_y});")
        if i < len(buildings) - 1:
            print()
    print("// --- end building placement ---\n")


# ------------------------------------------------------------------
# CLI
# ------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Generate a non-uniform ns-3 building layout."
    )
    parser.add_argument("--size",    type=int,   default=1500,
                        help="Area side length in metres (default: 1500)")
    parser.add_argument("--cell",    type=int,   default=100,
                        help="Grid cell size in metres (default: 100)")
    parser.add_argument("--bsize",   type=int,   default=100,
                        help="Building footprint side length (default: 100)")
    parser.add_argument("--height",  type=float, default=25.0,
                        help="Building height in metres (default: 25.0)")
    parser.add_argument("--profile", choices=list(PROFILES.keys()),
                        default="core",
                        help="Density profile (default: core)")
    parser.add_argument("--seed",    type=int,   default=None,
                        help="Random seed for reproducibility")
    parser.add_argument("--out",     default="building_layout.csv",
                        help="Output CSV path (default: building_layout.csv)")
    parser.add_argument("--spacing", type=float, default=10.0,
                        help="Minimum distance between buildings (m)")
    parser.add_argument("--nodes",   type=int,   default=0,
                        help="Number of random positions to generate inside area")
    parser.add_argument("--clearance", type=float, default=0.0,
                        help="Minimum distance from buildings for generated positions (m)")
    parser.add_argument("--points-out", default=None,
                        help="CSV path to write generated positions (x,y)")
    parser.add_argument("--plot",    action="store_true",
                        help="Visualise the layout with matplotlib (x-y axes display)")
    parser.add_argument("--ns3",     action="store_true",
                        help="Print a ready-to-paste ns-3 C++ snippet")
    args = parser.parse_args()

    print(f"Generating layout: {args.size}x{args.size}m, "
          f"cell={args.cell}m, building={args.bsize}m, "
          f"height={args.height}m, profile={args.profile}, "
          f"min_spacing={args.spacing}m, seed={args.seed}")

    buildings = generate_layout(
        area_size       = args.size,
        cell_size       = args.cell,
        building_size   = args.bsize,
        building_height = args.height,
        profile_name    = args.profile,
        min_spacing     = args.spacing,
        seed            = args.seed,
    )

    total_cells   = (args.size // args.cell) ** 2
    coverage      = 100 * len(buildings) / total_cells
    print(f"Placed {len(buildings)} / {total_cells} cells "
          f"({coverage:.1f}% coverage)")

    write_csv(buildings, args.out)

    # optionally generate node positions
    if args.nodes > 0:
        pts = generate_positions(
            count = args.nodes,
            area_size = args.size,
            buildings = buildings,
            clearance = args.clearance,
            seed = args.seed,
        )
        if args.points_out:
            write_positions(pts, args.points_out)
        else:
            print(f"Generated {len(pts)} positions (no output file specified)")

    if args.plot:
        plot_layout(buildings, args.size, args.profile,
                    positions = pts if args.nodes > 0 else None)

    if args.ns3:
        print_ns3_snippet(buildings)


if __name__ == "__main__":
    main()