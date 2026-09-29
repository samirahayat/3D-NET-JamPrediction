"""
grid_builder.py — Procedural road-network generator.

This module contains a single public function, ``build_road_network()``,
which is called by :class:`TrafficModel.__init__` to populate the grid
with road cells, junction blocks, traffic-light controllers, approach
lights, and edge entry points.

The network layout is a simple **grid of streets**: two horizontal
corridors and two vertical corridors whose intersections form four
junctions.  Lane counts are randomised (1 or 2 per direction).

Layout overview (example with 2-lane roads)::

      W ← ← ← ← ← ← ← ← ← ← ←
      E → → → → → → → → → → → →
              ↑ ↓       ↑ ↓
              N S       N S
              ↑ ↓       ↑ ↓
      W ← ← ← ← ← ← ← ← ← ← ←
      E → → → → → → → → → → → →
              ↑ ↓       ↑ ↓
              N S       N S
"""

import random

from cells import RoadCell, JunctionCell, ApproachLight
from controllers import JunctionController


def build_road_network(model, lanes_choice_prob, green_duration,
                       yellow_duration, yellow_max_wait,
                       num_horizontal_roads=2, num_vertical_roads=2):
    """
    Populate ``model.grid`` with the full road network and return
    configuration data used by the model.

    Parameters
    ----------
    model : TrafficModel
        The model whose grid and schedule we populate.
    lanes_choice_prob : float
        Probability that any corridor has 2 lanes per direction
        (otherwise 1 lane).
    green_duration, yellow_duration, yellow_max_wait : int
        Timing parameters forwarded to :class:`JunctionController`.
    num_horizontal_roads : int
        Number of horizontal (East/West) corridors.
    num_vertical_roads : int
        Number of vertical (North/South) corridors.

    Returns
    -------
    dict with keys:
        ``roads``               – set of (x, y) road positions
        ``junctions``           – set of (x, y) junction positions
        ``junction_controllers``– dict  jid → JunctionController
        ``entry_cells``         – list of ((x, y), direction) spawn points
    """
    width, height = model.width, model.height

    # ── 1. Choose corridor centres and lane counts ───────────────
    # Spread corridors evenly across the grid, leaving a margin at the edges.
    # The step size adapts so corridors don't overlap or crowd together.
    h_count = max(1, int(num_horizontal_roads))
    v_count = max(1, int(num_vertical_roads))

    h_candidates = list(range(6, height - 6, max(4, (height - 12) // (h_count + 1))))
    v_candidates = list(range(6, width - 6, max(4, (width - 12) // (v_count + 1))))

    # Clamp to available candidates (can't have more roads than fit)
    h_count = min(h_count, len(h_candidates))
    v_count = min(v_count, len(v_candidates))

    h_centers = sorted(random.sample(h_candidates, h_count))
    v_centers = sorted(random.sample(v_candidates, v_count))

    h_lanes = {y: (2 if random.random() < lanes_choice_prob else 1) for y in h_centers}
    v_lanes = {x: (2 if random.random() < lanes_choice_prob else 1) for x in v_centers}

    # ── 2. Mark road positions ───────────────────────────────────
    roads = set()
    roads = _mark_road_positions(roads, h_centers, h_lanes, v_centers, v_lanes,
                                 width, height)

    # ── 3. Build junction blocks ─────────────────────────────────
    junctions, junction_controllers = _build_junctions(
        model, roads, h_centers, h_lanes, v_centers, v_lanes,
        width, height, green_duration, yellow_duration, yellow_max_wait,
    )

    # ── 4. Place RoadCell agents ─────────────────────────────────
    _place_road_cells(model, roads, h_centers, h_lanes, v_centers, v_lanes, width, height)

    # ── 5. Place JunctionCell agents ─────────────────────────────
    road_directions = _compute_original_road_directions(
        h_centers, h_lanes, v_centers, v_lanes, width, height,
    )
    _place_junction_cells(model, junction_controllers, road_directions)

    # ── 6. Identify grid-edge entry cells ────────────────────────
    entry_cells = _find_entry_cells(model, width, height)

    # ── 7. Identify grid-edge exit cells ─────────────────────────
    exit_info = _find_exit_cells(model, width, height)

    return {
        "roads": roads,
        "junctions": junctions,
        "junction_controllers": junction_controllers,
        "entry_cells": entry_cells,
        "exit_cells": exit_info["exit_cells"],
        "exit_groups": exit_info["exit_groups"],
        "h_centers": h_centers,
        "v_centers": v_centers,
    }


# ═════════════════════════════════════════════════════════════════════
#  Private helpers
# ═════════════════════════════════════════════════════════════════════

def _mark_road_positions(roads, h_centers, h_lanes, v_centers, v_lanes,
                         width, height):
    """
    Return the set of all (x, y) positions that are road cells.

    For each horizontal centre ``y`` with ``L`` lanes:
      • rows ``y`` … ``y+L-1`` are East-bound
      • rows ``y-1`` … ``y-L`` are West-bound

    Vertical corridors follow the same logic with columns.
    """
    # Horizontal corridors
    for y in h_centers:
        lanes = h_lanes[y]
        for lane in range(lanes):
            row = y + lane                       # East-bound lane
            for x in range(width):
                if 0 <= row < height:
                    roads.add((x, row))
        for lane in range(lanes):
            row = y - lane - 1                   # West-bound lane
            for x in range(width):
                if 0 <= row < height:
                    roads.add((x, row))

    # Vertical corridors
    for x in v_centers:
        lanes = v_lanes[x]
        for lane in range(lanes):
            col = x + lane                       # North-bound lane
            for y in range(height):
                if 0 <= col < width:
                    roads.add((col, y))
        for lane in range(lanes):
            col = x - lane - 1                   # South-bound lane
            for y in range(height):
                if 0 <= col < width:
                    roads.add((col, y))

    return roads


def _build_junctions(model, roads, h_centers, h_lanes, v_centers, v_lanes,
                     width, height, green_duration, yellow_duration,
                     yellow_max_wait):
    """
    Create junction blocks where horizontal and vertical corridors
    intersect.  Junction width/height equals the total lane width of
    the crossing road (``lanes_per_dir × 2``).

    Also places :class:`ApproachLight` agents around each junction and
    removes junction positions from the ``roads`` set.
    """
    junctions = set()
    junction_controllers = {}
    used = set()  # prevent overlapping junctions

    for vx in v_centers:
        for hy in h_centers:
            v_w = v_lanes[vx] * 2   # total columns the vertical road occupies
            h_w = h_lanes[hy] * 2   # total rows the horizontal road occupies
            top_left_x = vx - v_w // 2
            top_left_y = hy - h_w // 2

            # Collect block positions
            block = [
                (top_left_x + dx, top_left_y + dy)
                for dx in range(v_w)
                for dy in range(h_w)
                if 0 <= top_left_x + dx < width and 0 <= top_left_y + dy < height
            ]
            block_set = set(block)

            # Skip if overlapping with a previously created junction
            if block_set & used:
                continue
            used.update(block_set)

            # Transfer positions from roads → junctions
            for pos in block:
                roads.discard(pos)
                junctions.add(pos)

            # Create and register the controller
            jid = f"J_{vx}_{hy}"
            controller = JunctionController(
                model.next_id(), model, jid, block,
                green_duration=green_duration,
                yellow_duration=yellow_duration,
                yellow_max_wait=yellow_max_wait,
            )
            junction_controllers[jid] = controller
            model.schedule.add(controller)

            # Place traffic-light indicators around the junction
            _place_approach_lights(model, controller, top_left_x, top_left_y,
                                   v_w, h_w, width, height)

    return junctions, junction_controllers


def _place_approach_lights(model, controller, tlx, tly, v_w, h_w, width, height):
    """
    Place four :class:`ApproachLight` agents (left, right, top, bottom)
    one cell outside the junction boundary.
    """
    positions_and_approaches = [
        ((tlx - 1, tly + h_w // 2), "H"),         # left approach
        ((tlx + v_w, tly + h_w // 2), "H"),        # right approach
        ((tlx + v_w // 2, tly - 1), "V"),           # bottom approach
        ((tlx + v_w // 2, tly + h_w), "V"),         # top approach
    ]
    for pos, approach in positions_and_approaches:
        lx, ly = pos
        if 0 <= lx < width and 0 <= ly < height:
            light = ApproachLight(model.next_id(), model, controller, approach, pos)
            model.grid.place_agent(light, pos)
            model.schedule.add(light)


def _place_road_cells(model, roads, h_centers, h_lanes, v_centers, v_lanes,
                      width, height):
    """
    Place :class:`RoadCell` agents with the correct lane direction.

    Direction assignment:
      • Horizontal centre ``y``: rows above centre → East, below → West
      • Vertical centre ``x``: columns right of centre → North, left → South
    """
    # Horizontal lanes
    for y in h_centers:
        lanes = h_lanes[y]
        for lane in range(lanes):
            row = y + lane
            for x in range(width):
                if (x, row) in roads:
                    model.grid.place_agent(
                        RoadCell(model.next_id(), model, "E"), (x, row))
        for lane in range(lanes):
            row = y - lane - 1
            for x in range(width):
                if (x, row) in roads:
                    model.grid.place_agent(
                        RoadCell(model.next_id(), model, "W"), (x, row))

    # Vertical lanes
    for x in v_centers:
        lanes = v_lanes[x]
        for lane in range(lanes):
            col = x + lane
            for y in range(height):
                if (col, y) in roads:
                    model.grid.place_agent(
                        RoadCell(model.next_id(), model, "N"), (col, y))
        for lane in range(lanes):
            col = x - lane - 1
            for y in range(height):
                if (col, y) in roads:
                    model.grid.place_agent(
                        RoadCell(model.next_id(), model, "S"), (col, y))


def _compute_original_road_directions(h_centers, h_lanes, v_centers, v_lanes,
                                      width, height):
    """
    Build a dict ``{(x, y): set_of_directions}`` recording which road
    directions originally overlapped every position (before junctions
    were carved out).  Used to set ``JunctionCell.valid_directions``.
    """
    road_directions = {}

    for y in h_centers:
        lanes = h_lanes[y]
        for lane in range(lanes):
            row = y + lane
            for x in range(width):
                if 0 <= row < height:
                    road_directions.setdefault((x, row), set()).add("E")
        for lane in range(lanes):
            row = y - lane - 1
            for x in range(width):
                if 0 <= row < height:
                    road_directions.setdefault((x, row), set()).add("W")

    for x in v_centers:
        lanes = v_lanes[x]
        for lane in range(lanes):
            col = x + lane
            for y in range(height):
                if 0 <= col < width:
                    road_directions.setdefault((col, y), set()).add("N")
        for lane in range(lanes):
            col = x - lane - 1
            for y in range(height):
                if 0 <= col < width:
                    road_directions.setdefault((col, y), set()).add("S")

    return road_directions


def _place_junction_cells(model, junction_controllers, road_directions):
    """Place :class:`JunctionCell` agents for every cell in every junction."""
    for jid, ctrl in junction_controllers.items():
        for pos in ctrl.block_cells:
            valid_dirs = road_directions.get(pos, set())
            jc = JunctionCell(model.next_id(), model, jid, valid_directions=valid_dirs)
            model.grid.place_agent(jc, pos)


def _find_entry_cells(model, width, height):
    """
    Identify grid-edge road cells where new cars can spawn.

    A cell qualifies if it sits on the grid border **and** its road
    direction points inward (e.g. an East-bound cell on the left edge).
    """
    entry_cells = []

    for x in range(width):
        # Bottom row — only North-bound lanes
        rc = _road_at(model, (x, 0))
        if rc and rc.direction == "N":
            entry_cells.append(((x, 0), "N"))
        # Top row — only South-bound lanes
        rc = _road_at(model, (x, height - 1))
        if rc and rc.direction == "S":
            entry_cells.append(((x, height - 1), "S"))

    for y in range(height):
        # Left column — only East-bound lanes
        rc = _road_at(model, (0, y))
        if rc and rc.direction == "E":
            entry_cells.append(((0, y), "E"))
        # Right column — only West-bound lanes
        rc = _road_at(model, (width - 1, y))
        if rc and rc.direction == "W":
            entry_cells.append(((width - 1, y), "W"))

    return entry_cells


def _find_exit_cells(model, width, height):
    """
    Identify grid-edge road cells where cars leave the grid.

    An exit cell sits on the border and its direction points **outward**
    (e.g. a North-bound cell on the top row, or an East-bound cell on
    the right column).

    Returns a dict with:
        ``exit_cells``  — ``{'N': [...], 'S': [...], 'E': [...], 'W': [...]}``
        ``exit_groups``  — list of ``(edge, [pos, ...])`` where each
                          group is a cluster of adjacent same-direction
                          lanes belonging to the same corridor.  Two
                          North-bound lanes at (15,39) and (16,39) form
                          one group.
    """
    exits = {"N": [], "S": [], "E": [], "W": []}

    for x in range(width):
        # Top row — North-bound lanes exit here
        rc = _road_at(model, (x, height - 1))
        if rc and rc.direction == "N":
            exits["N"].append((x, height - 1))
        # Bottom row — South-bound lanes exit here
        rc = _road_at(model, (x, 0))
        if rc and rc.direction == "S":
            exits["S"].append((x, 0))

    for y in range(height):
        # Right column — East-bound lanes exit here
        rc = _road_at(model, (width - 1, y))
        if rc and rc.direction == "E":
            exits["E"].append((width - 1, y))
        # Left column — West-bound lanes exit here
        rc = _road_at(model, (0, y))
        if rc and rc.direction == "W":
            exits["W"].append((0, y))

    # Group adjacent exit cells into logical exits.
    # Two lanes are "adjacent" if they differ by 1 in the coordinate
    # that varies along the edge (x for N/S edges, y for E/W edges).
    exit_groups = []
    for edge in ("N", "S", "E", "W"):
        cells = sorted(exits[edge])
        if not cells:
            continue
        # Which coordinate varies along this edge?
        axis = 0 if edge in ("N", "S") else 1   # x for top/bottom, y for left/right
        group = [cells[0]]
        for c in cells[1:]:
            if c[axis] == group[-1][axis] + 1:
                group.append(c)    # adjacent — same corridor
            else:
                exit_groups.append((edge, list(group)))
                group = [c]
        exit_groups.append((edge, list(group)))

    return {"exit_cells": exits, "exit_groups": exit_groups}


def _road_at(model, pos):
    """Return the RoadCell at ``pos``, or None."""
    return next(
        (a for a in model.grid.get_cell_list_contents([pos])
         if isinstance(a, RoadCell)),
        None,
    )
