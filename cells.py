"""
cells.py — Passive grid-cell agents.

These agents do not move or make decisions. They represent the physical
infrastructure of the road network (road lanes, junction surfaces, and
traffic-light indicators) and are placed on the MESA grid so the
visualization layer can draw them and active agents (cars, controllers)
can query what type of surface a grid position is.
"""

from mesa import Agent


# ═══════════════════════════════════════════════════════════════════════
#  RoadCell — one cell of a one-way lane
# ═══════════════════════════════════════════════════════════════════════
class RoadCell(Agent):
    """
    A single cell of a one-directional road lane.

    Attributes
    ----------
    direction : str
        One of 'N', 'S', 'E', 'W' — the only direction a car may
        travel while occupying this cell.
    car : CarAgent | None
        Reference to the car currently occupying this cell (used for
        quick occupancy checks without scanning the full cell contents).
    """

    def __init__(self, unique_id, model, direction):
        super().__init__(unique_id, model)
        self.direction = direction
        self.car = None  # set by CarAgent.advance() on arrival / departure


# ═══════════════════════════════════════════════════════════════════════
#  JunctionCell — one cell inside a junction block
# ═══════════════════════════════════════════════════════════════════════
class JunctionCell(Agent):
    """
    A single cell that belongs to a junction area.

    Junctions are rectangular blocks where horizontal and vertical
    roads overlap.  Each JunctionCell knows which junction it belongs
    to and which lane directions originally crossed this position
    (used by the pathfinding system to understand connectivity).

    Attributes
    ----------
    junction_id : str
        Identifier of the parent junction (e.g. "J_12_6").
    car : CarAgent | None
        Reference to the car currently on this cell.
    valid_directions : set[str]
        Set of directions ('N', 'S', 'E', 'W') that were present in
        the road lanes originally overlapping this position (before
        the junction was carved out of the road grid).
    """

    def __init__(self, unique_id, model, junction_id, valid_directions=None):
        super().__init__(unique_id, model)
        self.junction_id = junction_id
        self.car = None
        self.valid_directions = valid_directions or set()


# ═══════════════════════════════════════════════════════════════════════
#  ApproachLight — visual traffic-light indicator
# ═══════════════════════════════════════════════════════════════════════
class ApproachLight(Agent):
    """
    A visual-only agent placed one cell before a junction entrance.

    It has no internal logic: its colour is derived at render time
    from the linked :class:`JunctionController`'s current phase and
    the approach type (horizontal / vertical).

    Attributes
    ----------
    controller : JunctionController
        The controller that governs this junction.
    approach : str
        ``'H'`` for a horizontal (East/West) approach,
        ``'V'`` for a vertical (North/South) approach.
    """

    def __init__(self, unique_id, model, controller, approach_type, pos):
        super().__init__(unique_id, model)
        self.controller = controller
        self.approach = approach_type

    def step(self):
        # State is derived from the controller — nothing to compute.
        pass


# ═══════════════════════════════════════════════════════════════════════
#  RoadEvent — obstacle blocking a road cell
# ═══════════════════════════════════════════════════════════════════════
class RoadEvent(Agent):
    """
    A road obstacle (accident, roadwork) that blocks one cell.

    Cars cannot pass through a RoadEvent.  If a parallel lane exists in
    the same direction, cars on the blocked lane will attempt to merge
    into it.  Cars on the free parallel lane yield periodically to
    create gaps for merging traffic.

    Attributes
    ----------
    direction : str
        Direction of the road this event blocks ('N', 'S', 'E', 'W').
    """

    def __init__(self, unique_id, model, direction):
        super().__init__(unique_id, model)
        self.direction = direction
