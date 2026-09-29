"""
car.py — CarAgent: the moving vehicle.

Cars use MESA's SimultaneousActivation scheduler, but the model splits
the tick into three phases:

    1. ``step()``  — all agents compute their *intended* next position
       in parallel (reading the grid but not yet writing).
    2. **Deadlock resolution** — the model detects cyclic blockages
       inside junctions (where each car's desired cell is occupied by
       the next car in a closed loop) and executes a coordinated
       rotation so all cycle members advance simultaneously.
    3. ``advance()`` — all agents apply their move (writing to the grid).

A car's ``step()`` handles three mutually exclusive situations:

    • **Inside a junction** — navigate toward the correct exit based on
      the car's ``intended_turn`` (straight / left / right / uturn).
      Uses BFS pathfinding through junction cells.  When the BFS path
      is blocked by a junction mate, the car records its *desire* so
      the cycle resolver can detect and break deadlocks.
    • **Approaching a junction** — obey the traffic light: only enter on
      GREEN for the matching approach AND only if the exit road in the
      car's desired direction has at least one free cell.  If the exit
      is fully occupied the car waits on the approach road, letting
      the jam propagate backward rather than gridlocking the junction.
    • **On a normal road** — move forward one cell if the next cell is a
      same-direction road and is unoccupied.
"""

import random
from collections import deque

from mesa import Agent

from cells import RoadCell, JunctionCell, RoadEvent


# Direction → (dx, dy) movement deltas
DELTAS = {
    "N": (0, 1),
    "S": (0, -1),
    "E": (1, 0),
    "W": (-1, 0),
}

# Road-event lane-change / merge constants
EVENT_DETECT_DIST = 6    # cells ahead to scan for an event (triggers lane change)
MERGE_ZONE_DEPTH = 8     # cells before event where free-lane cars yield
YIELD_INTERVAL = 3       # free-lane cars stop every N ticks in the merge zone

# Maps (entry_direction, intended_turn) → desired exit direction
TURN_MAP = {
    "N": {"left": "W", "right": "E", "uturn": "S"},
    "S": {"left": "E", "right": "W", "uturn": "N"},
    "E": {"left": "N", "right": "S", "uturn": "W"},
    "W": {"left": "S", "right": "N", "uturn": "E"},
}


class CarAgent(Agent):
    """
    A vehicle that travels the road network.

    Attributes
    ----------
    direction : str
        Current travel direction ('N', 'S', 'E', 'W').
    intended_turn : str
        What the car plans to do at the next junction:
        ``'straight'``, ``'left'``, or ``'right'``.
        Computed from route planning toward the car's exit target.
    exit_target : tuple[int, int] | None
        The (x, y) position this car wants to reach to leave the grid.
    exit_edge : str | None
        Which edge the car's exit is on: ``'N'``, ``'S'``, ``'E'``, ``'W'``.
    next_pos : tuple | None
        Position computed during ``step()`` and applied in ``advance()``.
    to_remove : bool
        Set to True when the car leaves the grid edge.
    """

    # ── Construction ─────────────────────────────────────────────

    def __init__(self, unique_id, model, direction,
                 exit_target=None, exit_edge=None, exit_group=None,
                 route=None):
        super().__init__(unique_id, model)
        self.direction = direction
        self.exit_target = exit_target
        self.exit_edge = exit_edge
        self.exit_group = exit_group      # index into model.exit_groups
        self.route = list(route) if route else []  # [(junction_id, turn), ...]
        self.intended_turn = "straight"   # will be set by route planning
        self.next_pos = None
        self.to_remove = False

        # ── Velocity (Nagel-Schreckenberg microscopic model) ──────
        self.velocity = 0  # current speed in cells/step (starts at 0)

        # Junction traversal state (set on entry, cleared on exit)
        self._junction_entry_id = None   # which junction we entered
        self._entry_direction = None     # direction when we entered
        self._turn_wait_counter = 0      # ticks spent waiting for desired path
        self._junction_time = 0          # total ticks spent in current junction
        self._desired_junction_move = None  # blocked target (for cycle detection)

        # Analytics / testing helpers
        self._junction_exit_count = 0
        history_maxlen = max(0, int(getattr(model, "car_history_maxlen", 0)))
        self.position_history = deque(maxlen=history_maxlen)
        self.intent_history = deque(maxlen=history_maxlen)

    # ═════════════════════════════════════════════════════════════
    #  Route planning — choose intended_turn based on exit target
    # ═════════════════════════════════════════════════════════════

    # Opposite directions — going the wrong way entirely
    _OPPOSITE = {"N": "S", "S": "N", "E": "W", "W": "E"}

    def plan_turn_for_junction(self):
        """
        Decide ``intended_turn`` based on the car's pre-computed route.

        The route is a list of ``(junction_id, turn_type)`` waypoints.
        At each junction the car encounters, it checks whether that
        junction matches the **next** waypoint.  If so, it takes the
        prescribed turn and pops the waypoint.  Otherwise it goes
        straight (still heading toward the correct waypoint junction).

        Before following the route, the car queries the local compute
        station for jam alerts.  If the station says our travel
        direction leads to a jammed junction ahead, we ask the model
        to reroute us around it.

        This handles multi-turn routes correctly: a car that needs to
        go East → North → West will follow three waypoints in sequence.
        """
        if not self.route:
            # No more waypoints — go straight toward the exit edge.
            # If we're already heading toward the exit edge, straight is right.
            # Otherwise fall back to a simple distance-based choice.
            self._plan_fallback()
            return

        # What is the next junction we need to turn at?
        next_jid, next_turn = self.route[0]

        # Are we at (or approaching) that junction?
        # Check if we're inside it:
        if self._junction_entry_id == next_jid:
            self.intended_turn = next_turn
            return

        # Check if the next junction ahead matches our waypoint:
        ahead_jid = self._peek_next_junction_id()
        if ahead_jid == next_jid:
            self.intended_turn = next_turn
            return

        # We're not at the waypoint junction yet — go straight to reach it.
        self.intended_turn = "straight"

    def _advance_route_waypoint(self, completed_junction_id):
        """
        Called when the car exits a junction.  If that junction was the
        next waypoint, pop it from the route.
        """
        if self.route and self.route[0][0] == completed_junction_id:
            self.route.pop(0)

    def _plan_fallback(self):
        """
        Simple distance-based fallback when the route is empty
        (car just needs to reach the edge).  Go straight if heading
        toward the exit edge; otherwise pick the best turn.
        """
        if self.exit_target is None:
            self.intended_turn = "straight"
            return

        if self.exit_edge == self.direction:
            self.intended_turn = "straight"
            return

        tx, ty = self.exit_target
        cx, cy = self.pos
        best_turn = "straight"
        best_dist = float("inf")

        for turn in ("straight", "left", "right"):
            if turn == "straight":
                out_dir = self.direction
            else:
                out_dir = TURN_MAP.get(self.direction, {}).get(turn)
                if out_dir is None:
                    continue
            dx, dy = DELTAS[out_dir]
            nx, ny = cx + dx, cy + dy
            dist = abs(tx - nx) + abs(ty - ny)
            if out_dir == self.exit_edge:
                dist -= 2
            if dist < best_dist:
                best_dist = dist
                best_turn = turn

        self.intended_turn = best_turn

    def _peek_next_junction_id(self):
        """
        Scan ahead along the current direction to find the next
        junction cell.  Returns its ``junction_id`` or ``None``.

        Uses the model's junction data for an efficient lookup: checks
        all junction controllers and picks the nearest one that lies
        directly ahead in the current direction.
        """
        cx, cy = self.pos
        dx, dy = DELTAS.get(self.direction, (0, 0))
        best_id = None
        best_dist = float("inf")

        for jid, ctrl in self.model.junction_controllers.items():
            block = ctrl.block_cells
            for bx, by in block:
                # Is this cell ahead of us in our travel direction?
                offset_x = bx - cx
                offset_y = by - cy

                # Must be strictly ahead (positive projection)
                if dx != 0:
                    if dx * offset_x <= 0:
                        continue
                    if offset_y != 0:
                        # Must be on the same row(s) — allow small lateral
                        # tolerance for multi-lane roads
                        if abs(offset_y) > 3:
                            continue
                    dist = abs(offset_x)
                elif dy != 0:
                    if dy * offset_y <= 0:
                        continue
                    if offset_x != 0:
                        if abs(offset_x) > 3:
                            continue
                    dist = abs(offset_y)
                else:
                    continue

                if dist < best_dist:
                    best_dist = dist
                    best_id = jid
                    break  # found one cell in this junction ahead, that's enough

        return best_id

    # ═════════════════════════════════════════════════════════════
    #  step() — compute intended next position (read-only phase)
    # ═════════════════════════════════════════════════════════════

    def step(self):
        """Decide where to move this tick (without actually moving)."""
        x, y = self.pos

        # Calculate the cell directly ahead and the approach type
        nxt, approach = self._next_cell_and_approach(x, y)

        # ── Edge of grid → flag for removal ──────────────────────
        if nxt is None or self.model.grid.out_of_bounds(nxt):
            self.next_pos = None
            self.to_remove = True
            self.model._to_remove.add(self)
            return

        # ── Are we currently on a junction cell? ─────────────────
        cur_junction_cell = self._junction_cell_at(self.pos)

        # ── CASE A: Inside a junction → junction navigation ──────
        if cur_junction_cell:
            self._step_inside_junction(cur_junction_cell)
            return

        # ── CASE B: Approaching a junction → traffic-light check ─
        nxt_junction_cell = self._junction_cell_at(nxt)
        if nxt_junction_cell:
            self._step_approaching_junction(nxt_junction_cell, nxt, approach)
            return

        # ── CASE C: Normal road — NaSch velocity model ───────────
        self._step_road_movement_nasch()

    # ─────────────────────────────────────────────────────────────
    #  CASE A helpers: inside a junction
    # ─────────────────────────────────────────────────────────────

    def _step_inside_junction(self, junction_cell):
        """
        Navigate through the junction toward the correct exit.

        Strategy (tried in order):
          0. **Time-based safety limits** (checked first):
             - >20 ticks: exit-focused only (BFS forward + road exits).
             - >15 ticks: force turn to ``straight`` if still turning.
          1. Exit directly if adjacent road matches desired direction.
          2. Follow BFS shortest path toward desired exit.
             If blocked by a junction mate, apply **yield-or-proceed**:
             the lower-``unique_id`` car yields (stays put) while the
             higher-id car sidesteps.  This breaks mutual-sidestep
             oscillation and lets one car clear the junction first.
          3. After waiting too long (>8 ticks), fall back to going
             straight instead of turning.
          4. Emergency: move to any free neighbour inside the junction,
             or exit onto any free adjacent road.

        If all options fail, the car records its blocked BFS target in
        ``_desired_junction_move`` so the model's cycle-deadlock
        resolver can detect and break circular wait chains.
        """
        controller = self.model.junction_controllers.get(junction_cell.junction_id)
        if not (controller and self.pos in controller.block_cells):
            # Safety: if we're somehow on a junction cell without a valid
            # controller, just stay put.
            self.next_pos = self.pos
            return

        # ── Record entry metadata on first tick inside junction ──
        if self._junction_entry_id is None:
            self._junction_entry_id = junction_cell.junction_id
            self._entry_direction = self.direction
            self._junction_time = 0

        self._junction_time += 1
        self._desired_junction_move = None  # reset each tick
        block_cells = set(controller.block_cells)
        reserved = getattr(self.model, "_reserved_targets", None)

        # ── Deadlock prevention for multi-car junctions ──────────
        # With SimultaneousActivation every car reads the same grid
        # state.  Instead of blanket-serialising all junction cars by
        # unique_id (which forces cars on non-overlapping paths to
        # wait for each other), we let every car attempt to navigate
        # and rely on the reservation system (_reserved_targets) to
        # prevent two cars from claiming the same cell.
        #
        # To avoid the "mutual sidestep" deadlock — where two cars
        # reading the same state each try to move into the other's
        # current cell — each car also treats cells occupied by other
        # junction-mates as obstacles *unless* the cell is obviously
        # being vacated (the mate has already reserved a different
        # target this tick).
        #
        # Yield-or-proceed: when our BFS target is blocked by a mate,
        # only ONE of the two cars sidesteps (the higher unique_id).
        # The other yields (stays put) providing a stable obstacle the
        # mover can navigate around.  This breaks the symmetry that
        # causes both cars to sidestep simultaneously and oscillate.
        _yielding_positions = set()  # cells held by mates that already moved
        _mate_positions = set()      # cells currently held by ALL mates
        _junction_mates = []         # other cars inside this junction
        if reserved is not None:
            _junction_mates = [
                a for a in self.model.grid.get_cell_list_contents(
                    list(block_cells))
                if isinstance(a, CarAgent)
                   and a._junction_entry_id == junction_cell.junction_id
                   and a.unique_id != self.unique_id
            ]
            for mate in _junction_mates:
                _mate_positions.add(mate.pos)
                # If this mate has already chosen a next_pos that
                # differs from its current pos, its current cell will
                # be vacated — we can treat it as passable.
                if (mate.next_pos is not None
                        and mate.next_pos != mate.pos):
                    _yielding_positions.add(mate.pos)

        # ── Desired exit direction ───────────────────────────────
        desired_out_dir = self._compute_desired_exit_direction()

        # ── Step 0a: Hard time cap — exit-focused navigation ─────
        # After too many ticks, restrict movement to direct exits
        # and BFS-forward steps toward any exit (no sidestepping).
        # Crucially, only advance along a BFS path if the *exit road*
        # at the end is actually free — otherwise the car bounces
        # toward a blocked exit and back (oscillation).
        # When BFS returns a path of length 1, the car is already
        # adjacent to a free exit — step directly onto the road
        # instead of ignoring the result and taking a longer
        # internal path in another direction (which causes bounce).
        if self._junction_time > 20:
            # 1) Direct exit on desired direction
            if desired_out_dir and self._try_direct_exit(desired_out_dir, reserved):
                return
            # 2) BFS toward nearest exit — only if the exit road is free
            for try_dir in [desired_out_dir] + [
                d for d in ("N", "S", "E", "W") if d != desired_out_dir
            ]:
                if not try_dir:
                    continue
                path, reason = self.model.find_junction_exit_path(
                    junction_cell.junction_id, self.pos, try_dir,
                    check_occupancy=True,   # only find paths with free exit
                )
                if path and len(path) == 1:
                    # Car is already adjacent to the free exit road
                    # in this direction — step directly onto it.
                    dx, dy = DELTAS[try_dir]
                    road_pos = (path[0][0] + dx, path[0][1] + dy)
                    if (self._cell_is_free(road_pos)
                            and not self._cell_has_event(road_pos)
                            and self._try_reserve_move(road_pos, reserved)):
                        self.direction = try_dir
                        return
                elif path and len(path) >= 2:
                    cell_free = (
                        self._cell_is_free(path[1])
                        or path[1] in _yielding_positions
                    )
                    if cell_free and self._try_reserve_move(path[1], reserved):
                        return
            # 3) Try exiting onto ANY adjacent road (no internal moves)
            if self._try_exit_any_road(block_cells, reserved):
                return
            # 4) Stay put — no sidestepping
            if reserved is not None:
                reserved.add(self.pos)
            self.next_pos = self.pos
            return

        # ── Step 0b: Forced straight after prolonged stay ────────
        if self._junction_time > 15 and self.intended_turn != "straight":
            self.intended_turn = "straight"
            self._turn_wait_counter = 0
            desired_out_dir = self._compute_desired_exit_direction()

        # ── Step 1: Try to exit directly onto desired exit road ──
        if desired_out_dir and self._try_direct_exit(desired_out_dir, reserved):
            return

        # ── Step 2: BFS path toward desired exit ─────────────────
        # First try to find a path to a FREE exit road (e.g. the
        # other lane) while still traversing through occupied junction
        # cells (geometric path).  If no free exit exists, fall back
        # to the pure geometric BFS so sidestep/wait logic can still
        # navigate the car toward the exit direction.
        path = None
        if desired_out_dir:
            path, reason = self.model.find_junction_exit_path(
                junction_cell.junction_id, self.pos, desired_out_dir,
                check_occupancy=False,
                require_free_exit=True,
            )
            if path is None:
                # No free exit lane — use pure geometric path
                path, _ = self.model.find_junction_exit_path(
                    junction_cell.junction_id, self.pos, desired_out_dir,
                    check_occupancy=False,
                    require_free_exit=False,
                )

        if path and len(path) >= 2:
            # Try the next step on the BFS path
            target_free = self._cell_is_free(path[1]) or path[1] in _yielding_positions
            if target_free and self._try_reserve_move(path[1], reserved):
                return
            # Path blocked — record desire for cycle-deadlock detection
            if path[1] in block_cells:
                self._desired_junction_move = path[1]

            # ── Yield-or-proceed ─────────────────────────────────
            # If a junction mate occupies our target cell, only one
            # car should sidestep; the other yields (stays put).
            # The lower unique_id yields, giving the higher-id car
            # a stable obstacle to navigate around.  This prevents
            # both cars from sidestepping into each other's paths
            # in the same tick (oscillation).
            #
            # If no mate is blocking (cell is reserved or occupied
            # by a non-junction agent), both BFS and sidestep are
            # attempted normally.
            blocking_mate = None
            for mate in _junction_mates:
                if mate.pos == path[1]:
                    blocking_mate = mate
                    break

            if blocking_mate is not None and self.unique_id < blocking_mate.unique_id:
                # YIELD: stay put, let the higher-id car navigate.
                # Don't sidestep — our stillness gives the mate a
                # fixed grid to plan around.  The cycle detector
                # handles genuine circular deadlocks where yielding
                # alone isn't enough.
                self._turn_wait_counter += 1
            else:
                # PROCEED: we have priority (or not blocked by a mate).
                # Try sidestepping to an adjacent junction cell that
                # still has a valid path to the same exit.
                if self._try_sidestep(block_cells, junction_cell, desired_out_dir,
                                      reserved, _yielding_positions):
                    self._desired_junction_move = None
                    return
                # All options blocked this tick — wait
                self._turn_wait_counter += 1

        elif path and len(path) == 1:
            # Sitting on the exit cell but the road beyond is occupied
            self._turn_wait_counter += 1

        else:
            # No geometric path for this turn — immediate fallback to straight
            if self.intended_turn != "straight":
                self.intended_turn = "straight"
                self._turn_wait_counter = 0
                self.next_pos = self.pos
                return

        # ── Step 3: Waited too long → fall back to straight ──────
        if self._turn_wait_counter > 8 and self.intended_turn != "straight":
            self.intended_turn = "straight"
            self._turn_wait_counter = 0
            self._desired_junction_move = None
            self.next_pos = self.pos
            return

        # ── Step 4: Emergency — try any free neighbour / exit ────
        if self._turn_wait_counter > 4:
            if self._try_emergency_move(block_cells, reserved):
                self._desired_junction_move = None
                return

        # Nothing worked — stay in place and reserve our cell so
        # mates processed later don't try to move into it.
        if reserved is not None:
            reserved.add(self.pos)
        self.next_pos = self.pos

    def _compute_desired_exit_direction(self):
        """Return the compass direction the car wants to exit toward."""
        if self.intended_turn == "straight":
            return self._entry_direction
        return TURN_MAP.get(self._entry_direction, {}).get(self.intended_turn)

    def _try_direct_exit(self, desired_out_dir, reserved):
        """
        If the cell adjacent in ``desired_out_dir`` is a matching road
        cell and it's free, reserve it and move there.
        """
        dx, dy = DELTAS[desired_out_dir]
        exit_pos = (self.pos[0] + dx, self.pos[1] + dy)
        if not self._in_bounds(exit_pos):
            return False
        exit_road = next(
            (a for a in self.model.grid.get_cell_list_contents([exit_pos])
             if isinstance(a, RoadCell) and a.direction == desired_out_dir),
            None,
        )
        if exit_road and exit_road.car is None and not self._cell_has_event(exit_pos):
            return self._try_reserve_move(exit_pos, reserved)
        return False

    def _try_sidestep(self, block_cells, junction_cell, desired_out_dir,
                      reserved, yielding_positions=None):
        """
        Move to a free adjacent junction cell that still has a valid
        BFS path to the desired exit.
        """
        if yielding_positions is None:
            yielding_positions = set()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            side = (self.pos[0] + dx, self.pos[1] + dy)
            side_free = self._cell_is_free(side) or side in yielding_positions
            if side in block_cells and side != self.pos and side_free:
                alt_path, _ = self.model.find_junction_exit_path(
                    junction_cell.junction_id, side, desired_out_dir,
                    check_occupancy=False,
                )
                if alt_path is not None:
                    if self._try_reserve_move(side, reserved):
                        return True
        return False

    def _try_emergency_move(self, block_cells, reserved):
        """
        Last resort: move to any free junction neighbour, or exit onto
        any free adjacent road regardless of intended turn.
        """
        # Try free neighbours inside the junction
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nbr = (self.pos[0] + dx, self.pos[1] + dy)
            if nbr in block_cells and self._cell_is_free(nbr):
                if self._try_reserve_move(nbr, reserved):
                    return True

        # Try exiting onto ANY adjacent road that is free
        for direction, (dx, dy) in DELTAS.items():
            out = (self.pos[0] + dx, self.pos[1] + dy)
            if out in block_cells or not self._in_bounds(out):
                continue
            road = next(
                (a for a in self.model.grid.get_cell_list_contents([out])
                 if isinstance(a, RoadCell) and a.direction == direction),
                None,
            )
            if road and road.car is None and not self._cell_has_event(out):
                if self._try_reserve_move(out, reserved):
                    self.direction = direction  # align with the exit road
                    return True
        return False

    def _try_exit_any_road(self, block_cells, reserved):
        """
        Try exiting onto ANY adjacent road that is free, regardless of
        intended turn.  Unlike ``_try_emergency_move``, this does NOT
        move to a free junction neighbour — it only attempts road exits.
        Used by the hard time cap to avoid internal oscillation.
        """
        for direction, (dx, dy) in DELTAS.items():
            out = (self.pos[0] + dx, self.pos[1] + dy)
            if out in block_cells or not self._in_bounds(out):
                continue
            road = next(
                (a for a in self.model.grid.get_cell_list_contents([out])
                 if isinstance(a, RoadCell) and a.direction == direction),
                None,
            )
            if road and road.car is None and not self._cell_has_event(out):
                if self._try_reserve_move(out, reserved):
                    self.direction = direction
                    return True
        return False

    # ─────────────────────────────────────────────────────────────
    #  CASE B helper: approaching a junction
    # ─────────────────────────────────────────────────────────────

    def _step_approaching_junction(self, nxt_junction_cell, nxt, approach):
        """
        The cell ahead (``nxt``) is a junction cell. Enter only if:
          • The traffic light is GREEN for our approach, AND
          • The entry cell is not already occupied, AND
          • At least one exit road cell in the car's desired exit
            direction is free (no car occupying it).

        The third condition prevents cars from entering a junction
        when they have nowhere to go — the exit road is fully
        blocked.  Instead the car waits on the approach road, letting
        the jam propagate backward along the approach road naturally
        rather than gridlocking within the junction.

        Re-plans the intended turn before entering so the car picks
        the best route through this junction toward its exit target.
        """
        # Re-plan turn right before we enter (direction may have
        # changed since the last planning, e.g. after a fallback exit)
        self.plan_turn_for_junction()

        controller = self.model.junction_controllers.get(nxt_junction_cell.junction_id)

        if controller is None:
            # No controller — allow entry if the cell is empty
            self.next_pos = nxt if nxt_junction_cell.car is None else self.pos
            return

        if controller.allow_entry(approach) and nxt_junction_cell.car is None:
            # ── Exit-road gate ───────────────────────────────────
            # Compute the direction the car wants to exit toward.
            # If every exit road cell in that direction is occupied,
            # do NOT enter — wait outside instead.
            if self.intended_turn == "straight":
                desired_exit = self.direction
            else:
                desired_exit = TURN_MAP.get(self.direction, {}).get(
                    self.intended_turn, self.direction
                )

            jid = nxt_junction_cell.junction_id
            if not self.model.junction_exit_has_free_road(jid, desired_exit):
                # Exit blocked — stay put, let jam propagate backward
                self.next_pos = self.pos
                self.velocity = 0
                return

            reserved = getattr(self.model, "_reserved_targets", None)
            if reserved is None or nxt not in reserved:
                if reserved is not None:
                    reserved.add(nxt)
                self.next_pos = nxt
                self.velocity = 1  # entering junction at low speed
            else:
                self.next_pos = self.pos
                self.velocity = 0
        else:
            self.next_pos = self.pos
            self.velocity = 0   # waiting at red light

    # ─────────────────────────────────────────────────────────────
    #  CASE C: NaSch velocity model for normal road cells
    # ─────────────────────────────────────────────────────────────

    def _scan_ahead_distance(self):
        """
        Scan ahead along the current direction and return the number of
        **free** road cells before the next obstacle.

        Obstacles that stop the scan:
        • Another car on a road cell.
        • A junction cell (car must stop before it for CASE B logic).
        • A road event (accident / obstacle).
        • A non-road cell or out-of-bounds position.

        Returns an integer in ``[0, v_max]``.
        """
        dx, dy = DELTAS[self.direction]
        v_max = getattr(self.model, 'v_max', 4)
        free = 0

        for step in range(1, v_max + 2):   # one extra for robust braking
            check_pos = (self.pos[0] + dx * step, self.pos[1] + dy * step)

            if not self._in_bounds(check_pos):
                break   # grid edge — last valid cell was already counted

            if self._junction_cell_at(check_pos):
                break   # junction ahead — stop before it

            if self._cell_has_car(check_pos):
                break   # car ahead — stop before it

            if self._cell_has_event(check_pos):
                break   # road event (obstacle) — stop before it

            road = next(
                (a for a in self.model.grid.get_cell_list_contents([check_pos])
                 if isinstance(a, RoadCell) and a.direction == self.direction),
                None,
            )
            if road is None:
                break   # not a valid road cell

            free = step

        return min(free, v_max)

    def _step_road_movement_nasch(self):
        """
        Nagel-Schreckenberg cellular-automaton rules per vehicle:

        1. **Acceleration**   — ``v ← min(v + 1, v_max)``
        2. **Braking**        — ``if v > gap: v ← gap``  (collision avoidance)
        3. **Random slowing** — with prob *p*: ``v ← max(v − 1, 0)``
        4. **Movement**       — advance *v* cells forward.

        Before the standard NaSch rules:
        • If a road event is detected ahead, attempt a lane change
          to the parallel lane.
        • If the car is on the free lane in a merge zone, it yields
          every ``YIELD_INTERVAL`` ticks to create gaps for merging
          traffic from the blocked lane.
        """
        v_max = getattr(self.model, 'v_max', 4)
        p_slow = getattr(self.model, 'random_slow_prob', 0.3)

        gap = self._scan_ahead_distance()

        # ── Road event: try lane change when obstacle ahead ─────
        if self._event_ahead_within(EVENT_DETECT_DIST):
            if self._try_event_lane_change():
                return

        # ── Free-lane yielding in merge zone ─────────────────
        if self._in_event_merge_zone():
            tick = getattr(self.model, "_tick_count", 0)
            if tick % YIELD_INTERVAL == 0:
                self.velocity = 0
                self.next_pos = self.pos
                return

        # 1. Acceleration
        self.velocity = min(self.velocity + 1, v_max)

        # 2. Braking — never exceed the gap to the next obstacle
        if self.velocity > gap:
            self.velocity = gap

        # 3. Random slowing (models driver hesitation / perception delay)
        if self.velocity > 0 and random.random() < p_slow:
            self.velocity -= 1

        # 4. Movement
        if self.velocity <= 0:
            self.velocity = 0
            self.next_pos = self.pos
            return

        dx, dy = DELTAS[self.direction]
        target = (self.pos[0] + dx * self.velocity,
                  self.pos[1] + dy * self.velocity)

        # Off-grid → exit the simulation
        if not self._in_bounds(target):
            self.next_pos = None
            self.to_remove = True
            self.model._to_remove.add(self)
            return

        # Validate the target is a free road cell in our direction
        road_cell = next(
            (a for a in self.model.grid.get_cell_list_contents([target])
             if isinstance(a, RoadCell) and a.direction == self.direction),
            None,
        )
        if road_cell and road_cell.car is None and not self._cell_has_event(target):
            reserved = getattr(self.model, "_reserved_targets", None)
            if reserved is None or target not in reserved:
                if reserved is not None:
                    reserved.add(target)
                self.next_pos = target
            else:
                self.next_pos = self.pos
        else:
            self.next_pos = self.pos

    # ═════════════════════════════════════════════════════════════
    #  advance() — apply the move (write phase)
    # ═════════════════════════════════════════════════════════════

    def advance(self):
        """Actually move the car to ``next_pos`` and update cell bookkeeping."""
        # Record history for analysis / tests
        self.position_history.append(self.pos if not self.to_remove else None)
        self.intent_history.append(self.intended_turn)

        # ── Removal (left the grid) ──────────────────────────────
        if self.next_pos is None:
            self._free_current_cell()
            self._remove_from_simulation()
            return

        # ── Free the cell we're leaving ──────────────────────────
        self._free_current_cell()

        # ── Move to the new position ─────────────────────────────
        try:
            self.model.grid.move_agent(self, self.next_pos)
        except Exception:
            self.model.grid.place_agent(self, self.pos)

        # ── Mark the destination cell as occupied ────────────────
        dest_holder = self._holder_at(self.pos)
        if dest_holder:
            dest_holder.car = self

            # If we just moved from a junction onto a road, finalise the exit
            if isinstance(dest_holder, RoadCell):
                was_in_junction = self._junction_entry_id is not None
                self.direction = dest_holder.direction

                if was_in_junction:
                    # Pop the waypoint if we just completed it
                    self._advance_route_waypoint(self._junction_entry_id)
                    # Plan the next turn based on route to exit
                    self.plan_turn_for_junction()
                    self._turn_wait_counter = 0
                    self._junction_time = 0
                    self._junction_exit_count += 1
                    # Reset velocity — car re-enters normal road from stop
                    self.velocity = 1

                # Clear junction state regardless
                self._junction_entry_id = None
                self._entry_direction = None

    # ═════════════════════════════════════════════════════════════
    #  Utility helpers (private)
    # ═════════════════════════════════════════════════════════════

    def _next_cell_and_approach(self, x, y):
        """
        Return ``(next_position, approach_type)`` for the cell directly
        ahead in the car's current direction.

        ``approach_type`` is ``'H'`` or ``'V'`` — used when checking
        traffic-light permissions.
        """
        dx, dy = DELTAS.get(self.direction, (0, 0))
        nxt = (x + dx, y + dy)
        approach = "H" if self.direction in ("E", "W") else "V"
        return nxt, approach

    def _junction_cell_at(self, pos):
        """Return the JunctionCell at ``pos``, or None."""
        return next(
            (a for a in self.model.grid.get_cell_list_contents([pos])
             if isinstance(a, JunctionCell)),
            None,
        )

    def _cell_has_car(self, pos):
        """Return True if any CarAgent occupies ``pos``."""
        return any(
            isinstance(a, CarAgent)
            for a in self.model.grid.get_cell_list_contents([pos])
        )

    def _cell_is_free(self, pos):
        """Return True if no CarAgent occupies ``pos``."""
        return not self._cell_has_car(pos)

    def _cell_has_event(self, pos):
        """Return True if a RoadEvent (obstacle) occupies ``pos``."""
        return any(
            isinstance(a, RoadEvent)
            for a in self.model.grid.get_cell_list_contents([pos])
        )

    def _in_bounds(self, pos):
        """Return True if ``pos`` is within the grid."""
        return 0 <= pos[0] < self.model.width and 0 <= pos[1] < self.model.height

    # ── Road-event lane change / merge helpers ───────────────────

    def _event_ahead_within(self, max_dist):
        """Return True if a RoadEvent is within *max_dist* cells ahead."""
        dx, dy = DELTAS[self.direction]
        for step in range(1, max_dist + 1):
            check_pos = (self.pos[0] + dx * step, self.pos[1] + dy * step)
            if not self._in_bounds(check_pos):
                return False
            if self._cell_has_event(check_pos):
                return True
            # Stop scanning past a junction (event can't be there)
            if self._junction_cell_at(check_pos):
                return False
        return False

    def _try_event_lane_change(self):
        """
        Attempt a **diagonal** lane change — one cell forward + one
        cell lateral — to a parallel lane (same direction) so the car
        avoids a road event ahead.

        Returns True if the lane change succeeds.
        """
        dx, dy = DELTAS[self.direction]
        if self.direction in ("E", "W"):
            lateral_offsets = [(0, 1), (0, -1)]
        else:
            lateral_offsets = [(1, 0), (-1, 0)]

        reserved = getattr(self.model, "_reserved_targets", None)

        for lx, ly in lateral_offsets:
            # Diagonal target: one step forward + one step lateral
            target = (self.pos[0] + dx + lx, self.pos[1] + dy + ly)
            if not self._in_bounds(target):
                continue
            # Must be a same-direction road cell that is free and has
            # no event of its own.
            road = next(
                (a for a in self.model.grid.get_cell_list_contents([target])
                 if isinstance(a, RoadCell) and a.direction == self.direction),
                None,
            )
            if (road and road.car is None
                    and not self._cell_has_event(target)
                    and not self._cell_has_car(target)):
                if reserved is None or target not in reserved:
                    if reserved is not None:
                        reserved.add(target)
                    self.next_pos = target
                    self.velocity = 1   # slow down during lane change
                    return True
        return False

    def _in_event_merge_zone(self):
        """
        Return True if this car is on the **free** parallel lane in the
        merge zone of a road event **and** at least one car is actually
        blocked (stopped or queued) behind the event on the adjacent
        lane.  If no car is stuck there, there is nothing to yield for.
        """
        events = getattr(self.model, "road_events", [])
        for event in events:
            if event.direction != self.direction:
                continue
            ep = event.pos
            if self.direction in ("E", "W"):
                lateral_dist = abs(self.pos[1] - ep[1])
                if lateral_dist != 1:
                    continue
                # Distance ahead to the event (positive = event is ahead)
                if self.direction == "E":
                    dist_ahead = ep[0] - self.pos[0]
                else:
                    dist_ahead = self.pos[0] - ep[0]
            else:   # N or S
                lateral_dist = abs(self.pos[0] - ep[0])
                if lateral_dist != 1:
                    continue
                if self.direction == "N":
                    dist_ahead = ep[1] - self.pos[1]
                else:
                    dist_ahead = self.pos[1] - ep[1]

            if 0 < dist_ahead <= MERGE_ZONE_DEPTH:
                # Check the blocked lane for a car behind the event
                if self._cars_blocked_behind_event(event):
                    return True
        return False

    def _cars_blocked_behind_event(self, event):
        """
        Return True if at least one car is queued behind *event* on the
        blocked lane (within ``MERGE_ZONE_DEPTH`` cells behind it).
        """
        ep = event.pos
        # "behind" the event = opposite to the travel direction
        dx, dy = DELTAS[event.direction]
        for step in range(1, MERGE_ZONE_DEPTH + 1):
            check = (ep[0] - dx * step, ep[1] - dy * step)
            if not self._in_bounds(check):
                break
            if self._cell_has_car(check):
                return True
            # Stop scanning past a junction — cars there aren't
            # blocked by this event.
            if self._junction_cell_at(check):
                break
        return False

    def _try_reserve_move(self, pos, reserved):
        """
        Attempt to reserve ``pos`` for this tick and set it as our
        ``next_pos``.  Returns True on success.
        """
        if reserved is None or pos not in reserved:
            if reserved is not None:
                reserved.add(pos)
            self.next_pos = pos
            self._turn_wait_counter = 0
            return True
        return False

    def _holder_at(self, pos):
        """Return the first RoadCell or JunctionCell at ``pos`` (or None)."""
        holders = [
            a for a in self.model.grid.get_cell_list_contents([pos])
            if isinstance(a, (RoadCell, JunctionCell))
        ]
        return holders[0] if holders else None

    def _free_current_cell(self):
        """Clear the ``car`` reference on whatever cell we're sitting on."""
        holder = self._holder_at(self.pos)
        if holder:
            holder.car = None

    def _remove_from_simulation(self):
        """Cleanly remove this car from the grid and schedule."""
        try:
            self.model.grid.remove_agent(self)
        except Exception:
            pass
        try:
            self.model.schedule.remove(self)
        except Exception:
            pass
        self.model._to_remove.discard(self)
